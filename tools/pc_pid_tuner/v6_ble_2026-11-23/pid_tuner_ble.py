# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : tools/pc_pid_tuner/v6_ble_2026-11-23/pid_tuner_ble.py
#  What    : PC PID tuner v6 (Bluetooth, v2.0 protocol)  -  live graphs, gain tuning and recorded step tests
#  Date    : 2026-11-23
#  Pairs with firmware: Drone.X v2.0
#  Run     : double-click the .bat file next to this one (Windows), or  python pid_tuner_ble.py
#  CLASSES: SeesawSim, DemoBackend, SerialBackend, BleBackend, Ring, Run, GainControl,
#    PlotStack, MainWindow
#  FUNCTIONS: clamp(), pid(), step_metrics(), apply_theme(), main()
#  Every class and function below has a '# ---- name(...)' title line with what it does.
# ==============================================================================================

# =============================================================================
#  ELC275 Capstone - PID Tuner GUI  (drone version: pitch PID, front/back motor pairs)
#  Ethan DiMeo
#
#  What it does (the professor's plan):
#    1. Pick Kp, Ki, Kd with the sliders (or type the exact number in the box).
#    2. The GUI sends the gains to the PIC32 over the serial port.
#    3. The PIC32 runs the PID every 4 ms, records angle + motor command,
#       and sends the data back.
#    4. The GUI graphs it so you can SEE if Kp is too small (slow, never gets
#       there), too big (oscillates), or about right. Repeat until it looks good.
#
#  Two modes:
#    DEMO    a simulated one-axis seesaw runs the SAME PID math as pid.c,
#            so you can practice tuning with no hardware plugged in.
#    SERIAL  talks to the PIC32 through the kit's USB port (UART2, 115200),
#            the same COM port PuTTY uses. This program takes PuTTY's place:
#            close PuTTY first (Windows lets only one program open a COM port).
#            The Console tab shows every line the PIC prints, like PuTTY does.
#
#  How to run on Windows:
#    double-click run_tuner.bat        (installs what it needs the first time)
#  or by hand:
#    py -m pip install PySide6 pyqtgraph numpy pyserial
#    py pid_tuner.py
#
#  The control law (same as pid.c, Kevin Lynch form), every dt = 4 ms:
#    e     = r - s                      error = setpoint - measured angle
#    eint  = eint + e*dt                clamped to +/- EINTMAX (anti-windup)
#    edot  = 0 - gyro_rate              rate of the error, from the gyro
#    u     = Kp*e + Ki*eint + Kd*edot   microseconds of correction
#    motor = clamp(1500 + u, 1000, 2000) us
#            (prof's timer counts: 300 + u, clamped 200..400, 1 count = 5 us)
#
#  Gain units:  Kp [us per deg]   Ki [us per deg*s]   Kd [us per deg/s]
#
# -----------------------------------------------------------------------------
#  SERIAL / BLE UART PROTOCOL (plain text lines ending in newline)
#
#  GUI -> PIC32
#    K,<kp>,<ki>,<kd>        set gains          (PIC replies A,<kp>,<ki>,<kd>)
#    S,<deg>                 set the setpoint
#    R,<from>,<to>,<ms>      step test: hold <from> about 1 s, then step to <to>,
#                            record every 4 ms for <ms> ms, then send it back
#    L,1   /   L,0           live data stream on / off
#    X                       KILL - motors off
#   drone extras (Motors panel, Serial mode only):
#    B,<front_us>,<back_us>  base pulse of the front pair (ESC 1+2) and back pair (ESC 3+4)
#    M,<front_us>,<back_us>  manual: PID off, front and back pairs at these pulses
#    G                       PID on, hold the setpoint (no step test)
#    Z                       zero the angle (hold the drone level first)
#    C                       calibrate: motors off, drone still + level 2 s (clears an IMU error)
#
#  PIC32 -> GUI
#    A,<kp>,<ki>,<kd>                                  gains now in use
#    D,<t_ms>,<angle>,<setpoint>,<motor_us>[,<P>,<I>,<D>[,<back_us>]]     one sample
#                                                      (angle = pitch, motor_us = front pair)
#    RUN_BEGIN   ...D lines...   RUN_END               a recorded step test
#                                                      (t_ms measured from the step,
#                                                       negative = before the step;
#                                                       the PIC sends nothing while recording)
#    anything else                                     shown in the Console tab
# =============================================================================

import sys
import time
import asyncio
import random
import threading
import queue
import csv

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

try:
    import serial
    import serial.tools.list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

try:
    from bleak import BleakClient, BleakScanner
    HAVE_BLEAK = True
except ImportError:
    HAVE_BLEAK = False


# Nordic UART Service used by the Adafruit Bluefruit LE SPI Friend.
# From the CENTRAL (laptop) point of view: 0002 is writable, 0003 notifies.
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_WRITE_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_NOTIFY_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEFAULT_BLE_NAME = ""  # auto-detect a Bluefruit advertising the Nordic UART service


# =============================================================================
#  Numbers shared with the firmware (keep these the same as control.c / pid.c)
# =============================================================================
DT = 0.004          # control loop period, s  (Timer4 = 4 ms = 250 Hz)
BASE_US = 1500.0    # motor pulse with zero correction  (prof's 300 counts)
MOTOR_MIN = 1000.0  # 1 ms = 200 counts
MOTOR_MAX = 2000.0  # 2 ms = 400 counts
EINTMAX = 20.0      # integrator clamp, degree-seconds (same as control.c)
UMAX = 500.0        # biggest correction allowed, us
US_PER_COUNT = 5.0  # prof's PWM: 200 counts per ms -> 5 us per count

# =============================================================================
#  Colors (dark theme)
# =============================================================================
SURFACE = "#1a1a19"     # graph background
PANEL = "#232322"       # window background
PANEL2 = "#2d2d2b"      # buttons / inputs
BORDER = "#3a3a37"
TEXT = "#f0efec"
TEXT2 = "#c3c2b7"
MUTED = "#8a8980"
LIMIT = "#6b6a63"       # clamp lines on the motor graph
CRITICAL = "#d03b3b"    # KILL button
WARNING = "#fab219"

# categorical colors, used in this fixed order
SERIES = ["#3987e5", "#d95926", "#199e70", "#c98500",
          "#d55181", "#008300", "#9085e9", "#e66767"]
C_ANGLE = SERIES[0]     # measured angle    (blue)
C_MOTOR = SERIES[1]     # motor command     (orange)
C_P = SERIES[2]         # P term            (aqua)
C_I = SERIES[3]         # I term            (yellow)
C_D = SERIES[4]         # D term            (magenta)
MAX_VISIBLE_RUNS = 8    # one color per visible run


# ---- clamp(x, lo, hi) ------------------------------------------------------------------------
#      Limits x to the range lo..hi.
def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


# =============================================================================
# ---- pid(e, edot, Kp, Ki, Kd, eint, eintmax, umax) -------------------------------------------
#      One PID step, the same math as the firmware's pid.c (used by the demo simulator).
#  PID - same math and same order as pid.c
#  (Python has no pointers, so eint comes back as a return value instead)
# =============================================================================
def pid(e, edot, Kp, Ki, Kd, eint, eintmax, umax):
    eint = eint + e * DT                # integrate the error
    eint = clamp(eint, -eintmax, eintmax)   # anti-windup

    P = Kp * e
    I = Ki * eint
    D = Kd * edot
    u = P + I + D                       # the PID law
    u = clamp(u, -umax, umax)           # saturate the output
    return u, eint, P, I, D


# =============================================================================
#  DEMO: simulated one-axis seesaw (an arm on a pivot with a motor/prop)
#
#  angle'' = G*(motor - 1500 - imbalance) - B*angle'
#  The motor does not change speed instantly (TAU lag), and the arm hits the
#  table at +/-40 degrees.  These numbers are made up but behave like a real
#  rig: P alone is slow when small and oscillates when big, D calms it down,
#  and I removes the leftover offset caused by the imbalance.
# =============================================================================
NOISE_LEVELS = {                         # (angle noise deg, gyro noise deg/s)
    "Off": (0.0, 0.0),
    "Low": (0.05, 1.0),
    "Medium (props spinning)": (0.15, 3.0),
    "High (bad vibration)": (0.4, 8.0),
}


# ---- class SeesawSim -------------------------------------------------------------------------
#      Simulated seesaw/drone arm (physics + the firmware's PID) for demo mode, no board needed.
class SeesawSim:
    G = 12.0        # deg/s^2 of angular acceleration per us above 1500
    B = 4.0         # friction + air damping, 1/s
    TAU = 0.05      # motor/ESC spin-up lag, s
    STOP = 40.0     # mechanical stops, deg
    SUBSTEPS = 4    # physics runs 4x faster than the controller

    # ---- SeesawSim.__init__(self) ------------------------------------------------------------
    #      Sets the simulator's physical constants and starts it level.
    def __init__(self):
        self.kp, self.ki, self.kd = 1.0, 0.0, 0.0
        self.r = 0.0                # setpoint, deg
        self.imbalance = 10.0       # us of motor needed just to hold level
        self.noise = NOISE_LEVELS["Medium (props spinning)"]
        self.t = 0.0
        self.reset(0.0)

    # ---- SeesawSim.reset(self, angle) --------------------------------------------------------
    #      Puts the simulated arm back to level and clears the PID state.
    def reset(self, angle):
        self.theta = angle          # true angle, deg
        self.omega = 0.0            # true rate, deg/s
        self.m = BASE_US            # what the motor is actually doing, us
        self.eint = 0.0
        self.armed = True

    # ---- SeesawSim.bump(self) ----------------------------------------------------------------
    #      Gives the simulated arm a sudden push (somebody flicks it).
    def bump(self):
        self.omega += 90.0          # somebody flicks the arm

    # ---- SeesawSim.step(self) ----------------------------------------------------------------
    #      Runs one 4 ms control period. Returns (t, angle, setpoint, motor, P, I, D).
    def step(self):
        """One 4 ms control period. Returns (t, angle, setpoint, motor, P, I, D)."""
        # 1. sense (the IMU is never perfect)
        s = self.theta + random.gauss(0.0, self.noise[0])
        rate = self.omega + random.gauss(0.0, self.noise[1])

        # 2. compute (identical to the firmware)
        if self.armed:
            e = self.r - s
            edot = 0.0 - rate
            u, self.eint, P, I, D = pid(e, edot, self.kp, self.ki, self.kd,
                                        self.eint, EINTMAX, UMAX)
            motor = clamp(BASE_US + u, MOTOR_MIN, MOTOR_MAX)
        else:
            self.eint = 0.0
            P = I = D = 0.0
            motor = MOTOR_MIN

        # 3. physics
        h = DT / self.SUBSTEPS
        for _ in range(self.SUBSTEPS):
            self.m += (motor - self.m) / self.TAU * h
            accel = self.G * (self.m - BASE_US - self.imbalance) - self.B * self.omega
            self.omega += accel * h
            self.theta += self.omega * h
            if self.theta > self.STOP:
                self.theta = self.STOP
                self.omega = min(self.omega, 0.0)
            if self.theta < -self.STOP:
                self.theta = -self.STOP
                self.omega = max(self.omega, 0.0)

        self.t += DT
        return (self.t, s, self.r, motor, P, I, D)


# =============================================================================
# ---- class DemoBackend -----------------------------------------------------------------------
#      Backend that talks to the simulator instead of a board (same commands as the real link).
#  Backends: the GUI talks to one of these. Both have the same functions,
#  and poll() returns a list of events:
#     ("sample", row)       live data   row = (t, angle, setpoint, motor, P, I, D)
#     ("run_begin", None)   a step test started
#     ("run_sample", row)   step-test data (t measured from the step)
#     ("run_end", None)
#     ("ack", (kp, ki, kd)) gains the controller is now using
#     ("log", text)         anything for the Console tab
# =============================================================================
class DemoBackend:
    PRE_STEP = 50           # samples recorded before the step (0.2 s)

    # ---- DemoBackend.__init__(self) ----------------------------------------------------------
    #      Sets up this object.
    def __init__(self):
        self.sim = SeesawSim()
        self.live = True
        self.run = None
        self.events = [("log", "Demo started: simulated seesaw, same PID as pid.c")]
        self.clock = time.perf_counter()
        self.acc = 0.0

    # ---- DemoBackend.describe(self) ----------------------------------------------------------
    #      Returns a short text describing the connection (shown in the status bar).
    def describe(self):
        return "Demo - simulated seesaw"

    # ---- DemoBackend.close(self) -------------------------------------------------------------
    #      Closes the connection.
    def close(self):
        pass

    # ---- DemoBackend.set_gains(self, kp, ki, kd) ---------------------------------------------
    #      Demo: the simulated seesaw uses the new gains at once and confirms them.
    def set_gains(self, kp, ki, kd):
        self.sim.kp, self.sim.ki, self.sim.kd = kp, ki, kd
        self.events.append(("ack", (kp, ki, kd)))

    # ---- DemoBackend.set_setpoint(self, r) ---------------------------------------------------
    #      Sends a new setpoint angle: S,deg.
    def set_setpoint(self, r):
        if self.run is None:
            self.sim.r = r

    # ---- DemoBackend.set_live(self, on) ------------------------------------------------------
    #      Turns the board's live data stream on or off: L,1 / L,0.
    def set_live(self, on):
        self.live = on

    # ---- DemoBackend.start_run(self, r_from, r_to, ms) ---------------------------------------
    #      Starts a recorded step test: R,from,to,ms.
    def start_run(self, r_from, r_to, ms):
        s = self.sim
        s.reset(r_from)
        s.r = r_from
        for _ in range(int(1.0 / DT)):      # hold r_from for 1 s first (not recorded)
            s.step()
        self.run = {"k": 0, "to": r_to, "n": self.PRE_STEP + int(ms / 1000.0 / DT)}
        self.events.append(("run_begin", None))

    # ---- DemoBackend.kill(self) --------------------------------------------------------------
    #      Sends X = KILL: motors off at once.
    def kill(self):
        self.sim.armed = False
        if self.run is not None:
            self.run = None
            self.events.append(("run_end", None))
        self.events.append(("log", "KILL - motors off. Press Reset sim (or start a run) to re-arm."))

    # ---- DemoBackend.bump(self) --------------------------------------------------------------
    #      Pushes the simulated arm.
    def bump(self):
        self.sim.bump()

    # ---- DemoBackend.reset(self) -------------------------------------------------------------
    #      Resets the simulator.
    def reset(self):
        r = self.sim.r
        self.sim.reset(0.0)
        self.sim.r = r

    # ---- DemoBackend.poll(self) --------------------------------------------------------------
    #      Returns every event (data lines, acks, status) received since the last call.
    def poll(self):
        # run the simulation in real time: however many 4 ms steps have passed
        now = time.perf_counter()
        self.acc += now - self.clock
        self.clock = now
        n = int(self.acc / DT)
        self.acc -= n * DT
        if n > 250:                         # window was frozen; don't try to catch up
            n = 250
            self.acc = 0.0

        out = self.events
        self.events = []
        for _ in range(n):
            run = self.run
            if run is not None and run["k"] >= self.PRE_STEP:
                self.sim.r = run["to"]          # the step happens here
            row = self.sim.step()
            if self.live:
                out.append(("sample", row))
            if run is not None:
                t_rel = (run["k"] - self.PRE_STEP) * DT
                out.append(("run_sample", (t_rel,) + row[1:]))
                run["k"] += 1
                if run["k"] >= run["n"]:
                    self.run = None
                    out.append(("run_end", None))
        return out


# ---- class SerialBackend ---------------------------------------------------------------------
#      Backend for the USB serial link (PuTTY's COM port) to the PIC32.
class SerialBackend:
    # ---- SerialBackend.__init__(self, port, baud) --------------------------------------------
    #      Sets up this object.
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.port = port
        self.baud = baud
        self.lines = queue.Queue()
        self.events = [("log", f"Opened {port} at {baud} baud")]
        self.in_run = False
        self.motor_b = None             # last motor B pulse (9th field of a D line)
        self.alive = True
        self.armed = False
        self.last_heartbeat = time.monotonic()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    # ---- SerialBackend.describe(self) --------------------------------------------------------
    #      Returns a short text describing the connection (shown in the status bar).
    def describe(self):
        return f"Serial - {self.port} @ {self.baud}"

    # ---- SerialBackend._reader(self) ---------------------------------------------------------
    #      Background thread: reads lines from the serial port and queues them for poll().
    # background thread: read bytes, cut them into lines
    def _reader(self):
        buf = b""
        while self.alive:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as ex:
                self.lines.put("!ERR " + str(ex))
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.lines.put(line.decode(errors="replace").strip())

    # ---- SerialBackend.close(self) -----------------------------------------------------------
    #      Closes the connection.
    def close(self):
        if self.armed:
            try:
                self.kill()
                time.sleep(0.03)
            except Exception:
                pass
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- SerialBackend.send(self, text) ------------------------------------------------------
    #      Sends one command line to the board.
    def send(self, text):
        self.ser.write((text + "\n").encode())
        self.events.append(("log", "> " + text))

    # ---- SerialBackend.set_gains(self, kp, ki, kd) -------------------------------------------
    #      Sends new gains: K,kp,ki,kd.
    def set_gains(self, kp, ki, kd):
        self.send(f"K,{kp:.4f},{ki:.4f},{kd:.4f}")

    # ---- SerialBackend.set_setpoint(self, r) -------------------------------------------------
    #      Sends a new setpoint angle: S,deg.
    def set_setpoint(self, r):
        self.send(f"S,{r:.2f}")

    # ---- SerialBackend.set_live(self, on) ----------------------------------------------------
    #      Turns the board's live data stream on or off: L,1 / L,0.
    def set_live(self, on):
        self.send("L,1" if on else "L,0")

    # ---- SerialBackend.start_run(self, r_from, r_to, ms) -------------------------------------
    #      Starts a recorded step test: R,from,to,ms.
    def start_run(self, r_from, r_to, ms):
        self.send(f"R,{r_from:.2f},{r_to:.2f},{int(ms)}")

    # ---- SerialBackend.arm(self) -------------------------------------------------------------
    #      Sends ARM (the board checks everything, then closes the motor-power relay).
    def arm(self):
        self.send("ARM")
        self.armed = True
        self.last_heartbeat = time.monotonic()

    # ---- SerialBackend.kill(self) ------------------------------------------------------------
    #      Sends X = KILL: motors off at once.
    def kill(self):
        self.armed = False
        self.send("X")

    # ---- SerialBackend.poll(self) ------------------------------------------------------------
    #      Returns every event (data lines, acks, status) received since the last call.
    def poll(self):
        if self.armed and time.monotonic() - self.last_heartbeat >= 0.10:
            self.send("H")
            self.last_heartbeat = time.monotonic()
        out = self.events
        self.events = []
        while True:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            if line.startswith("!ERR"):
                raise IOError(line[5:])
            if not line:
                continue
            f = line.split(",")
            try:
                if f[0] == "D" and len(f) >= 5:
                    v = [float(x) for x in f[1:8]]
                    while len(v) < 7:
                        v.append(float("nan"))      # no P/I/D sent
                    row = (v[0] / 1000.0, v[1], v[2], v[3], v[4], v[5], v[6])
                    if len(f) >= 9:
                        self.motor_b = float(f[8])
                    out.append(("run_sample" if self.in_run else "sample", row))
                    continue
                if f[0] == "A" and len(f) == 4:
                    out.append(("ack", (float(f[1]), float(f[2]), float(f[3]))))
                    continue
            except ValueError:
                pass                                # garbled line -> console
            if line.startswith("DISARMED"):
                self.armed = False              # the PIC turned the motors off: stop the heartbeat
                out.append(("disarmed", line))
            elif line.startswith("ARMED"):
                out.append(("armed", line))
            elif line == "RUN_BEGIN":
                self.in_run = True
                out.append(("run_begin", None))
            elif line == "RUN_END":
                self.in_run = False
                out.append(("run_end", None))
            else:
                out.append(("log", line))
        return out


# ---- class BleBackend ------------------------------------------------------------------------
#      Backend for the Bluetooth LE link to the Bluefruit (Nordic UART service, via bleak).
class BleBackend:
    """BLE UART backend for Adafruit Bluefruit LE SPI Friend.

    The Bluefruit is the BLE peripheral. This PC is the BLE central:
      write  -> Nordic UART UUID 0002
      notify <- Nordic UART UUID 0003
    """

    # ---- BleBackend.__init__(self, name_or_address) ------------------------------------------
    #      Sets up this object.
    def __init__(self, name_or_address=DEFAULT_BLE_NAME):
        if not HAVE_BLEAK:
            raise RuntimeError("Missing bleak. Run: py -m pip install bleak")
        self.name_or_address = name_or_address.strip() or DEFAULT_BLE_NAME
        self.lines = queue.Queue()
        self.events = [("log", "Scanning for Bluefruit BLE UART...")]
        self.in_run = False
        self.motor_b = None
        self.armed = False
        self.last_heartbeat = time.monotonic()
        self.t0 = time.monotonic()
        self.rd_pre, self.rd_from, self.rd_to, self.rd_bf, self.rd_bb = 50, 0.0, 0.0, 1500.0, 1500.0
        self._rx = bytearray()
        self._alive = True
        self._write_lock = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop_thread, daemon=True)
        self._thread.start()
        future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        try:
            self.device_name, self.address = future.result(timeout=20.0)
        except Exception:
            self._alive = False
            self._loop.call_soon_threadsafe(self._loop.stop)
            raise

    # ---- BleBackend._loop_thread(self) -------------------------------------------------------
    #      Background thread running the asyncio loop that owns the BLE connection.
    def _loop_thread(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    # ---- BleBackend._connect(self) -----------------------------------------------------------
    #      Scans for the Bluefruit (Nordic UART service), connects and subscribes to its TX
    #      notifications.
    async def _connect(self):
        target = self.name_or_address.casefold()
        found = None
        # Prefer the Nordic UART service advertisement; the Bluefruit keeps its
        # factory device name unless the module is explicitly renamed over AT.
        try:
            discovered = await BleakScanner.discover(timeout=7.0, return_adv=True)
            devices = []
            service_matches = []
            for _addr, pair in discovered.items():
                d, adv = pair
                devices.append(d)
                uuids = [u.casefold() for u in (adv.service_uuids or [])]
                if NUS_SERVICE_UUID in uuids:
                    service_matches.append(d)
                nm = (d.name or "").strip()
                if target and (d.address.casefold() == target or nm.casefold() == target):
                    found = d
            if found is None and not target and len(service_matches) == 1:
                found = service_matches[0]
            if found is None and not target:
                bluefruit_names = [d for d in devices if "bluefruit" in (d.name or "").casefold()]
                if len(bluefruit_names) == 1:
                    found = bluefruit_names[0]
            if found is None and target:
                matches = [d for d in devices if target in (d.name or "").casefold()]
                if len(matches) == 1:
                    found = matches[0]
        except TypeError:
            devices = await BleakScanner.discover(timeout=7.0)
            if target:
                matches = [d for d in devices if d.address.casefold() == target or
                           target in (d.name or "").casefold()]
                if len(matches) == 1:
                    found = matches[0]
        if found is None:
            names = ", ".join(f"{d.name or 'Unknown'} ({d.address})" for d in devices)
            raise RuntimeError(
                "Could not uniquely identify the Bluefruit UART device. "
                "Leave the name blank when only one UART device is nearby, or enter its BLE name/address. "
                f"Nearby devices: {names or 'none'}"
            )

        self._write_lock = asyncio.Lock()
        self.client = BleakClient(found, disconnected_callback=self._on_disconnect)
        await self.client.connect(timeout=10.0)
        await self.client.start_notify(NUS_NOTIFY_UUID, self._on_notify)
        self._device_name = found.name or self.name_or_address
        self.events.append(("log", f"BLE connected to {self._device_name} ({found.address})"))
        return self._device_name, found.address

    # ---- BleBackend._on_notify(self, _sender, data) ------------------------------------------
    #      Called for every BLE notification: joins the bytes into text lines.
    def _on_notify(self, _sender, data):
        # Bleak callback may receive fragments; line reconstruction is in poll().
        self.lines.put(bytes(data))

    # ---- BleBackend._on_disconnect(self, _client) --------------------------------------------
    #      Called when the BLE link drops: reports it to the window.
    def _on_disconnect(self, _client):
        self.armed = False
        self.lines.put(b"!ERR BLE disconnected")

    # ---- BleBackend.describe(self) -----------------------------------------------------------
    #      Returns a short text describing the connection (shown in the status bar).
    def describe(self):
        return f"Bluetooth LE - {self.device_name} ({self.address})"

    # ---- BleBackend._write_bytes(self, data) -------------------------------------------------
    #      Writes bytes to the Bluefruit's RX characteristic in 20-byte pieces.
    async def _write_bytes(self, data):
        if not self.client.is_connected:
            raise IOError("BLE link is disconnected")
        async with self._write_lock:
            # Conservative default ATT payload: 20 bytes (MTU 23).
            for off in range(0, len(data), 20):
                await self.client.write_gatt_char(
                    NUS_WRITE_UUID, data[off:off + 20], response=False
                )

    # ---- BleBackend.send(self, text) ---------------------------------------------------------
    #      Sends one command line to the board.
    def send(self, text):
        payload = (text.rstrip("\r\n") + "\n").encode("ascii")
        future = asyncio.run_coroutine_threadsafe(self._write_bytes(payload), self._loop)
        # ---- done(f) -------------------------------------------------------------------------
        #      Callback that hands the finished BLE operation's result back to the GUI thread.
        def done(f):
            try:
                f.result()
                self.events.append(("log", "> " + text))
            except Exception as ex:
                self.lines.put(("!ERR " + str(ex)).encode())
        future.add_done_callback(done)

    # ---- BleBackend.set_gains(self, kp, ki, kd) ----------------------------------------------
    #      Sends new gains: K,kp,ki,kd.
    def set_gains(self, kp, ki, kd):
        self.send(f"K,{kp:.4f},{ki:.4f},{kd:.4f}")

    # ---- BleBackend.set_setpoint(self, r) ----------------------------------------------------
    #      Sends a new setpoint angle: S,deg.
    def set_setpoint(self, r):
        self.send(f"S,{r:.2f}")

    # ---- BleBackend.set_live(self, on) -------------------------------------------------------
    #      Turns the board's live data stream on or off: L,1 / L,0.
    def set_live(self, on):
        self.send("L,1" if on else "L,0")

    # ---- BleBackend.start_run(self, r_from, r_to, ms) ----------------------------------------
    #      Starts a recorded step test: R,from,to,ms.
    def start_run(self, r_from, r_to, ms):
        self.send(f"R,{r_from:.2f},{r_to:.2f},{int(ms)}")

    # ---- BleBackend.arm(self) ----------------------------------------------------------------
    #      Sends ARM (the board checks everything, then closes the motor-power relay).
    def arm(self):
        self.send("ARM")
        self.armed = True
        self.last_heartbeat = time.monotonic()

    # ---- BleBackend.kill(self) ---------------------------------------------------------------
    #      Sends X = KILL: motors off at once.
    def kill(self):
        self.armed = False
        self.send("X")

    # ---- BleBackend.poll(self) ---------------------------------------------------------------
    #      Returns every event (data lines, acks, status) received since the last call.
    def poll(self):
        if self.armed and time.monotonic() - self.last_heartbeat >= 0.10:
            self.send("H")
            self.last_heartbeat = time.monotonic()

        out = self.events
        self.events = []
        while True:
            try:
                item = self.lines.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, bytes):
                if item.startswith(b"!ERR "):
                    raise IOError(item[5:].decode(errors="replace"))
                self._rx.extend(item)
                while b"\n" in self._rx:
                    line, _, rest = self._rx.partition(b"\n")
                    self._rx = bytearray(rest)
                    text = line.decode(errors="replace").strip()
                    if text:
                        self._parse_line(text, out)
            else:
                out.append(("log", str(item)))
        return out

    # ---- BleBackend._parse_line(self, line, out) ---------------------------------------------
    #      Turns one received text line (D, RD, P, RUN_BEGIN/RUN_END, status) into an event.
    def _parse_line(self, line, out):
        # Firmware 2.0 sends short lines over Bluetooth (the link is only ~100-300 bytes/s):
        #   RUN_BEGIN,samples,pre,from,to,base_f,base_b,kp,ki,kd
        #   RD,k,a,u,a,u,...   a = pitch x 10, u = PID output (us), 8 samples a line, 4 ms apart
        #   P,pitch x 10,roll x 10,armed      live angle while the motors are off
        f = line.split(",")
        nan = float("nan")
        try:
            if f[0] == "RUN_BEGIN" and len(f) >= 7:
                self.rd_pre = int(f[2])
                self.rd_from = float(f[3])
                self.rd_to = float(f[4])
                self.rd_bf = float(f[5])
                self.rd_bb = float(f[6])
                self.in_run = True
                out.append(("run_begin", None))
                return
            if f[0] == "RD" and len(f) >= 4 and self.in_run:
                k = int(f[1])
                for i in range(2, len(f) - 1, 2):
                    a = int(f[i]) / 10.0
                    u = float(f[i + 1])
                    sp = self.rd_from if k < self.rd_pre else self.rd_to
                    front = clamp(self.rd_bf + u, 1100.0, MOTOR_MAX)    # same mixer as the PIC
                    self.motor_b = clamp(self.rd_bb - u, 1100.0, MOTOR_MAX)
                    out.append(("run_sample", ((k - self.rd_pre) * 0.004, a, sp, front, nan, nan, nan)))
                    k += 1
                return
            if f[0] == "P" and len(f) >= 4:
                out.append(("sample", (time.monotonic() - self.t0, int(f[1]) / 10.0, nan, nan, nan, nan, nan)))
                return
            if f[0] == "D" and len(f) >= 5:
                v = [float(x) for x in f[1:8]]
                while len(v) < 7:
                    v.append(float("nan"))
                row = (v[0] / 1000.0, v[1], v[2], v[3], v[4], v[5], v[6])
                if len(f) >= 9:
                    self.motor_b = float(f[8])
                out.append(("run_sample" if self.in_run else "sample", row))
                return
            if f[0] == "A" and len(f) == 4:
                out.append(("ack", (float(f[1]), float(f[2]), float(f[3]))))
                return
        except ValueError:
            pass
        if line.startswith("DISARMED"):
            self.armed = False                  # the PIC turned the motors off: stop the heartbeat
            out.append(("disarmed", line))
        elif line.startswith("ARMED"):
            out.append(("armed", line))
        elif line == "RUN_BEGIN":
            self.in_run = True
            out.append(("run_begin", None))
        elif line == "RUN_END":
            self.in_run = False
            out.append(("run_end", None))
        else:
            out.append(("log", line))

    # ---- BleBackend.close(self) --------------------------------------------------------------
    #      Closes the connection.
    def close(self):
        if self.armed:
            try:
                self.kill()
                time.sleep(0.08)
            except Exception:
                pass
        self._alive = False
        try:
            future = asyncio.run_coroutine_threadsafe(self.client.disconnect(), self._loop)
            future.result(timeout=3.0)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=1.0)


# =============================================================================
# ---- class Ring ------------------------------------------------------------------------------
#      Ring buffer of the most recent live samples (time, angle, setpoint, motors, P/I/D).
#  Ring buffer for live data (keeps the last 2 minutes)
# =============================================================================
class Ring:
    # ---- Ring.__init__(self, cap, ncol) ------------------------------------------------------
    #      Makes an empty buffer that holds cap samples.
    def __init__(self, cap, ncol=7):
        self.cap = cap
        self.buf = np.zeros((cap, ncol))
        self.n = 0

    # ---- Ring.clear(self) --------------------------------------------------------------------
    #      Empties the buffer.
    def clear(self):
        self.n = 0

    # ---- Ring.last_t(self) -------------------------------------------------------------------
    #      Time of the newest sample.
    def last_t(self):
        if self.n == 0:
            return None
        return self.buf[(self.n - 1) % self.cap, 0]

    # ---- Ring.extend(self, rows) -------------------------------------------------------------
    #      Adds new samples (oldest ones fall off when full).
    def extend(self, rows):
        a = np.asarray(rows, dtype=float)
        k = len(a)
        if k > self.cap:
            a = a[-self.cap:]
            k = self.cap
        i = self.n % self.cap
        first = min(k, self.cap - i)
        self.buf[i:i + first] = a[:first]
        self.buf[:k - first] = a[first:]
        self.n += k

    # ---- Ring.last_seconds(self, sec) --------------------------------------------------------
    #      Returns the samples from the last sec seconds, oldest first.
    def last_seconds(self, sec):
        if self.n == 0:
            return np.zeros((0, self.buf.shape[1]))
        if self.n <= self.cap:
            data = self.buf[:self.n]
        else:
            i = self.n % self.cap
            data = np.concatenate((self.buf[i:], self.buf[:i]))
        start = np.searchsorted(data[:, 0], data[-1, 0] - sec)
        return data[start:]


# =============================================================================
# ---- class Run -------------------------------------------------------------------------------
#      One recorded step test: its gains, settings and every 4 ms sample.
#  One recorded step test
# =============================================================================
class Run:
    # ---- Run.__init__(self, num, kp, ki, kd, r_from, r_to) -----------------------------------
    #      Makes a new run with its number, gains and step settings.
    def __init__(self, num, kp, ki, kd, r_from, r_to):
        self.num = num
        self.kp, self.ki, self.kd = kp, ki, kd
        self.r_from, self.r_to = r_from, r_to
        self.rows = []
        self.data = None            # numpy array when finished
        self.metrics = None
        self.color = SERIES[(num - 1) % len(SERIES)]
        self.visible = True
        self.curve = None

    # ---- Run.array(self) ---------------------------------------------------------------------
    #      Returns the run's samples as a numpy array.
    def array(self):
        if self.data is not None:
            return self.data
        if not self.rows:
            return np.zeros((0, 7))
        return np.asarray(self.rows, dtype=float)

    # ---- Run.label(self) ---------------------------------------------------------------------
    #      Text label for the run list: run number and gains.
    def label(self):
        return f"Run {self.num}:  Kp {self.kp:g}  Ki {self.ki:g}  Kd {self.kd:g}"


# ---- step_metrics(run) -----------------------------------------------------------------------
#      Works out rise time, overshoot, settling time, steady-state error and % time saturated
#      for a run.
def step_metrics(run):
    """Rise time, overshoot, settling time, steady-state error, % time saturated."""
    d = run.array()
    m = {"rise": "—", "overshoot": "—", "settle": "—", "sserr": "—", "sat": "—"}
    if len(d) < 10:
        return m
    t, y, motor = d[:, 0], d[:, 1], d[:, 3]
    after = t >= 0
    t, y, motor = t[after], y[after], motor[after]
    if len(t) < 10:
        return m
    # light smoothing (7 samples = 28 ms) so sensor noise doesn't fake the numbers
    ys = np.convolve(y, np.ones(7) / 7, mode="same")
    ys[:3], ys[-3:] = y[:3], y[-3:]
    y = ys

    sat = np.mean((motor <= MOTOR_MIN + 0.5) | (motor >= MOTOR_MAX - 0.5)) * 100
    m["sat"] = f"{sat:.0f} %"
    tail = t >= t[-1] - 0.5                         # last half second
    m["sserr"] = f"{np.mean(run.r_to - y[tail]):+.2f}°"

    step = run.r_to - run.r_from
    if abs(step) < 0.5:                             # no real step: just holding
        return m
    n = (y - run.r_from) / step                     # 0 = start, 1 = setpoint
    if (n >= 0.9).any():
        t10 = t[np.argmax(n >= 0.1)]
        t90 = t[np.argmax(n >= 0.9)]
        m["rise"] = f"{t90 - t10:.3f} s"
        m["overshoot"] = f"{max(0.0, (n.max() - 1.0) * 100):.0f} %"
    else:
        m["rise"] = "never"
        m["overshoot"] = f"reached {n.max() * 100:.0f} %"

    # settled = stays within +/-2% of where it ends up (at least 0.5 deg)
    final = np.mean(y[tail])
    band = max(0.02 * abs(final - run.r_from), 0.5)
    outside = np.abs(y - final) > band
    if outside[-1]:
        m["settle"] = "no"
    elif not outside.any():
        m["settle"] = f"{t[0]:.3f} s"
    else:
        last = np.nonzero(outside)[0][-1]
        m["settle"] = f"{t[last + 1]:.3f} s"
    return m


# =============================================================================
# ---- class GainControl -----------------------------------------------------------------------
#      One gain row (Kp, Ki or Kd): slider + number box + step size, with the applied value
#      shown.
#  One gain: label + slider + number box (type exact values) + step size
# =============================================================================
class GainControl(QtWidgets.QFrame):
    changed = QtCore.Signal()
    TICKS = 1000                    # slider resolution

    # ---- GainControl.__init__(self, name, units, value, slider_max, step, tip) ---------------
    #      Builds the slider, number box, step box and 'applied' label for one gain.
    def __init__(self, name, units, value, slider_max, step, tip):
        super().__init__()
        self.name = name
        self.setObjectName("gain")

        title = QtWidgets.QLabel(f"<b style='font-size:15px'>{name}</b>"
                                 f"&nbsp;&nbsp;<span style='color:{MUTED}'>{units}</span>")
        title.setToolTip(tip)
        self.applied = QtWidgets.QLabel("applied —")
        self.applied.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, self.TICKS)
        self.slider.setToolTip(tip)

        # the number box: type an exact value + Enter, or use the arrows /
        # mouse wheel / up-down keys to nudge it by the step size
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setDecimals(3)
        self.spin.setRange(0.0, 10000.0)
        self.spin.setKeyboardTracking(False)        # only send when you press Enter
        self.spin.setMinimumWidth(96)
        self.spin.setToolTip("Type an exact value and press Enter.\n"
                             "Arrows / mouse wheel / ↑↓ keys nudge by the step size.")

        self.step_combo = QtWidgets.QComboBox()
        self.step_combo.addItems(["1", "0.1", "0.01", "0.001"])
        self.step_combo.setToolTip("How much one arrow click / slider notch changes it")

        self.max_spin = QtWidgets.QDoubleSpinBox()
        self.max_spin.setDecimals(3)
        self.max_spin.setRange(0.001, 10000.0)
        self.max_spin.setPrefix("max ")
        self.max_spin.setMaximumWidth(112)
        self.max_spin.setToolTip("Right end of the slider. Smaller = finer slider.")

        grid = QtWidgets.QGridLayout(self)
        grid.setContentsMargins(8, 6, 8, 6)
        grid.setVerticalSpacing(4)
        grid.addWidget(title, 0, 0, 1, 2)
        grid.addWidget(self.applied, 0, 2, 1, 2)
        grid.addWidget(self.slider, 1, 0, 1, 4)
        grid.addWidget(self.spin, 2, 0)
        grid.addWidget(QtWidgets.QLabel("step"), 2, 1, alignment=QtCore.Qt.AlignRight)
        grid.addWidget(self.step_combo, 2, 2)
        grid.addWidget(self.max_spin, 2, 3)
        grid.setColumnStretch(0, 2)

        self.max_spin.setValue(slider_max)
        self.step_combo.setCurrentText(f"{step:g}")
        self.spin.setSingleStep(step)
        self.spin.setValue(value)
        self._sync_slider()
        self._applied_value = None
        self._refresh_applied()

        self.slider.valueChanged.connect(self._slider_moved)
        self.spin.valueChanged.connect(self._spin_changed)
        self.step_combo.currentTextChanged.connect(self._step_changed)
        self.max_spin.valueChanged.connect(lambda _: self._sync_slider())

    # ---- GainControl.value(self) -------------------------------------------------------------
    #      The gain currently set in the box.
    def value(self):
        return round(self.spin.value(), 3)

    # ---- GainControl.set_value(self, v) ------------------------------------------------------
    #      Sets the gain (box and slider move together).
    def set_value(self, v):
        self.spin.setValue(v)

    # ---- GainControl.step(self) --------------------------------------------------------------
    #      The step size used by the arrows.
    def step(self):
        return float(self.step_combo.currentText())

    # ---- GainControl._slider_moved(self, pos) ------------------------------------------------
    #      Slider moved: copy its value into the number box.
    def _slider_moved(self, pos):
        v = pos / self.TICKS * self.max_spin.value()
        v = round(round(v / self.step()) * self.step(), 3)     # snap to the step size
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)
        self._refresh_applied()
        self.changed.emit()

    # ---- GainControl._spin_changed(self, v) --------------------------------------------------
    #      Number box changed: move the slider to match.
    def _spin_changed(self, v):
        if v > self.max_spin.value():               # typed past the slider: stretch it
            self.max_spin.setValue(v * 1.5)
        self._sync_slider()
        self._refresh_applied()
        self.changed.emit()

    # ---- GainControl._step_changed(self, text) -----------------------------------------------
    #      Step size changed: update the arrows' step.
    def _step_changed(self, text):
        self.spin.setSingleStep(float(text))

    # ---- GainControl._sync_slider(self) ------------------------------------------------------
    #      Re-scales the slider to the current maximum.
    def _sync_slider(self):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(self.spin.value() / self.max_spin.value() * self.TICKS)))
        self.slider.blockSignals(False)

    # ---- GainControl.set_applied(self, v) ----------------------------------------------------
    #      Records the value the board confirmed (shown next to the box).
    def set_applied(self, v):
        self._applied_value = v
        self._refresh_applied()

    # ---- GainControl._refresh_applied(self) --------------------------------------------------
    #      Highlights the row if the box differs from what the board is using.
    def _refresh_applied(self):
        a = self._applied_value
        if a is None:
            self.applied.setText(f"<span style='color:{MUTED}'>applied —</span>")
        elif abs(a - self.value()) < 1e-6:
            self.applied.setText(f"<span style='color:{MUTED}'>applied</span> "
                                 f"<span style='color:{TEXT}'>{a:g}</span>")
        else:
            self.applied.setText(f"<span style='color:{WARNING}'>● not sent</span> "
                                 f"<span style='color:{MUTED}'>applied {a:g}</span>")


# =============================================================================
# ---- class PlotStack -------------------------------------------------------------------------
#      The stacked graphs (angle/setpoint, motor pulses, P/I/D terms) with a shared time axis.
#  A stack of 3 graphs sharing the time axis: angle / motor / P-I-D terms
# =============================================================================
class PlotStack:
    # ---- PlotStack.__init__(self, xlabel, legend_angle) --------------------------------------
    #      Builds the stacked graphs and their curves.
    def __init__(self, xlabel, legend_angle=True):
        self.widget = pg.GraphicsLayoutWidget()
        self.widget.ci.setSpacing(4)

        self.angle = self.widget.addPlot(row=0, col=0)
        self.motor = self.widget.addPlot(row=1, col=0)
        self.terms = self.widget.addPlot(row=2, col=0)
        self.plots = [self.angle, self.motor, self.terms]

        self.angle.setLabel("left", "Pitch (°)")
        self.motor.setLabel("left", "Front pair (µs)")
        self.terms.setLabel("left", "P / I / D (µs)")
        self.terms.setLabel("bottom", xlabel)
        for p in self.plots:
            p.showGrid(x=True, y=True, alpha=0.12)
            p.getAxis("left").setWidth(64)
            p.setClipToView(True)
            p.setDownsampling(auto=True, mode="peak")
            p.setMenuEnabled(True)
        self.motor.setXLink(self.angle)
        self.terms.setXLink(self.angle)
        self.angle.getAxis("bottom").setStyle(showValues=False)
        self.motor.getAxis("bottom").setStyle(showValues=False)

        lay = self.widget.ci.layout
        lay.setRowStretchFactor(0, 5)
        lay.setRowStretchFactor(1, 3)
        lay.setRowStretchFactor(2, 3)

        legend = {"offset": (-10, 10), "labelTextColor": TEXT2,
                  "brush": pg.mkBrush(26, 26, 25, 220), "pen": pg.mkPen(BORDER)}
        if legend_angle:
            self.angle.addLegend(**legend)
        self.motor.addLegend(**legend)
        self.terms.addLegend(**legend)

        # clamp lines = the prof's 200..400 counts. The motor graph zooms to the
        # data; the clamp lines come into view when the motor actually hits them.
        lim = pg.mkPen(LIMIT, width=1, style=QtCore.Qt.DashLine)
        below = {"position": 0.01, "color": MUTED, "anchors": [(0, 0), (0, 0)]}
        above = {"position": 0.01, "color": MUTED, "anchors": [(0, 1), (0, 1)]}
        self.motor.addItem(pg.InfiniteLine(MOTOR_MAX, angle=0, pen=lim,
                                           label="2000 µs = 400 counts (max)", labelOpts=below),
                           ignoreBounds=True)
        self.motor.addItem(pg.InfiniteLine(MOTOR_MIN, angle=0, pen=lim,
                                           label="1000 µs = 200 counts (min)", labelOpts=above),
                           ignoreBounds=True)
        self.motor.addItem(pg.InfiniteLine(BASE_US, angle=0,
                                           pen=pg.mkPen(LIMIT, width=1, style=QtCore.Qt.DotLine)),
                           ignoreBounds=True)
        self.terms.addItem(pg.InfiniteLine(0, angle=0,
                                           pen=pg.mkPen(LIMIT, width=1, style=QtCore.Qt.DotLine)),
                           ignoreBounds=True)
        self.motor.setLimits(yMin=MOTOR_MIN - 50, yMax=MOTOR_MAX + 50, minYRange=100)
        self.terms.setLimits(minYRange=20)
        self.xlabel = xlabel

        self.c_motor = self.motor.plot(pen=pg.mkPen(C_MOTOR, width=2), name="Front pair command")
        self.c_P = self.terms.plot(pen=pg.mkPen(C_P, width=2), name="P = Kp·e")
        self.c_I = self.terms.plot(pen=pg.mkPen(C_I, width=2), name="I = Ki·∫e dt")
        self.c_D = self.terms.plot(pen=pg.mkPen(C_D, width=2), name="D = Kd·ė")

        # hover cursor: a vertical line on every graph + a dot on the angle
        self.vlines = []
        for p in self.plots:
            line = pg.InfiniteLine(angle=90, movable=False,
                                   pen=pg.mkPen(TEXT2, width=1, style=QtCore.Qt.DotLine))
            line.setVisible(False)
            line.setZValue(50)
            p.addItem(line, ignoreBounds=True)
            self.vlines.append(line)
        self.dot = pg.ScatterPlotItem(size=10, pen=pg.mkPen(SURFACE, width=2),
                                      brush=pg.mkBrush(TEXT))
        self.dot.setZValue(60)
        self.angle.addItem(self.dot, ignoreBounds=True)
        self.terms_shown = True

    # ---- PlotStack.show_terms(self, on) ------------------------------------------------------
    #      Shows or hides the P/I/D graph.
    def show_terms(self, on):
        if on == self.terms_shown:
            return
        axis = self.motor.getAxis("bottom")
        if on:
            self.widget.addItem(self.terms, row=2, col=0)
            self.widget.ci.layout.setRowStretchFactor(2, 3)
            axis.setStyle(showValues=False)
            axis.showLabel(False)
        else:
            self.widget.removeItem(self.terms)
            axis.setStyle(showValues=True)
            self.motor.setLabel("bottom", self.xlabel)
        self.terms_shown = on

    # ---- PlotStack.x_at(self, scene_pos) -----------------------------------------------------
    #      Converts a mouse position into a time on the x axis.
    def x_at(self, scene_pos):
        for p in self.plots:
            if p.isVisible() and p.scene() is not None and p.sceneBoundingRect().contains(scene_pos):
                return p.vb.mapSceneToView(scene_pos).x()
        return None

    # ---- PlotStack.cursor(self, t, angle) ----------------------------------------------------
    #      Moves the vertical hover line on every graph to time x.
    def cursor(self, t, angle):
        for line in self.vlines:
            line.setPos(t)
            line.setVisible(True)
        self.dot.setData([t], [angle])


# =============================================================================
# ---- class MainWindow ------------------------------------------------------------------------
#      The tuner's main window: connection, gains, setpoint, motors, live graph, recorded runs,
#      console.
#  Main window
# =============================================================================
class MainWindow(QtWidgets.QMainWindow):
    # ---- MainWindow.__init__(self) -----------------------------------------------------------
    #      Builds the window, restores saved settings and starts the update timer (about 33
    #      times a second).
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ELC275 Capstone · PID Tuner")
        self.settings = QtCore.QSettings("ELC275", "PIDTuner")

        self.backend = None
        self.live = Ring(int(120 / DT))         # last 2 minutes
        self.live_view = np.zeros((0, 7))       # what is on screen right now
        self.runs = []
        self.current_run = None
        self.selected_run = None
        self.pending_meta = None
        self.gains_dirty = False
        self.rate_count = 0
        self.rate_t0 = time.perf_counter()

        self._build_ui()
        self._restore_settings()

        QtGui.QShortcut(QtGui.QKeySequence("Ctrl+R"), self, activated=self.start_run)
        QtGui.QShortcut(QtGui.QKeySequence("Esc"), self, activated=self.kill)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)                    # redraw about 33 times a second

        self.on_mode_changed()

    # ------------------------------------------------------------------ UI --
    # ---- MainWindow._build_ui(self) ----------------------------------------------------------
    #      Builds the whole window layout.
    def _build_ui(self):
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(split)

        # ---------------- left: tabs with the graphs ----------------
        self.tabs = QtWidgets.QTabWidget()
        split.addWidget(self.tabs)
        self.tabs.addTab(self._build_live_tab(), "Live")
        self.tabs.addTab(self._build_runs_tab(), "Step runs")
        self.tabs.addTab(self._build_console_tab(), "Console")

        # ---------------- right: controls ----------------
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(420)
        scroll.setMaximumWidth(480)
        panel = QtWidgets.QWidget()
        scroll.setWidget(panel)
        split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 0)

        col = QtWidgets.QVBoxLayout(panel)
        col.setContentsMargins(8, 8, 8, 8)
        col.setSpacing(10)
        col.addWidget(self._build_connection_box())
        col.addWidget(self._build_gains_box())
        col.addWidget(self._build_setpoint_box())
        self.motor_box = self._build_motor_box()
        col.addWidget(self.motor_box)
        col.addWidget(self._build_readings_box())
        self.demo_box = self._build_demo_box()
        col.addWidget(self.demo_box)
        col.addStretch(1)

        self.statusBar().showMessage("Ready")

    # ---- MainWindow._build_live_tab(self) ----------------------------------------------------
    #      Builds the Live tab (scrolling graphs).
    def _build_live_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)

        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Show last"))
        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(2, 60)
        self.window_spin.setValue(10)
        self.window_spin.setSuffix(" s")
        bar.addWidget(self.window_spin)
        self.pause_btn = QtWidgets.QPushButton("Pause graph")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setToolTip("Freeze the graph so you can zoom, pan and hover.\n"
                                  "Data keeps recording in the background.")
        self.pause_btn.toggled.connect(self.on_pause)
        bar.addWidget(self.pause_btn)
        self.dots_cb = QtWidgets.QCheckBox("Show every sample")
        self.dots_cb.setToolTip("Put a dot on every 4 ms data point")
        self.dots_cb.toggled.connect(self.on_dots)
        bar.addWidget(self.dots_cb)
        self.terms_cb = QtWidgets.QCheckBox("P / I / D terms")
        self.terms_cb.setChecked(True)
        self.terms_cb.toggled.connect(self.on_terms)
        bar.addWidget(self.terms_cb)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.live.clear)
        bar.addWidget(clear)
        bar.addStretch(1)
        self.rate_label = QtWidgets.QLabel("")
        self.rate_label.setStyleSheet(f"color:{MUTED}")
        bar.addWidget(self.rate_label)
        v.addLayout(bar)

        self.live_readout = self._readout_label()
        v.addWidget(self.live_readout)

        self.ls = PlotStack("Time (s)")
        self.l_angle = self.ls.angle.plot(pen=pg.mkPen(C_ANGLE, width=2), name="Measured pitch")
        self.l_sp = self.ls.angle.plot(pen=pg.mkPen(TEXT2, width=1.5, style=QtCore.Qt.DashLine),
                                       name="Setpoint")
        self.ls.widget.scene().sigMouseMoved.connect(self.on_hover_live)
        v.addWidget(self.ls.widget, 1)
        return w

    # ---- MainWindow._build_runs_tab(self) ----------------------------------------------------
    #      Builds the Runs tab (recorded step tests, table of results, overlay graphs).
    def _build_runs_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)

        self.runs_readout = self._readout_label()
        v.addWidget(self.runs_readout)

        vsplit = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        v.addWidget(vsplit, 1)

        self.rs = PlotStack("Time from step (s)")
        self.r_sp = self.rs.angle.plot(pen=pg.mkPen(TEXT2, width=1.5, style=QtCore.Qt.DashLine),
                                       name="Setpoint (selected run)")
        self.rs.widget.scene().sigMouseMoved.connect(self.on_hover_runs)
        vsplit.addWidget(self.rs.widget)

        bottom = QtWidgets.QWidget()
        bv = QtWidgets.QVBoxLayout(bottom)
        bv.setContentsMargins(0, 0, 0, 0)
        self.table = QtWidgets.QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(
            ["Show", "Run", "Kp", "Ki", "Kd", "Rise 10–90%", "Overshoot",
             "Settles (±2%)", "Offset at end", "Motor maxed"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self.on_run_selected)
        self.table.itemChanged.connect(self.on_run_item_changed)
        self.table.cellDoubleClicked.connect(lambda *_: self.use_run_gains())
        bv.addWidget(self.table, 1)

        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Use this run's gains")
        b.setToolTip("Copy the selected run's Kp/Ki/Kd back into the sliders (or double-click a row)")
        b.clicked.connect(self.use_run_gains)
        row.addWidget(b)
        b = QtWidgets.QPushButton("Export CSV…")
        b.clicked.connect(self.export_runs)
        row.addWidget(b)
        b = QtWidgets.QPushButton("Clear all runs")
        b.clicked.connect(self.clear_runs)
        row.addWidget(b)
        row.addStretch(1)
        bv.addLayout(row)
        vsplit.addWidget(bottom)
        vsplit.setSizes([620, 220])
        return w

    # ---- MainWindow._build_console_tab(self) -------------------------------------------------
    #      Builds the Console tab (raw text to and from the board).
    def _build_console_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(3000)
        self.console.setFont(self._mono())
        v.addWidget(self.console, 1)
        row = QtWidgets.QHBoxLayout()
        self.raw_edit = QtWidgets.QLineEdit()
        self.raw_edit.setPlaceholderText("Type a raw command for the PIC32, e.g.  K,2.5,0,0.4   then Enter")
        self.raw_edit.returnPressed.connect(self.send_raw)
        row.addWidget(self.raw_edit, 1)
        b = QtWidgets.QPushButton("Send")
        b.clicked.connect(self.send_raw)
        row.addWidget(b)
        v.addLayout(row)
        return w

    # ---- MainWindow._build_connection_box(self) ----------------------------------------------
    #      Builds the connection box (demo / serial / Bluetooth, port, connect button).
    def _build_connection_box(self):
        box = QtWidgets.QGroupBox("Connection")
        g = QtWidgets.QGridLayout(box)
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Demo (simulated seesaw)", "Serial → PIC32",
                                  "Bluetooth LE → Bluefruit"])
        g.addWidget(QtWidgets.QLabel("Mode"), 0, 0)
        g.addWidget(self.mode_combo, 0, 1, 1, 2)

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.refresh_btn = QtWidgets.QToolButton()
        self.refresh_btn.setText("⟳")
        self.refresh_btn.setToolTip("Look for COM ports again")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        g.addWidget(QtWidgets.QLabel("Port / BLE name"), 1, 0)
        g.addWidget(self.port_combo, 1, 1)
        g.addWidget(self.refresh_btn, 1, 2)

        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(["115200", "230400", "460800", "57600", "9600"])
        g.addWidget(QtWidgets.QLabel("Baud"), 2, 0)
        g.addWidget(self.baud_combo, 2, 1, 1, 2)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        g.addWidget(self.connect_btn, 3, 0, 1, 3)
        self.conn_label = QtWidgets.QLabel("")
        self.conn_label.setWordWrap(True)
        g.addWidget(self.conn_label, 4, 0, 1, 3)
        g.setColumnStretch(1, 1)

        self.refresh_ports()
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        return box

    # ---- MainWindow._build_gains_box(self) ---------------------------------------------------
    #      Builds the Kp/Ki/Kd rows and the Apply button.
    def _build_gains_box(self):
        box = QtWidgets.QGroupBox("PID gains")
        v = QtWidgets.QVBoxLayout(box)
        v.setSpacing(6)
        self.g_kp = GainControl("Kp", "µs per °", 1.0, 10.0, 0.1,
                                "Proportional: push back harder the farther you are from the setpoint")
        self.g_ki = GainControl("Ki", "µs per °·s", 0.0, 10.0, 0.1,
                                "Integral: slowly builds up to remove a leftover offset")
        self.g_kd = GainControl("Kd", "µs per °/s", 0.0, 2.0, 0.01,
                                "Derivative: brakes against the rotation speed, kills overshoot")
        self.gains = [self.g_kp, self.g_ki, self.g_kd]
        for gc in self.gains:
            gc.changed.connect(self.on_gain_changed)
            v.addWidget(gc)

        row = QtWidgets.QHBoxLayout()
        self.live_update_cb = QtWidgets.QCheckBox("Send while I adjust")
        self.live_update_cb.setChecked(True)
        self.live_update_cb.setToolTip("On: every change goes to the controller right away.\n"
                                       "Off: change all three, then press Apply.")
        self.live_update_cb.toggled.connect(lambda _: self._update_apply_btn())
        row.addWidget(self.live_update_cb)
        row.addStretch(1)
        self.apply_btn = QtWidgets.QPushButton("Apply gains")
        self.apply_btn.clicked.connect(self.apply_gains)
        row.addWidget(self.apply_btn)
        v.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        z = QtWidgets.QPushButton("Ki = Kd = 0  (tune Kp alone)")
        z.clicked.connect(self.zero_ki_kd)
        row.addWidget(z)
        v.addLayout(row)

        tip = QtWidgets.QLabel("Order: Kp alone until it's quick with a little overshoot → "
                               "add Kd until the overshoot is gone → add Ki only if it "
                               "stops short of the setpoint.")
        tip.setWordWrap(True)
        tip.setStyleSheet(f"color:{MUTED}; font-size:11px")
        v.addWidget(tip)
        return box

    # ---- MainWindow._build_setpoint_box(self) ------------------------------------------------
    #      Builds the setpoint and step-test controls.
    def _build_setpoint_box(self):
        box = QtWidgets.QGroupBox("Setpoint && step test")
        g = QtWidgets.QGridLayout(box)

        self.sp_spin = QtWidgets.QDoubleSpinBox()
        self.sp_spin.setRange(-40.0, 40.0)
        self.sp_spin.setDecimals(1)
        self.sp_spin.setSingleStep(0.5)
        self.sp_spin.setSuffix(" °")
        self.sp_spin.setKeyboardTracking(False)
        self.sp_spin.setToolTip("Pitch to hold. 0 = level, + = nose up.")
        self.sp_spin.valueChanged.connect(self.on_setpoint_changed)
        g.addWidget(QtWidgets.QLabel("Setpoint"), 0, 0)
        g.addWidget(self.sp_spin, 0, 1, 1, 3)

        quick = QtWidgets.QHBoxLayout()
        for val in (-15, 0, 15, 30):
            b = QtWidgets.QPushButton(f"{val:+d}°" if val else "0°")
            b.setFixedWidth(52)
            b.clicked.connect(lambda _=False, x=val: self.sp_spin.setValue(x))
            quick.addWidget(b)
        quick.addStretch(1)
        g.addLayout(quick, 1, 1, 1, 3)

        self.from_spin = QtWidgets.QDoubleSpinBox()
        self.from_spin.setRange(-40.0, 40.0)
        self.from_spin.setDecimals(1)
        self.from_spin.setSuffix(" °")
        self.from_spin.setToolTip("The test holds this angle, then jumps to the setpoint")
        self.dur_spin = QtWidgets.QDoubleSpinBox()
        self.dur_spin.setRange(0.5, 20.0)
        self.dur_spin.setDecimals(1)
        self.dur_spin.setValue(3.0)
        self.dur_spin.setSuffix(" s")
        g.addWidget(QtWidgets.QLabel("Step from"), 2, 0)
        g.addWidget(self.from_spin, 2, 1)
        g.addWidget(QtWidgets.QLabel("record"), 2, 2, alignment=QtCore.Qt.AlignRight)
        g.addWidget(self.dur_spin, 2, 3)

        self.run_btn = QtWidgets.QPushButton("▶  Start step run   (Ctrl+R)")
        self.run_btn.setObjectName("runbtn")
        self.run_btn.setMinimumHeight(36)
        self.run_btn.clicked.connect(self.start_run)
        g.addWidget(self.run_btn, 3, 0, 1, 4)

        self.arm_btn = QtWidgets.QPushButton("ARM ESC power")
        self.arm_btn.setMinimumHeight(32)
        self.arm_btn.clicked.connect(self.arm)
        g.addWidget(self.arm_btn, 4, 0, 1, 4)

        self.kill_btn = QtWidgets.QPushButton("KILL motors   (Esc)")
        self.kill_btn.setObjectName("killbtn")
        self.kill_btn.setMinimumHeight(36)
        self.kill_btn.clicked.connect(self.kill)
        g.addWidget(self.kill_btn, 5, 0, 1, 4)

        self.stream_cb = QtWidgets.QCheckBox("Stream live data")
        self.stream_cb.setChecked(True)
        self.stream_cb.toggled.connect(self.on_stream_toggled)
        g.addWidget(self.stream_cb, 6, 0, 1, 4)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(3, 1)
        return box

    # ---- MainWindow._build_motor_box(self) ---------------------------------------------------
    #      Builds the manual/base motor pulse controls.
    def _build_motor_box(self):
        box = QtWidgets.QGroupBox("Motors: front && back pairs (drone)")
        g = QtWidgets.QGridLayout(box)

        # ---- us_spin(tip) --------------------------------------------------------------------
        #      Makes a number box for a motor pulse in microseconds.
        def us_spin(tip):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(MOTOR_MIN, MOTOR_MAX)
            s.setDecimals(0)
            s.setSingleStep(10.0)
            s.setSuffix(" µs")
            s.setKeyboardTracking(False)        # only counts as changed when you press Enter
            s.setToolTip(tip)
            return s

        g.addWidget(QtWidgets.QLabel("Front (ESC 1+2)"), 0, 1, alignment=QtCore.Qt.AlignCenter)
        g.addWidget(QtWidgets.QLabel("Back (ESC 3+4)"), 0, 2, alignment=QtCore.Qt.AlignCenter)

        # base: where each pair sits while the PID runs (front = base front + u, back = base back - u)
        self.base_a = us_spin("Front pair (ESC 1+2) pulse with zero correction while the PID runs.\n"
                              "Raise it a little if the front is the weaker end.")
        self.base_b = us_spin("Back pair (ESC 3+4) pulse with zero correction while the PID runs.\n"
                              "Raise it a little if the back is the weaker end.")
        self.base_a.setValue(BASE_US)
        self.base_b.setValue(BASE_US)
        b = QtWidgets.QPushButton("Send base")
        b.setToolTip("Sends B,<front>,<back> to the PIC32")
        b.clicked.connect(self.send_base)
        g.addWidget(QtWidgets.QLabel("PID base"), 1, 0)
        g.addWidget(self.base_a, 1, 1)
        g.addWidget(self.base_b, 1, 2)
        g.addWidget(b, 1, 3)

        # manual: PID off, each pair at its own pulse
        self.man_a = us_spin("Front pair pulse in manual mode (PID off)")
        self.man_b = us_spin("Back pair pulse in manual mode (PID off)")
        self.man_a.setValue(MOTOR_MIN)
        self.man_b.setValue(MOTOR_MIN)
        b = QtWidgets.QPushButton("Run manual")
        b.setToolTip("PID off. Front and back pairs spin at these pulses (1 s soft start).\n"
                     "Sends M,<front>,<back>. KILL stops them.")
        b.clicked.connect(self.send_manual)
        g.addWidget(QtWidgets.QLabel("Manual"), 2, 0)
        g.addWidget(self.man_a, 2, 1)
        g.addWidget(self.man_b, 2, 2)
        g.addWidget(b, 2, 3)

        self.man_live_cb = QtWidgets.QCheckBox("Send manual values as I change them")
        self.man_live_cb.setToolTip("Every change of Manual front or back is sent right away.\n"
                                    "This switches the PIC to manual mode (PID off).")
        self.man_a.valueChanged.connect(self.on_manual_changed)
        self.man_b.valueChanged.connect(self.on_manual_changed)
        g.addWidget(self.man_live_cb, 3, 0, 1, 4)

        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("PID on (hold setpoint)")
        b.setToolTip("Turns the PID on without a step test (G). KILL stops it.")
        b.clicked.connect(lambda: self.send_cmd("G"))
        row.addWidget(b)
        b = QtWidgets.QPushButton("Zero angle")
        b.setToolTip("Hold the drone level, then click:\n"
                     "that position becomes 0° (Z).")
        b.clicked.connect(lambda: self.send_cmd("Z"))
        row.addWidget(b)
        b = QtWidgets.QPushButton("Calibrate")
        b.setToolTip("Motors off, then 2 s calibration: keep the drone still and level (C).\n"
                     "Also clears an IMU error, so you don't have to reflash.")
        b.clicked.connect(lambda: self.send_cmd("C"))
        row.addWidget(b)
        g.addLayout(row, 4, 0, 1, 4)

        self.motor_now = QtWidgets.QLabel("Now:  front —   ·   back —")
        self.motor_now.setFont(self._mono(11))
        g.addWidget(self.motor_now, 5, 0, 1, 4)

        g.setColumnStretch(1, 1)
        g.setColumnStretch(2, 1)
        return box

    # ---- MainWindow._build_readings_box(self) ------------------------------------------------
    #      Builds the live number readouts (angle, error, motors).
    def _build_readings_box(self):
        box = QtWidgets.QGroupBox("Right now")
        g = QtWidgets.QGridLayout(box)
        self.read_labels = {}
        names = [("angle", "Pitch"), ("error", "Error"), ("motor", "Front pair"), ("pid", "P · I · D")]
        for i, (key, text) in enumerate(names):
            name = QtWidgets.QLabel(text)
            name.setStyleSheet(f"color:{MUTED}")
            val = QtWidgets.QLabel("—")
            val.setFont(self._mono(14 if key != "pid" else 11))
            g.addWidget(name, i, 0)
            g.addWidget(val, i, 1)
            self.read_labels[key] = val
        g.setColumnStretch(1, 1)
        return box

    # ---- MainWindow._build_demo_box(self) ----------------------------------------------------
    #      Builds the demo-mode controls (imbalance, noise, bump).
    def _build_demo_box(self):
        box = QtWidgets.QGroupBox("Demo seesaw")
        g = QtWidgets.QGridLayout(box)

        self.imb_spin = QtWidgets.QDoubleSpinBox()
        self.imb_spin.setRange(-100.0, 100.0)
        self.imb_spin.setDecimals(0)
        self.imb_spin.setValue(10.0)
        self.imb_spin.setSuffix(" µs")
        self.imb_spin.setToolTip("Extra motor needed just to hold level (heavy side, weak motor).\n"
                                 "With this, P or PD alone stops short of the setpoint - Ki fixes it.")
        self.imb_spin.valueChanged.connect(self.on_demo_params)
        g.addWidget(QtWidgets.QLabel("Imbalance"), 0, 0)
        g.addWidget(self.imb_spin, 0, 1)

        self.noise_combo = QtWidgets.QComboBox()
        self.noise_combo.addItems(list(NOISE_LEVELS.keys()))
        self.noise_combo.setCurrentText("Medium (props spinning)")
        self.noise_combo.setToolTip("IMU noise. Big Kd makes the motor command shake when this is high.")
        self.noise_combo.currentTextChanged.connect(self.on_demo_params)
        g.addWidget(QtWidgets.QLabel("Sensor noise"), 1, 0)
        g.addWidget(self.noise_combo, 1, 1)

        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Bump it")
        b.setToolTip("Flick the arm to see how fast it recovers")
        b.clicked.connect(lambda: self.backend and isinstance(self.backend, DemoBackend)
                          and self.backend.bump())
        row.addWidget(b)
        b = QtWidgets.QPushButton("Reset sim")
        b.setToolTip("Put the arm back at 0° and re-arm")
        b.clicked.connect(lambda: self.backend and isinstance(self.backend, DemoBackend)
                          and self.backend.reset())
        row.addWidget(b)
        g.addLayout(row, 2, 0, 1, 2)
        g.setColumnStretch(1, 1)
        return box

    # ---- MainWindow._readout_label(self) -----------------------------------------------------
    #      Makes the hover readout label under the graphs.
    def _readout_label(self):
        lab = QtWidgets.QLabel("Hover over a graph to read the exact value of any 4 ms sample.")
        lab.setFont(self._mono(10))
        lab.setStyleSheet(f"color:{TEXT2}; padding:4px 6px; background:{SURFACE};"
                          f"border:1px solid {BORDER}; border-radius:4px")
        lab.setTextFormat(QtCore.Qt.RichText)
        lab.setWordWrap(True)
        lab.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        lab.setFixedHeight(46)
        return lab

    # ---- MainWindow._mono(size) --------------------------------------------------------------
    #      Returns a monospaced font of the given size.
    @staticmethod
    def _mono(size=10):
        f = QtGui.QFont("Consolas")
        f.setStyleHint(QtGui.QFont.Monospace)
        f.setPointSize(size)
        return f

    # ------------------------------------------------------------ settings --
    # ---- MainWindow._restore_settings(self) --------------------------------------------------
    #      Restores gain ranges and window settings saved from last time.
    def _restore_settings(self):
        s = self.settings
        for gc in self.gains:
            mx = s.value(f"{gc.name}/max", None)
            if mx is not None:
                gc.max_spin.setValue(float(mx))
            st = s.value(f"{gc.name}/step", None)
            if st is not None:
                gc.step_combo.setCurrentText(str(st))
            val = s.value(f"{gc.name}/value", None)
            if val is not None:
                gc.set_value(float(val))
        port = s.value("port", None)
        if port:
            self.port_combo.setCurrentText(str(port))
        baud = s.value("baud", None)
        if baud:
            self.baud_combo.setCurrentText(str(baud))
        for key, spin in (("motors/baseA", self.base_a), ("motors/baseB", self.base_b),
                          ("motors/manA", self.man_a), ("motors/manB", self.man_b)):
            v = s.value(key, None)
            if v is not None:
                spin.setValue(float(v))
        geo = s.value("geometry", None)
        if geo is not None:
            self.restoreGeometry(geo)
        else:
            screen = QtGui.QGuiApplication.primaryScreen().availableGeometry()
            self.resize(min(1440, int(screen.width() * 0.92)), min(900, int(screen.height() * 0.9)))
        self.gains_dirty = True

    # ---- MainWindow.closeEvent(self, ev) -----------------------------------------------------
    #      On close: saves the gains, ranges, port and motor settings for next time, then closes
    #      the link.
    def closeEvent(self, ev):
        s = self.settings
        for gc in self.gains:
            s.setValue(f"{gc.name}/value", gc.value())
            s.setValue(f"{gc.name}/max", gc.max_spin.value())
            s.setValue(f"{gc.name}/step", gc.step_combo.currentText())
        s.setValue("port", self.port_combo.currentText())
        s.setValue("baud", self.baud_combo.currentText())
        s.setValue("motors/baseA", self.base_a.value())
        s.setValue("motors/baseB", self.base_b.value())
        s.setValue("motors/manA", self.man_a.value())
        s.setValue("motors/manB", self.man_b.value())
        s.setValue("geometry", self.saveGeometry())
        self.disconnect_backend()
        super().closeEvent(ev)

    # ---------------------------------------------------------- connection --
    # ---- MainWindow.refresh_ports(self) ------------------------------------------------------
    #      Lists the serial ports again.
    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        if HAVE_SERIAL:
            for p in serial.tools.list_ports.comports():
                self.port_combo.addItem(f"{p.device} — {p.description}")
        if current:
            self.port_combo.setCurrentText(current)

    # ---- MainWindow.is_demo_mode(self) -------------------------------------------------------
    #      True if the demo (simulator) backend is selected.
    def is_demo_mode(self):
        return self.mode_combo.currentIndex() == 0

    # ---- MainWindow.on_mode_changed(self, *_) ------------------------------------------------
    #      Demo / serial / Bluetooth picked: show the right connection fields.
    def on_mode_changed(self, *_):
        self.disconnect_backend()
        demo = self.is_demo_mode()
        serial_mode = self.mode_combo.currentIndex() == 1
        ble_mode = self.mode_combo.currentIndex() == 2
        self.port_combo.setEnabled(serial_mode or ble_mode)
        self.refresh_btn.setEnabled(serial_mode)
        self.refresh_btn.setVisible(serial_mode)
        self.baud_combo.setEnabled(serial_mode)
        self.baud_combo.setVisible(serial_mode)
        self.demo_box.setVisible(demo)
        self.motor_box.setVisible(not demo)
        self.arm_btn.setVisible(not demo)
        if demo:
            self.connect_backend()
        elif serial_mode and not HAVE_SERIAL:
            self.conn_label.setText(f"<span style='color:{WARNING}'>pyserial is not installed.</span> "
                                    "Run:  py -m pip install pyserial")
        elif ble_mode and not HAVE_BLEAK:
            self.conn_label.setText(f"<span style='color:{WARNING}'>bleak is not installed.</span> "
                                    "Run:  py -m pip install bleak")
        elif ble_mode:
            self.port_combo.clear()
            self.port_combo.setEditText(DEFAULT_BLE_NAME)
            self.port_combo.setToolTip("Optional BLE name/address; leave blank to auto-detect the Nordic UART service")
        elif serial_mode:
            self.refresh_ports()
        self._update_connect_ui()

    # ---- MainWindow.on_connect_clicked(self) -------------------------------------------------
    #      Connect/Disconnect button.
    def on_connect_clicked(self):
        if self.backend is not None:
            self.disconnect_backend()
        else:
            self.connect_backend()
        self._update_connect_ui()

    # ---- MainWindow.connect_backend(self) ----------------------------------------------------
    #      Opens the chosen link and sends the current settings.
    def connect_backend(self):
        if self.is_demo_mode():
            self.backend = DemoBackend()
            self.on_demo_params()
        else:
            try:
                if self.mode_combo.currentIndex() == 1:
                    if not HAVE_SERIAL:
                        raise RuntimeError("Install pyserial: py -m pip install pyserial")
                    port = self.port_combo.currentText().split(" — ")[0].strip()
                    baud = int(self.baud_combo.currentText())
                    self.backend = SerialBackend(port, baud)
                else:
                    if not HAVE_BLEAK:
                        raise RuntimeError("Install bleak: py -m pip install bleak")
                    target = self.port_combo.currentText().strip() or DEFAULT_BLE_NAME
                    self.backend = BleBackend(target)
            except Exception as ex:
                msg = f"{ex}"
                if "denied" in str(ex).lower() or isinstance(ex, PermissionError):
                    msg += ("\n\nThe serial port is already open in another program. "
                            "Close PuTTY, then press Connect again.")
                QtWidgets.QMessageBox.warning(self, "Could not connect", msg)
                self.backend = None
                return
        self.live.clear()
        self.backend.set_live(self.stream_cb.isChecked())
        self.backend.set_setpoint(self.sp_spin.value())
        self.apply_gains()
        if isinstance(self.backend, (SerialBackend, BleBackend)):
            self.send_base()                    # PIC uses the same base pulses the GUI shows
        self._update_connect_ui()

    # ---- MainWindow.disconnect_backend(self) -------------------------------------------------
    #      Closes the link (and finishes any run that was still arriving).
    def disconnect_backend(self):
        if self.backend is not None:
            self.backend.close()
            self.log(f"Disconnected ({self.backend.describe()})")
        self.backend = None
        if self.current_run is not None:
            self.finish_run()
        for gc in self.gains:
            gc.set_applied(None)
        self._update_connect_ui()

    # ---- MainWindow._update_connect_ui(self) -------------------------------------------------
    #      Enables/disables controls to match the connection state.
    def _update_connect_ui(self):
        on = self.backend is not None
        demo = self.is_demo_mode()
        self.connect_btn.setVisible(not demo)
        self.connect_btn.setText("Disconnect" if on else "Connect")
        if on:
            self.conn_label.setText(f"<span style='color:{TEXT}'>● {self.backend.describe()}</span>")
        elif not demo:
            hint = ("Pick the kit's COM port and press Connect." if self.mode_combo.currentIndex() == 1
                    else f"Enter the Bluefruit name (default {DEFAULT_BLE_NAME}) and press Connect.")
            self.conn_label.setText(f"<span style='color:{MUTED}'>Not connected. {hint}</span>")
        self.run_btn.setEnabled(on and not isinstance(self.backend, BleBackend))
        if isinstance(self.backend, BleBackend):
            self.run_btn.setToolTip("Full-resolution step-test recordings use USB/UART2; BLE is live telemetry only.")
        else:
            self.run_btn.setToolTip("")
        self.motor_box.setEnabled(isinstance(self.backend, (SerialBackend, BleBackend)))
        self._update_apply_btn()

    # --------------------------------------------------------------- gains --
    # ---- MainWindow.on_gain_changed(self) ----------------------------------------------------
    #      A gain box changed: update the Apply button.
    def on_gain_changed(self):
        self.gains_dirty = True
        self._update_apply_btn()

    # ---- MainWindow._update_apply_btn(self) --------------------------------------------------
    #      Highlights Apply when the boxes differ from the board's gains.
    def _update_apply_btn(self):
        self.apply_btn.setEnabled(self.backend is not None and self.gains_dirty
                                  and not self.live_update_cb.isChecked())

    # ---- MainWindow.apply_gains(self) --------------------------------------------------------
    #      Sends the gains in the boxes to the board.
    def apply_gains(self):
        if self.backend is None:
            return
        try:
            self.backend.set_gains(self.g_kp.value(), self.g_ki.value(), self.g_kd.value())
        except Exception as ex:
            self.log(f"send failed: {ex}")
            return
        self.gains_dirty = False
        self._update_apply_btn()

    # ---- MainWindow.on_ack(self, g) ----------------------------------------------------------
    #      The board confirmed a command: update the 'applied' values.
    def on_ack(self, g):
        for gc, v in zip(self.gains, g):
            gc.set_applied(v)

    # ---- MainWindow.zero_ki_kd(self) ---------------------------------------------------------
    #      Sets Ki and Kd to 0 (P-only tuning).
    def zero_ki_kd(self):
        self.g_ki.set_value(0.0)
        self.g_kd.set_value(0.0)

    # -------------------------------------------------------------- motors --
    # ---- MainWindow.send_cmd(self, text) -----------------------------------------------------
    #      Sends one command and logs it in the console.
    def send_cmd(self, text):
        if not isinstance(self.backend, (SerialBackend, BleBackend)):
            self.log("(connect to the PIC32 over Serial or Bluetooth first)")
            return
        try:
            self.backend.send(text)
        except Exception as ex:
            self.log(f"send failed: {ex}")

    # ---- MainWindow.send_base(self) ----------------------------------------------------------
    #      Sends base motor pulses: B,front,back.
    def send_base(self):
        self.send_cmd(f"B,{self.base_a.value():.0f},{self.base_b.value():.0f}")

    # ---- MainWindow.send_manual(self) --------------------------------------------------------
    #      Sends manual motor pulses: M,front,back.
    def send_manual(self):
        self.send_cmd(f"M,{self.man_a.value():.0f},{self.man_b.value():.0f}")

    # ---- MainWindow.on_manual_changed(self, *_) ----------------------------------------------
    #      Manual pulse changed: send it if 'live' is ticked.
    def on_manual_changed(self, *_):
        if self.man_live_cb.isChecked() and isinstance(self.backend, (SerialBackend, BleBackend)):
            self.send_manual()

    # ------------------------------------------------------------ setpoint --
    # ---- MainWindow.on_setpoint_changed(self, v) ---------------------------------------------
    #      Sends a new setpoint.
    def on_setpoint_changed(self, v):
        if self.backend is not None:
            try:
                self.backend.set_setpoint(v)
            except Exception as ex:
                self.log(f"send failed: {ex}")

    # ---- MainWindow.on_stream_toggled(self, on) ----------------------------------------------
    #      Turns the live stream on or off.
    def on_stream_toggled(self, on):
        if self.backend is not None:
            self.backend.set_live(on)

    # ---- MainWindow.on_demo_params(self, *_) -------------------------------------------------
    #      Copies the demo controls (imbalance, noise) into the simulator.
    def on_demo_params(self, *_):
        if isinstance(self.backend, DemoBackend):
            self.backend.sim.imbalance = self.imb_spin.value()
            self.backend.sim.noise = NOISE_LEVELS[self.noise_combo.currentText()]

    # ----------------------------------------------------------- step runs --
    # ---- MainWindow.start_run(self) ----------------------------------------------------------
    #      Starts a recorded step test with the current settings.
    def start_run(self):
        if isinstance(self.backend, BleBackend):
            self.log("Step-test recordings are USB/UART2 only; BLE is reserved for control and decimated live telemetry.")
            return
        if self.backend is None or self.current_run is not None:
            return
        self.apply_gains()                      # always run with what the sliders show
        r_from = self.from_spin.value()
        r_to = self.sp_spin.value()
        if abs(r_to - r_from) < 0.5:
            self.statusBar().showMessage("Setpoint = step-from, so this run only tests holding still "
                                         "(no step).", 6000)
        self.pending_meta = (self.g_kp.value(), self.g_ki.value(), self.g_kd.value(), r_from, r_to)
        try:
            self.backend.start_run(r_from, r_to, self.dur_spin.value() * 1000)
        except Exception as ex:
            self.log(f"send failed: {ex}")
            return
        self.tabs.setCurrentIndex(1)

    # ---- MainWindow.arm(self) ----------------------------------------------------------------
    #      ARM button.
    def arm(self):
        if not isinstance(self.backend, (SerialBackend, BleBackend)):
            self.log("Connect to the PIC32 before arming.")
            return
        answer = QtWidgets.QMessageBox.warning(
            self, "Arm ESC power",
            "This energizes the ESC power relay. Confirm the aircraft is restrained, "
            "the propellers are removed for bench testing, and all throttle outputs are at minimum.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            self.backend.arm()
            self.statusBar().showMessage("ARM requested — relay closes at minimum pulse; wait 2 seconds before starting.", 7000)
        except Exception as ex:
            self.log(f"arm send failed: {ex}")

    # ---- MainWindow.kill(self) ---------------------------------------------------------------
    #      KILL button: motors off.
    def kill(self):
        if self.backend is not None:
            try:
                self.backend.kill()
            except Exception as ex:
                self.log(f"send failed: {ex}")
        self.statusBar().showMessage("KILL sent - motors off", 5000)

    # ---- MainWindow.begin_run(self) ----------------------------------------------------------
    #      The board started sending a run (RUN_BEGIN): make a new Run.
    def begin_run(self):
        if self.pending_meta is not None:
            kp, ki, kd, r_from, r_to = self.pending_meta
        else:                                   # run started from the PIC side
            kp, ki, kd = self.g_kp.value(), self.g_ki.value(), self.g_kd.value()
            r_from, r_to = self.from_spin.value(), self.sp_spin.value()
        self.pending_meta = None
        run = Run(len(self.runs) + 1, kp, ki, kd, r_from, r_to)
        self.runs.append(run)
        self.current_run = run

        shown = [r for r in self.runs if r.visible and r is not run]
        while len(shown) >= MAX_VISIBLE_RUNS:   # keep at most 8 runs on the graph
            self._set_run_visible(shown.pop(0), False)

        run.curve = pg.PlotDataItem(name=run.label())
        self.rs.angle.addItem(run.curve)
        self._add_table_row(run)
        self.select_run(run)
        self.run_btn.setEnabled(False)
        self.run_btn.setText("● recording…")

    # ---- MainWindow.finish_run(self) ---------------------------------------------------------
    #      The run is complete (RUN_END): work out its step numbers and add it to the table.
    def finish_run(self):
        run = self.current_run
        if run is None:
            return
        self.current_run = None
        run.data = run.array()
        run.rows = []
        run.metrics = step_metrics(run)
        self._fill_table_row(run)
        self.redraw_runs()
        self.rs.angle.enableAutoRange()
        self.rs.terms.enableAutoRange()
        self.run_btn.setText("▶  Start step run   (Ctrl+R)")
        self.run_btn.setEnabled(self.backend is not None)
        m = run.metrics
        self.statusBar().showMessage(
            f"Run {run.num}:  rise {m['rise']}   overshoot {m['overshoot']}   "
            f"settles {m['settle']}   offset {m['sserr']}   motor maxed {m['sat']}")
        # the demo keeps holding the new setpoint after the run
        self.sp_spin.blockSignals(True)
        self.sp_spin.setValue(run.r_to)
        self.sp_spin.blockSignals(False)

    # ---- MainWindow._swatch(self, color) -----------------------------------------------------
    #      Coloured square for a run in the table.
    def _swatch(self, color):
        pm = QtGui.QPixmap(12, 12)
        pm.fill(QtCore.Qt.transparent)
        p = QtGui.QPainter(pm)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setBrush(QtGui.QColor(color))
        p.setPen(QtCore.Qt.NoPen)
        p.drawRoundedRect(0, 0, 12, 12, 3, 3)
        p.end()
        return QtGui.QIcon(pm)

    # ---- MainWindow._add_table_row(self, run) ------------------------------------------------
    #      Adds a run to the results table.
    def _add_table_row(self, run):
        self.table.blockSignals(True)
        row = self.table.rowCount()
        self.table.insertRow(row)
        show = QtWidgets.QTableWidgetItem()
        show.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
        show.setCheckState(QtCore.Qt.Checked)
        self.table.setItem(row, 0, show)
        num = QtWidgets.QTableWidgetItem(self._swatch(run.color), f"{run.num}")
        self.table.setItem(row, 1, num)
        for c, v in ((2, run.kp), (3, run.ki), (4, run.kd)):
            self.table.setItem(row, c, QtWidgets.QTableWidgetItem(f"{v:g}"))
        self.table.setItem(row, 5, QtWidgets.QTableWidgetItem("recording…"))
        for c in range(6, 10):
            self.table.setItem(row, c, QtWidgets.QTableWidgetItem(""))
        self.table.blockSignals(False)

    # ---- MainWindow._fill_table_row(self, run) -----------------------------------------------
    #      Fills a run's row (gains, rise, overshoot, settle, error, saturation).
    def _fill_table_row(self, run):
        row = run.num - 1
        m = run.metrics
        for c, key in ((5, "rise"), (6, "overshoot"), (7, "settle"), (8, "sserr"), (9, "sat")):
            self.table.item(row, c).setText(m[key])

    # ---- MainWindow._set_run_visible(self, run, on) ------------------------------------------
    #      Shows or hides one run on the graphs.
    def _set_run_visible(self, run, on):
        if run.visible == on:
            return
        run.visible = on
        if on:
            self.rs.angle.addItem(run.curve)
        else:
            self.rs.angle.removeItem(run.curve)
        item = self.table.item(run.num - 1, 0)
        if item is not None:
            self.table.blockSignals(True)
            item.setCheckState(QtCore.Qt.Checked if on else QtCore.Qt.Unchecked)
            self.table.blockSignals(False)
        self.redraw_runs()

    # ---- MainWindow.on_run_item_changed(self, item) ------------------------------------------
    #      A run's checkbox changed: show or hide it.
    def on_run_item_changed(self, item):
        if item.column() == 0:
            run = self.runs[item.row()]
            self._set_run_visible(run, item.checkState() == QtCore.Qt.Checked)

    # ---- MainWindow.select_run(self, run) ----------------------------------------------------
    #      Makes a run the selected one (thicker line).
    def select_run(self, run):
        self.selected_run = run
        self.table.blockSignals(True)
        self.table.selectRow(run.num - 1)
        self.table.blockSignals(False)
        self.redraw_runs()

    # ---- MainWindow.on_run_selected(self) ----------------------------------------------------
    #      A run was clicked in the table.
    def on_run_selected(self):
        rows = self.table.selectionModel().selectedRows()
        if rows:
            self.selected_run = self.runs[rows[0].row()]
            if not self.selected_run.visible:
                self._set_run_visible(self.selected_run, True)
            self.redraw_runs()

    # ---- MainWindow.use_run_gains(self) ------------------------------------------------------
    #      Copies the selected run's gains back into the gain boxes.
    def use_run_gains(self):
        r = self.selected_run
        if r is None:
            return
        self.g_kp.set_value(r.kp)
        self.g_ki.set_value(r.ki)
        self.g_kd.set_value(r.kd)
        self.statusBar().showMessage(f"Loaded gains from run {r.num}", 4000)

    # ---- MainWindow.clear_runs(self) ---------------------------------------------------------
    #      Deletes every recorded run.
    def clear_runs(self):
        for r in self.runs:
            if r.visible and r.curve is not None:
                self.rs.angle.removeItem(r.curve)
        self.runs = []
        self.current_run = None
        self.selected_run = None
        self.table.setRowCount(0)
        self.run_btn.setText("▶  Start step run   (Ctrl+R)")
        self.run_btn.setEnabled(self.backend is not None)
        self.redraw_runs()

    # ---- MainWindow.export_runs(self) --------------------------------------------------------
    #      Saves every run to a CSV file.
    def export_runs(self):
        if not self.runs:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export runs", "pid_runs.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run", "Kp", "Ki", "Kd", "step_from_deg", "step_to_deg",
                        "t_s", "angle_deg", "setpoint_deg", "motor_us", "P_us", "I_us", "D_us"])
            for r in self.runs:
                for row in r.array():
                    w.writerow([r.num, r.kp, r.ki, r.kd, r.r_from, r.r_to] +
                               [f"{x:.4f}" for x in row])
        self.statusBar().showMessage(f"Saved {path}", 5000)

    # ------------------------------------------------------------- drawing --
    # ---- MainWindow.tick(self) ---------------------------------------------------------------
    #      Timer tick: collects events from the link and redraws.
    def tick(self):
        if self.backend is None:
            return
        try:
            events = self.backend.poll()
        except Exception as ex:
            self.log(f"Connection lost: {ex}")
            self.disconnect_backend()
            return

        live_rows = []
        for kind, data in events:
            if kind == "sample":
                live_rows.append(data)
            elif kind == "run_sample":
                if self.current_run is None:
                    self.begin_run()
                self.current_run.rows.append(data)
            elif kind == "run_begin":
                if self.current_run is not None:
                    self.finish_run()
                self.begin_run()
            elif kind == "run_end":
                self.finish_run()
            elif kind == "ack":
                self.on_ack(data)
            elif kind == "armed":
                self.log(data)
                self.statusBar().showMessage("ARMED - ESC power on. Wait 2 s, then start.", 7000)
            elif kind == "disarmed":
                self.log(data)
                reason = data.split(",", 1)[1] if "," in data else ""
                self.statusBar().showMessage(f"DISARMED - {reason}. Motors off, relay open; ARM again to continue.", 10000)
            elif kind == "log":
                self.log(data)

        if live_rows:
            last = self.live.last_t()
            if last is not None and live_rows[0][0] < last - 1.0:
                self.live.clear()               # PIC was reset: its clock started over
            self.live.extend(live_rows)
            self.rate_count += len(live_rows)
            self._update_readings(live_rows[-1])

        now = time.perf_counter()
        if now - self.rate_t0 >= 1.0:
            hz = self.rate_count / (now - self.rate_t0)
            self.rate_label.setText(f"{hz:.0f} samples/s")
            self.rate_count = 0
            self.rate_t0 = now

        if self.gains_dirty and self.live_update_cb.isChecked():
            self.apply_gains()                  # at most once per redraw = rate limited

        if not self.pause_btn.isChecked():
            self.redraw_live()
        if self.current_run is not None:
            self._update_current_run()

    # ---- MainWindow._update_readings(self, row) ----------------------------------------------
    #      Updates the live number readouts.
    def _update_readings(self, row):
        t, a, sp, motor, P, I, D = row
        self.read_labels["angle"].setText(f"{a:+7.2f} °")
        self.read_labels["error"].setText(f"{sp - a:+7.2f} °")
        self.read_labels["motor"].setText(f"{motor:6.0f} µs  ({motor / US_PER_COUNT:.0f} cnt)")
        if np.isnan(P):
            self.read_labels["pid"].setText("(not sent by PIC)")
        else:
            self.read_labels["pid"].setText(f"{P:+.0f} · {I:+.0f} · {D:+.0f} µs")
        if isinstance(self.backend, (SerialBackend, BleBackend)) and self.backend.motor_b is not None:
            self.motor_now.setText(f"Now:  front {motor:4.0f} µs   ·   back {self.backend.motor_b:4.0f} µs")

    # ---- MainWindow.redraw_live(self) --------------------------------------------------------
    #      Redraws the live graphs.
    def redraw_live(self):
        d = self.live.last_seconds(self.window_spin.value())
        self.live_view = d
        if len(d) == 0:
            for c in (self.l_angle, self.l_sp, self.ls.c_motor, self.ls.c_P, self.ls.c_I, self.ls.c_D):
                c.setData([], [])
            return
        t = d[:, 0]
        self.l_angle.setData(t, d[:, 1])
        self.l_sp.setData(t, d[:, 2])
        self.ls.c_motor.setData(t, d[:, 3])
        if self.terms_cb.isChecked():
            self.ls.c_P.setData(t, d[:, 4], connect="finite")
            self.ls.c_I.setData(t, d[:, 5], connect="finite")
            self.ls.c_D.setData(t, d[:, 6], connect="finite")
        self.ls.angle.setXRange(t[-1] - self.window_spin.value(), t[-1], padding=0)

    # ---- MainWindow.redraw_runs(self) --------------------------------------------------------
    #      Redraws the recorded-run graphs.
    def redraw_runs(self):
        sel = self.selected_run
        for r in self.runs:
            if not r.visible or r.curve is None:
                continue
            d = r.array()
            c = QtGui.QColor(r.color)
            if r is sel:
                pen = pg.mkPen(c, width=3)
                r.curve.setZValue(10)
            else:
                c.setAlphaF(0.5)
                pen = pg.mkPen(c, width=1.5)
                r.curve.setZValue(1)
            r.curve.setPen(pen)
            r.curve.setSymbol("o" if self.dots_cb.isChecked() else None)
            r.curve.setSymbolSize(4)
            r.curve.setSymbolPen(None)
            r.curve.setSymbolBrush(c)
            if len(d):
                r.curve.setData(d[:, 0], d[:, 1])
            else:
                r.curve.setData([], [])

        self._draw_selected_run()

    # ---- MainWindow._update_current_run(self) ------------------------------------------------
    #      While recording, updates only the run that is still growing.
    def _update_current_run(self):
        # while recording, only the run that is growing needs new data
        run = self.current_run
        d = run.array()
        if run.visible and run.curve is not None and len(d):
            run.curve.setData(d[:, 0], d[:, 1])
        if run is self.selected_run:
            self._draw_selected_run()

    # ---- MainWindow._draw_selected_run(self) -------------------------------------------------
    #      Draws the selected run on top.
    def _draw_selected_run(self):
        sel = self.selected_run
        if sel is None or len(sel.array()) == 0:
            for c in (self.r_sp, self.rs.c_motor, self.rs.c_P, self.rs.c_I, self.rs.c_D):
                c.setData([], [])
            return
        d = sel.array()
        t = d[:, 0]
        t0, t1 = t[0], max(t[-1], 0.0)
        self.r_sp.setData([t0, 0.0, 0.0, t1], [sel.r_from, sel.r_from, sel.r_to, sel.r_to])
        self.rs.c_motor.setData(t, d[:, 3])
        self.rs.c_P.setData(t, d[:, 4], connect="finite")
        self.rs.c_I.setData(t, d[:, 5], connect="finite")
        self.rs.c_D.setData(t, d[:, 6], connect="finite")

    # ---- MainWindow.on_pause(self, on) -------------------------------------------------------
    #      Pauses/resumes the live graph.
    def on_pause(self, on):
        self.pause_btn.setText("Resume" if on else "Pause graph")
        if not on:
            for line in self.ls.vlines:
                line.setVisible(False)
            self.ls.dot.setData([], [])

    # ---- MainWindow.on_dots(self, on) --------------------------------------------------------
    #      Shows or hides a dot on every 4 ms sample.
    def on_dots(self, on):
        sym = "o" if on else None
        self.l_angle.setSymbol(sym)
        self.l_angle.setSymbolSize(4)
        self.l_angle.setSymbolPen(None)
        self.l_angle.setSymbolBrush(C_ANGLE)
        self.redraw_runs()

    # ---- MainWindow.on_terms(self, on) -------------------------------------------------------
    #      Shows or hides the P/I/D terms.
    def on_terms(self, on):
        self.ls.show_terms(on)
        self.rs.show_terms(on)
        if not on:
            for c in (self.ls.c_P, self.ls.c_I, self.ls.c_D):
                c.setData([], [])

    # --------------------------------------------------------------- hover --
    # ---- MainWindow._nearest(t_arr, x) -------------------------------------------------------
    #      Index of the sample whose time is closest to x.
    @staticmethod
    def _nearest(t_arr, x):
        i = int(np.searchsorted(t_arr, x))
        if i <= 0:
            return 0
        if i >= len(t_arr):
            return len(t_arr) - 1
        return i if abs(t_arr[i] - x) < abs(t_arr[i - 1] - x) else i - 1

    # ---- MainWindow._sw(color) ---------------------------------------------------------------
    #      Small coloured square used in the readout text.
    @staticmethod
    def _sw(color):
        return f"<span style='color:{color}'>■</span>"

    # ---- MainWindow._row_text(self, row) -----------------------------------------------------
    #      Formats one sample for the hover readout.
    def _row_text(self, row):
        t, a, sp, motor, P, I, D = row
        s = (f"{self._sw(C_ANGLE)} angle <b style='color:{TEXT}'>{a:+.2f}°</b> &nbsp; "
             f"setpoint {sp:+.1f}° &nbsp; error {sp - a:+.2f}° &nbsp; "
             f"{self._sw(C_MOTOR)} motor <b style='color:{TEXT}'>{motor:.0f} µs</b> "
             f"({motor / US_PER_COUNT:.0f} cnt)")
        if not np.isnan(P):
            s += (f" &nbsp; {self._sw(C_P)} P {P:+.1f} &nbsp; {self._sw(C_I)} I {I:+.1f}"
                  f" &nbsp; {self._sw(C_D)} D {D:+.1f}")
        return s

    # ---- MainWindow.on_hover_live(self, pos) -------------------------------------------------
    #      Mouse over the live graph: show the exact sample under it.
    def on_hover_live(self, pos):
        x = self.ls.x_at(pos)
        d = self.live_view
        if x is None or len(d) == 0:
            return
        i = self._nearest(d[:, 0], x)
        row = d[i]
        self.ls.cursor(row[0], row[1])
        self.live_readout.setText(f"t = {row[0]:.3f} s &nbsp;│&nbsp; " + self._row_text(row))

    # ---- MainWindow.on_hover_runs(self, pos) -------------------------------------------------
    #      Mouse over the runs graph: show the exact sample under it.
    def on_hover_runs(self, pos):
        x = self.rs.x_at(pos)
        sel = self.selected_run
        if x is None or sel is None or len(sel.array()) == 0:
            return
        d = sel.array()
        i = self._nearest(d[:, 0], x)
        row = d[i]
        self.rs.cursor(row[0], row[1])
        text = f"t = {row[0]:+.3f} s &nbsp;│&nbsp; <b>Run {sel.num}</b> &nbsp; " + self._row_text(row)
        others = []
        for r in self.runs:
            if r is sel or not r.visible:
                continue
            od = r.array()
            if len(od):
                j = self._nearest(od[:, 0], row[0])
                others.append(f"{self._sw(r.color)} run {r.num} {od[j, 1]:+.2f}°")
        if others:
            text += " &nbsp;│&nbsp; " + " &nbsp; ".join(others)
        self.runs_readout.setText(text)

    # ------------------------------------------------------------- console --
    # ---- MainWindow.log(self, text) ----------------------------------------------------------
    #      Adds a line to the console.
    def log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.console.appendPlainText(f"[{stamp}] {text}")

    # ---- MainWindow.send_raw(self) -----------------------------------------------------------
    #      Sends whatever is typed in the console box.
    def send_raw(self):
        text = self.raw_edit.text().strip()
        if not text:
            return
        if isinstance(self.backend, (SerialBackend, BleBackend)):
            try:
                self.backend.send(text)
            except Exception as ex:
                self.log(f"send failed: {ex}")
        else:
            self.log("(demo mode ignores raw commands - switch Mode to Serial)")
        self.raw_edit.clear()


# =============================================================================
# ---- apply_theme(app) ------------------------------------------------------------------------
#      Sets the dark Fusion colour theme for the whole app.
#  Dark theme
# =============================================================================
def apply_theme(app):
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    C = QtGui.QColor
    pal.setColor(QtGui.QPalette.Window, C(PANEL))
    pal.setColor(QtGui.QPalette.WindowText, C(TEXT))
    pal.setColor(QtGui.QPalette.Base, C(SURFACE))
    pal.setColor(QtGui.QPalette.AlternateBase, C(PANEL2))
    pal.setColor(QtGui.QPalette.Text, C(TEXT))
    pal.setColor(QtGui.QPalette.Button, C(PANEL2))
    pal.setColor(QtGui.QPalette.ButtonText, C(TEXT))
    pal.setColor(QtGui.QPalette.ToolTipBase, C(PANEL2))
    pal.setColor(QtGui.QPalette.ToolTipText, C(TEXT))
    pal.setColor(QtGui.QPalette.Highlight, C(SERIES[0]))
    pal.setColor(QtGui.QPalette.HighlightedText, C("#ffffff"))
    pal.setColor(QtGui.QPalette.PlaceholderText, C(MUTED))
    for role in (QtGui.QPalette.Text, QtGui.QPalette.ButtonText, QtGui.QPalette.WindowText):
        pal.setColor(QtGui.QPalette.Disabled, role, C(MUTED))
    app.setPalette(pal)
    app.setStyleSheet(f"""
        QGroupBox {{ border:1px solid {BORDER}; border-radius:6px; margin-top:16px;
                     padding:10px 8px 8px 8px; font-weight:600; }}
        QGroupBox::title {{ subcontrol-origin: margin; left:10px; padding:0 4px; color:{TEXT2}; }}
        QFrame#gain {{ background:{SURFACE}; border:1px solid {BORDER}; border-radius:6px; }}
        QFrame#gain QLabel {{ background:transparent; }}
        QPushButton {{ padding:5px 10px; border:1px solid {BORDER}; border-radius:5px;
                       background:{PANEL2}; }}
        QPushButton:hover {{ border-color:{MUTED}; }}
        QPushButton:checked {{ background:{SERIES[0]}; color:white; }}
        QPushButton:disabled {{ color:{MUTED}; }}
        QPushButton#runbtn {{ background:#2a78d6; color:white; font-weight:600; border:none; }}
        QPushButton#runbtn:disabled {{ background:#1c5cab; color:#cde2fb; }}
        QPushButton#killbtn {{ background:{CRITICAL}; color:white; font-weight:700; border:none; }}
        QTabBar::tab {{ padding:6px 16px; }}
        QTableWidget {{ gridline-color:{BORDER}; }}
        QHeaderView::section {{ background:{PANEL2}; color:{TEXT2}; border:none; padding:4px 6px; }}
        QSlider::groove:horizontal {{ height:4px; background:{BORDER}; border-radius:2px; }}
        QSlider::sub-page:horizontal {{ background:{SERIES[0]}; border-radius:2px; }}
        QSlider::handle:horizontal {{ background:{TEXT}; width:14px; margin:-6px 0; border-radius:7px; }}
    """)


# ---- main() ----------------------------------------------------------------------------------
#      Program entry: starts the Qt app and opens the main window.
def main():
    # antialias off = much faster live redraw (smooth lines cost ~10x more to draw)
    pg.setConfigOptions(antialias=False, background=SURFACE, foreground=TEXT2)
    app = QtWidgets.QApplication(sys.argv)
    apply_theme(app)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
