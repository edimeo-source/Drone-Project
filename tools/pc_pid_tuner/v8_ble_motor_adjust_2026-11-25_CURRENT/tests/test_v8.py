# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : tools/pc_pid_tuner/v8_ble_motor_adjust_2026-11-25_CURRENT/tests/test_v8.py
#  What    : PC-side test for PC PID tuner v8 (USB or Bluetooth, v3.1: two axes + motor adjust buttons) (no board needed)
#  Date    : 2026-11-25
#  FUNCTIONS: check(), fake_init(), serial_feed()
# ==============================================================================================

"""Offscreen test of PC tuner v8 against the firmware v3.1 protocol (no board, no Bluetooth).

python test_v8.py [fw_out folder]
fw_out = real firmware output written by Drone v3.1's host_tests/run_tests.sh (run that first).
Needs: PySide6, pyqtgraph, numpy (the tuner's own packages)."""
import os, sys, queue, time
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pid_tuner_ble as T
from PySide6 import QtWidgets
fails = 0
def check(c, msg):
    global fails
    print(("PASS  " if c else "FAIL  ") + msg)
    fails += 0 if c else 1

# ---- 1. serial parser
sb = T.SerialBackend.__new__(T.SerialBackend)
sb.events, sb.lines, sb.in_run, sb.armed, sb.last_heartbeat, sb.motor_b = [], queue.Queue(), False, False, 0, None
for ln in ["AR,+2.0000,+0.1000,+0.3000", "AP,+1.5000,+0.0000,+0.2000", "A,+1.0000,+0.0000,+0.0000",
           "AXIS,1,ROLL", "D,1000,+2.50,+0.00,1540,+25.0,+0.0,-3.0,1460,+2.50,-1.00",
           "D,1004,-1.00,+0.00,1530,+10.0,+0.0,-1.0,1470,+2.00,-1.00,+0.0,+0.0,1570,1490,1510,1430,+40.0,+30.0"]:
    sb.lines.put(ln)
ev = sb.poll()
kinds = [k for k, _ in ev]
check(("ack", (2.0, 0.1, 0.3, 1)) in ev and ("ack", (1.5, 0.0, 0.2, 2)) in ev and ("ack", (1.0, 0.0, 0.0, 0)) in ev,
      "AR / AP / A acks carry their axis")
check(("axis", 1) in ev, "AXIS line -> axis event")
check(kinds.count("sample") == 2 and sb.motor_b == 1470 and sb.roll_now == 2.0 and sb.pitch_now == -1.0,
      "D lines (11 and 19 fields) parse; roll / pitch kept")
check(sb.esc == [1570, 1490, 1510, 1430], "BOTH D line: all 4 ESC pulses kept")

# ---- 2. Bluetooth parser: BOTH-mode run (quads) and ROLL run (pairs)
bb = T.BleBackend.__new__(T.BleBackend)
bb.in_run, bb.armed, bb.motor_b, bb.t0 = False, False, None, time.monotonic()
out = []
bb._parse_line("RUN_BEGIN,6,2,+0.0,+4.0,1600,1600,+1.0000,+0.0000,+0.2500,3,+1.2000,+0.0000,+0.3000", out)
bb._parse_line("RD,0,10,-20,5,30,11,-21,6,31,12,-22,7,32,13,-23,8,33", out)
rs = [d for k, d in out if k == "run_sample"]
check(len(rs) == 4 and abs(rs[0][1] - (-2.0)) < 1e-6 and abs(rs[0][3] - 1630) < 1e-6 and abs(bb.motor_b - (1600 - 33)) < 1e-6,
      "BOTH run over Bluetooth: 4 samples per line, graph = pitch, front = T + PC, back = T - PC")
out = []
bb._parse_line("RUN_BEGIN,4,2,+0.0,+5.0,1600,1600,+1.0000,+0.0000,+0.2500,1,+1.0000,+0.0000,+0.2500", out)
bb._parse_line("RD,0,10,40,20,35,30,30,40,25", out)
rs = [d for k, d in out if k == "run_sample"]
check(len(rs) == 4 and abs(rs[1][1] - 2.0) < 1e-6 and abs(rs[1][3] - 1635) < 1e-6,
      "ROLL run over Bluetooth: pairs, graph = roll, + side = T + RC")
out = []
bb._parse_line("RUN_BEGIN,4,2,+0.0,+5.0,1500,1500,+1.0000,+0.0000,+0.2500", out)     # v2.0 firmware
bb._parse_line("RD,0,10,40,20,35", out)
check(len([1 for k, _ in out if k == "run_sample"]) == 2, "old v2.0 RUN_BEGIN (no axis) still parses as pitch")

# ---- 3. the window: axis selector, per-axis gains, commands sent on connect
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
sent = []
def fake_init(self, port, baud):
    self.events, self.lines, self.in_run, self.armed, self.last_heartbeat, self.motor_b = [], queue.Queue(), False, False, 0, None
    self.port_name = port
T.SerialBackend.__init__ = fake_init
T.SerialBackend.send = lambda self, text: sent.append(text)
T.SerialBackend.close = lambda self: None
T.SerialBackend.describe = lambda self: "fake serial"
w = T.MainWindow()
w.mode_combo.setCurrentIndex(1)
w.port_combo.setEditText("COM9")
# roll gains 2/0.5/0.3, pitch gains 1.5/0/0.2 typed before connecting
w.gax_roll.setChecked(True)
w.g_kp.set_value(2.0); w.g_ki.set_value(0.5); w.g_kd.set_value(0.3)
w.gax_pitch.setChecked(True)
check(abs(w.g_kp.value() - 1.0) < 1e-9, "switching the shown gains to Pitch loads the pitch gains (still 1.0)")
w.g_kp.set_value(1.5); w.g_ki.set_value(0.0); w.g_kd.set_value(0.2)
w.gax_roll.setChecked(True)
check(abs(w.g_kp.value() - 2.0) < 1e-9 and abs(w.g_ki.value() - 0.5) < 1e-9, "back to Roll: the roll gains come back (2.0, 0.5)")
w.axis_combo.setCurrentIndex(0)
sent.clear()
w.connect_backend()
ia = sent.index("A,1") if "A,1" in sent else 99
check(ia < min(i for i, x in enumerate(sent) if x.startswith(("KR", "KP", "B,", "U,"))), "connect: axis (A,1 = roll only) sent before gains / throttle")
check("KR,2.0000,0.5000,0.3000" in sent and "KP,1.5000,0.0000,0.2000" in sent, "connect: both gain sets sent (KR and KP)")
check(any(x.startswith("B,") and x.count(",") == 1 for x in sent) and any(x.startswith("U,") for x in sent), "connect: B,T (one throttle) and U,rate")
sent.clear()
w.g_kp.set_value(2.5); w.apply_gains()
check(sent and sent[-1].startswith("KR,2.5000"), "Apply sends the shown axis's gains (KR)")
w.on_ack((2.5, 0.5, 0.3, 1))
check(w.g_kp._applied_value == 2.5, "AR ack shown on the roll sliders")
w.on_ack((9.0, 0.0, 0.0, 2))
check(w.g_kp._applied_value == 2.5 and w.axis_applied[2] == (9.0, 0.0, 0.0), "AP ack stored for pitch, roll sliders untouched")
sent.clear(); w.axis_combo.setCurrentIndex(2)
check(sent == ["A,3"], "axis Both -> A,3")
w.hold_roll.setValue(5); w.hold_pitch.setValue(10)
sent.clear()
for b in w.findChildren(QtWidgets.QPushButton):
    if b.text() == "Hold":
        b.click()
check(sent == ["SR,5.0", "SP,10.0"], "Hold sends SR,5.0 and SP,10.0 (professor's demo)")
w.axis_combo.setCurrentIndex(0)
check(w.ls.angle.getAxis("left").labelText.startswith("Roll") and w.read_names["motor"].text() == "Left pair",
      "Roll mode: graphs and readouts say Roll / Left pair")
w.axis_combo.setCurrentIndex(1)
check(w.ls.angle.getAxis("left").labelText.startswith("Pitch") and w.read_names["motor"].text() == "Front pair",
      "Pitch mode: graphs and readouts say Pitch / Front pair")
sent.clear(); w.send_base()
check(sent == [f"B,{w.base_a.value():.0f}"], "Send T -> B,<T>")
# ---- v8: motor adjust buttons
sent.clear()
w.adj_step.setCurrentText("10")
w.adjust_yaw(+1)
check(sent == ["O,10,-10,-10,10"], "turn CW (step 10) sends O,10,-10,-10,10 (the command the firmware test flies with)")
w.adj_step.setCurrentText("25"); sent.clear()
w.adjust_motor(3, -1)
check(sent == ["O,10,-10,-35,10"], "RL - (step 25) sends the whole set: O,10,-10,-35,10")
for _ in range(20):
    w.adjust_motor(1, +1)
check(w.adj[1] == 200 and sent[-1] == "O,200,-10,-35,10", "FL + stops at the firmware's 200 us")
sb.lines.put("OFFS,15,-5,0,7")
for k, d in sb.poll():
    if k == "offs":
        w.on_offsets(d)
check(w.adj == [0, 15, -5, 0, 7] and "FL +15" in w.adj_now.text() and "+15" in w.adj_labels[1].text(),
      "OFFS reply from the PIC: the box shows what the drone uses")
w.zero_adjust()
check(sent[-1] == "O,0,0,0,0", "Zero all sends O,0,0,0,0")
w.adjust_yaw(-1); sent.clear()
w.disconnect_backend(); w.connect_backend()
check("O,-25,25,25,-25" in sent, "connect sends the saved motor adjust (the drone starts at 0)")
w.disconnect_backend()

# ---- 4. REAL firmware output (written by ../host_tests/run_tests.sh into fw_out/)
import math
_t = os.path.dirname(os.path.abspath(__file__))
here = next((c for c in [sys.argv[1] if len(sys.argv) > 1 else "", os.environ.get("FW_OUT", ""),
             os.path.join(_t, "../../../../firmware/03_drone/v3.1_motor_adjust_buttons/host_tests/fw_out"),   # in the repo
             os.path.join(_t, "../host_tests/fw_out")]                                                       # working copy
             if c and os.path.exists(os.path.join(c, "pc_roll.txt"))), "")
def serial_feed(path):
    b = T.SerialBackend.__new__(T.SerialBackend)
    b.events, b.lines, b.in_run, b.armed, b.last_heartbeat, b.motor_b = [], queue.Queue(), False, False, 0, None
    for ln in open(path).read().splitlines():
        b.lines.put(ln)
    return b, b.poll()
if here:
    b, ev = serial_feed(os.path.join(here, "pc_roll.txt"))
    rs = [d for k, d in ev if k == "run_sample"]
    nlines = len(open(os.path.join(here, "pc_roll.txt")).read().splitlines())
    check(len(rs) == nlines - 2 and not [1 for k, _ in ev if k == "log"] and rs[60][2] == 5.0 and rs[10][2] == 0.0,
          f"firmware ROLL step dump ({len(rs)} samples) parses cleanly; setpoint 0 -> 5")
    b, ev = serial_feed(os.path.join(here, "pc_both.txt"))
    rs = [d for k, d in ev if k == "run_sample"]
    check(rs and not [1 for k, _ in ev if k == "log"] and len(b.esc) == 4 and rs[60][2] == 4.0,
          f"firmware BOTH step dump ({len(rs)} samples) parses cleanly; 4 ESC pulses read")
    bb = T.BleBackend.__new__(T.BleBackend)
    bb.in_run, bb.armed, bb.motor_b, bb.t0 = False, False, None, time.monotonic()
    out = []
    for ln in open(os.path.join(here, "phone_both.txt")).read().splitlines():
        bb._parse_line(ln, out)
    rs = [d for k, d in out if k == "run_sample"]
    ok = len(rs) == 58 and all(abs(rs[k][1] - round(-21.0 * math.cos(0.2 * k)) / 10.0) < 1e-6 for k in range(58))
    check(ok, "firmware BOTH Bluetooth run (58 samples, quads) -> tuner: every pitch sample exact")
else:
    print("(skip) no firmware output found - run Drone v3.1 host_tests/run_tests.sh first, or pass its fw_out folder")
print(f"\n{fails} failure(s)" if fails else "\nall passed")
sys.exit(1 if fails else 0)
