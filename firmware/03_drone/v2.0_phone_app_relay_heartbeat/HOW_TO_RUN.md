# ELC275 Drone v2.0: how to run it and how it works
Ethan DiMeo, Nov 23 2026

## What is in this folder

| Folder / file | What it is |
|---|---|
| `Drone.X/` | MPLAB X project for the drone (XC32). Firmware 2.0. Uses `config.bits` (120 MHz). |
| `DroneTuner.apk` | The phone app, ready to install (Android 7.0 and newer). |
| `android_app/` | Source of the phone app (Drone Tuner). One Java file, no libraries. Android Studio can rebuild it. |
| `pid_tuner_ble.py`, `run_tuner_ble.bat` | PC tuner. **Serial** mode = USB cable. **BLE** mode = a laptop with Bluetooth (now reads the 2.0 data lines). |
| `relay_test/` | Your relay test program, unchanged. It is a separate program: don't add it to Drone.X. |
| `host_tests/` | PC tests of the firmware with a simulated Bluefruit (`sh run_tests.sh`). |

## 1. Wiring check

- **Splitters on SCK / MOSI / MISO: OK.** SPI is a bus. The PIC drives SCK and MOSI into two inputs. Only the selected chip drives MISO: the IMU's SDO and the Bluefruit's MISO both float while their CS is high. The firmware never pulls both CS lines low together.
- **The joints must be crimped, not twisted and taped.** That follows the teacher's no-solder rule, and a loose MISO joint is exactly what made WHO_AM_I read `0x7F` before.
- **IMU 3.3 V on J12 pin 1, Bluefruit VIN on pin 2 (5 V): OK.** The Bluefruit has its own 3.3 V regulator, and its logic is 3.3 V, the same as the PIC.
- **Separate GND pins: OK.** Every GND pin on the kit is the same ground.
- **IMU ADDR jumper must stay cut.** Closed, it ties SDO (our shared MISO) to a fixed level and fights the Bluefruit.
- **Relay: CH1 from RK4 (J12 pin 33).** RK4 HIGH = motor power ON, RK4 LOW = power CUT. The PIC also holds RK4 LOW while it is in reset.
  **Meter check before any props:** hold the kit's reset button and measure the ESC power pads. You must read 0 V. If you read battery voltage, the motor wire is on the wrong relay contact (NC), and a crash or reset would leave the motors powered. Move it to NO.

## 2. What your relay test program does (`main_bluefruit_relay_test-2.c`)

It is a stand-alone test with its own config pragmas. That is why it is not merged into Drone.X: the drone firmware already uses the same relay rule.

1. **Clock:** internal FRC 8 MHz, so the core timer runs at 4 MHz (`MS_TICKS = ms * 4000`). UART2 is 9600 baud ($\frac{4\,\text{MHz}}{16(25+1)} = 9615$).
2. **`early_gpio`:**
   - sets the relay latch LOW *before* making RK4 an output, so the relay can't click on at power-up;
   - sets both CS lines high and the ESC pins low;
   - switches off the OC and timer modules;
   - turns on the pull-up for SW1.
3. **SPI:** bit-banged, mode 0, about 8 µs per bit (~125 kHz).
4. **SDEP (Adafruit's SPI packets):**
   - Each packet is up to 4 header bytes + 16 data bytes.
   - `0xFE` means "busy", so it retries.
   - It drains old packets at start, then sends `ATE=0` and `AT`.
5. **Main loop:**
   - Every 250 ms: `AT+GAPGETCONN` (is a phone connected?).
   - About every 20 ms: `AT+BLEUARTRX` (read phone text).
   - Words it understands: `START`, `STOP`, `STATUS`.
     - `STOP` acts at once.
     - A word that isn't finished within 200 ms is thrown away.
     - Replies go back with `AT+BLEUARTTX`.
6. **START** works only when all of these are true: a phone is connected, the link was checked in the last 1 s, SW1 is up, and old phone text has been drained. Then RK4 = 1 (motor power ON) and the green LED comes on.
7. **`relay_service`** sets RK4 = 0 (power CUT) on any of: disconnect, a link check older than 1 s, or SW1. An SPI failure also cuts power and restarts the module.

So in that program, **1 = motors powered, 0 = power cut.** Firmware 2.0 does the same on the same pin.

## 3. Why only pitch (two PWM groups)

The professor's mixer drives two groups. The front pair (ESC 1 + 2) gets $f = b_f + u$ and the back pair (ESC 3 + 4) gets $b = b_b - u$.

- **Pitch works:** making the front stronger than the back tips the nose up.
- **Roll can't work:** roll needs the left side stronger than the right. Inside each group the left and right motors always get the *same* pulse, so the left–right difference is 0 µs and no command can change it.
- **What roll would need:** four independent outputs, $m_{FL} = b + u_p + u_r$, $m_{FR} = b + u_p - u_r$, and so on.
- **Roll is still measured.** It is used for the 75° tip cutoff.

## 4. Bluetooth speed: "4 MB/s" is not right

- **4 MHz** is the fastest **SPI clock** the Bluefruit allows between the PIC and the module (we use 1 MHz). It is a wire clock, not a radio rate. Even 4 MHz is only $4\,\text{Mbit/s} = 0.5\,\text{MB/s}$ on the wire.
- **The radio** is Bluetooth 4.x at 1 Mbit/s, and each packet carries only 20 bytes of your data.
- **Measured on this module:** 213 B/s (Nordic forum), and about 100 B/s by default or 300 B/s after `AT+GAPINTERVALS=20,40,25,30` (which firmware 2.0 sends). The phone app shows the real rate it measures while data comes in.

**Why live 4 ms data is impossible:** one sample is about 8 bytes, so
$$\frac{8\ \text{bytes}}{0.004\ \text{s}} = 2000\ \text{B/s} \gg 300\ \text{B/s}.$$
That is the reason for the professor's array method below.

**How long a step test takes to arrive:** $N = 50 + \frac{3000\ \text{ms}}{4\ \text{ms}} = 800$ samples, about $800 \times 8 = 6.4$ kB, so $\frac{6400}{300} \approx 21$ s to $\frac{6400}{100} = 64$ s.

## 5. How it works

### The 4 ms loop (Timer4 interrupt, `control.c`)
Each pass does these steps in order:
1. Read the IMU over SPI1 (mode 3, 2 MHz).
2. Complementary filter, $\theta = 0.98(\theta + \omega\,\Delta t) + 0.02\,\theta_{acc}$.
3. PID in the Lynch form, $e = r - \theta$, $\dot e = -\omega$, $u = K_p e + K_i \int e\,dt + K_d \dot e$. The integral is limited to ±20 and $u$ to ±500 µs.
4. Mixer: front $= b_f + u$, back $= b_b - u$. The floor is 1100 µs, and the soft start lets the top limit climb only 4 µs per loop.
5. Send the pulses to OC1–OC4 (50 Hz, 1–2 ms).
6. If a step test is running, store the sample in the arrays.

### IMU first on the shared SPI pins (`bluefruit.c`)
Timer4 counts $0 \to 3749$ in 4 ms ($1.067$ µs per count).
- **The window:** a Bluefruit packet may start only when `TMR4 <= 2800`, which is 1.0 ms or more before the next IMU read.
- **During a packet:**
  - the 4 ms interrupt is held off (`T4IE = 0`), so it can't land mid-packet;
  - SPI switches to the Bluefruit's mode 0 at 1 MHz;
  - after the packet, the IMU's mode 3 at 2 MHz comes back.
- **Why the IMU is never late:** a packet takes about 0.3 ms (the 100 µs CS wait plus 20 bytes at 1 MHz). That always ends before 3.3 ms, so the IMU read at 4 ms is never late.

The simulated-Bluefruit test checks this rule on every byte (826 packets).

### ARM, heartbeat, relay (`control.c`, `main.c`)
1. **ARM** checks: calibrated, SW1 up, within 75°, motors off. Then ESCs are set to 1000 µs, RK4 = 1 (power ON), and the 2 s ESC start-up begins.
2. **Heartbeat:** the app that armed sends `H` every 100 ms. If **500 ms** pass without an `H`, the motors go off and RK4 = 0. Only the link that armed counts, so a PC can't keep a phone-armed drone alive.
3. **Losing the phone:** if it disconnects, or the Bluefruit stops answering while the phone has it armed, the motors go off at once.
4. **Other kills:** KILL (`X`), SW1, tip past ±75°, or an IMU error all turn the motors off and open the relay.

### Professor's array method (`tuner.c`)
1. `R,from,to,ms`: PID on, hold `from` for 1 s, then step to `to`.
2. Every 4 ms sample goes into arrays inside the interrupt. **Nothing is sent while recording.**
3. At the end, the motors drop to 1000 µs. The drone stays armed and the relay stays closed.
4. `data_complete = 1`, then main sends the arrays:

```
RUN_BEGIN,800,50,0.0,10.0,1500,1500,1.0000,0.0000,0.0000
RD,0,a,u,a,u,...   (8 samples per line, a = pitch x10, u = PID output in us)
RUN_END
```

The phone rebuilds the motor pulses with the same mixer, clamped to 1100–2000 µs:
$$f = b_f + u, \qquad b = b_b - u.$$
The test checks that all 58 test samples arrive exactly (to 0.1° and 1 µs).

## 6. Step by step for today

**A. Flash and check (props OFF)**
1. Open `Drone.X` in MPLAB X 6.15 and set XC32 v6.00 → Build → Program.
2. Open PuTTY on the kit's COM port at **115200**. You should see these in order:
   - `WHO_AM_I = 0x6C`
   - `Bluefruit OK` with its firmware version
   - the calibration step: keep the drone still and level for 2 s
   - `Bluefruit ready`
3. If WHO_AM_I shows `0x7F` or `0xFF`, check the MISO splitter joint and IMU pin 7.
4. Do the relay meter check from section 1.

**B. Phone**
1. Install `DroneTuner.apk` (send it to the phone, then tap it).
   - Android asks to allow installs from your Files or browser app: allow it once.
   - If Play Protect warns about an unknown developer, tap **More details → Install anyway**. The app is signed with our own key, not a Play Store key.
2. Open it, tap **Connect**, and allow **Nearby devices**. On Android 11 and older, it asks for Location instead, and Location must be on.
3. You should see "Connected to the drone". Tilt the drone: pitch and roll update about 4 times a second.

**C. Safety tests (props OFF)**
1. **ARM:** the relay clicks and the ESCs beep. The screen says to wait 2 s.
2. **KILL:** the relay opens.
3. **Link loss:** ARM again, then turn the phone's Bluetooth off. The relay must open within about 0.5 s.
4. **SW1:** ARM, then press SW1 on the kit. The relay opens.

**D. Step test**
1. Set **base front / back** (start low, for example 1150–1300 µs).
2. Set **Kp** (P only first, as the professor said). Leave Ki and Kd at 0.
3. ARM, wait 2 s, then **Run step test** from 0 to 10, 3000 ms.
4. The motors run for 1 s + 3 s, then stop. Data arrives in about 20–60 s with a live byte-rate readout.
5. Read the graph and the numbers (rise, overshoot, settling, steady-state error). **Share CSV** sends the data for your report.
6. Change the gains and repeat. Raise Kp until it oscillates, back off, then add Kd, then a little Ki.

**Without the phone:** the PC tuner works over USB (Serial mode, 115200) with the same commands. It sends the heartbeat for you. In BLE mode on a laptop with Bluetooth (`py -m pip install bleak`), it also reads the 2.0 data lines.

## 7. Commands

| Command | Meaning |
|---|---|
| `ARM` | Close the relay; the app must send `H` every 100 ms after this |
| `H` | Heartbeat |
| `X` or `KILL` or `STOP` | Motors off, relay open |
| `K,kp,ki,kd` | Gains (reply `A,...`) |
| `B,front,back` | Base pulses (reply `BASE,...`; `B` alone reports them) |
| `R,from,to,ms` | Step test (phone: at most 5000 ms) |
| `G` | PID hold at the setpoint |
| `S,deg` | Setpoint |
| `M,front,back` | Manual pulses, PID off |
| `Z` / `C` | Zero the angle / calibrate |
| `STATUS` | `STATUS,armed/disarmed,pitch,roll` |

Drone → phone status lines: `ARMED: ...`, `DISARMED,<reason>`, `P,pitch×10,roll×10,armed` (only while the motors are off).

PuTTY keys (unchanged): `c` recalibrate, `z` zero yaw, `n` 60 s noise test, `x` KILL, `h` help.

## Sources for the Bluetooth numbers
- Adafruit, "SDEP (SPI Data Transport)": the SPI clock must be ≤ 4 MHz, the payload is 16 bytes, and there is a 100 µs wait after CS. https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le/sdep-spi-data-transport
- Nordic DevZone, "nRF51822 low TX rate (Bluefruit LE SPI Friend)": 213 B/s. https://devzone.nordicsemi.com/f/nordic-q-a/11044/nrf51822-low-tx-rate-bluefruit-le-spi-friend
- Corbin Dunn, "Tricks for fast Bluetooth LE data transfers": about 100 B/s by default, about 300 B/s with `AT+GAPINTERVALS=20,40,25,30`. https://www.corbinstreehouse.com/blog/?p=4281
- Novel Bits, "Bluetooth 5 speed": the 1 Mbps radio rate is not reachable by an application, and BLE 4.0/4.1 carries 20 bytes per packet. https://novelbits.io/bluetooth-5-speed-maximum-throughput/
