# ELC275 Capstone: Self-Leveling Quadcopter on a PIC32MZ

**Ethan DiMeo, Delaware Tech ELC275 (Microprocessor Systems), Fall 2026. Final demo: Dec 2, 2026.**

An F450 quad frame flown by a **PIC32MZ2048EFM144 EF Starter Kit**. It uses:

- an **LSM6DSO IMU** on SPI;
- four ESCs and 2212/920 KV motors;
- an **Adafruit Bluefruit LE SPI Friend** for the phone link;
- a **relay that cuts motor power** on a STOP command, when a check fails, or when the heartbeat goes missing;
- a **PID tuner** (PC app and Android app) that records step tests and graphs them.

The first stage follows the professor's plan: a tethered drone, two PWM groups (front pair and back pair), and a **pitch-only PID**.

This repo holds **every software version from the first draft to the current firmware**. It also holds every 3D-printed part, version by version.

The firmware is in the order it was written. Every `.c` / `.h` file starts with a comment block that gives:

- what the file is;
- its version and date;
- whether it worked;
- **the J12 pinout for that version**;
- a list of its functions.

**Every function and type has a title block** right above it. The code itself is exactly as it ran: only comments were added.

---

## Start here (current versions)

| What | Where |
|---|---|
| Drone firmware (MPLAB X project) | [`firmware/03_drone/v3.1_motor_adjust_buttons/Drone.X`](firmware/03_drone/v3.1_motor_adjust_buttons/Drone.X) |
| How to run v3.1: tune roll, then pitch, then both; stop a spin with the motor adjust buttons | [`firmware/03_drone/v3.1_motor_adjust_buttons/HOW_TO_RUN.md`](firmware/03_drone/v3.1_motor_adjust_buttons/HOW_TO_RUN.md) |
| PC PID tuner (USB or Bluetooth) | [`tools/pc_pid_tuner/v8_ble_motor_adjust_2026-11-25_CURRENT`](tools/pc_pid_tuner/v8_ble_motor_adjust_2026-11-25_CURRENT) |
| Android app (install the APK) | [`tools/android_drone_tuner/v3.1_motor_adjust_2026-11-25_CURRENT`](tools/android_drone_tuner/v3.1_motor_adjust_2026-11-25_CURRENT) |
| Electronics tray to print | [`cad/07_box_v5_honeycomb_tray_CURRENT`](cad/07_box_v5_honeycomb_tray_CURRENT) (+ collar v3.1 from [`cad/04_box_v3_tray_and_collar`](cad/04_box_v3_tray_and_collar)) |
| Sensor deck to print | [`cad/sensor_deck/v3_1_CURRENT`](cad/sensor_deck/v3_1_CURRENT) |
| Replacement landing leg (legs are off since Nov 25) | [`cad/landing_leg/v1_CURRENT`](cad/landing_leg/v1_CURRENT) |
| Full J12 pinout, every version | [`docs/PINOUT.md`](docs/PINOUT.md) |
| Wiring diagrams | [`docs/wiring`](docs/wiring) |

## I want to...

| ...do this | Open this (single file: copy it into `main.c`, put `config.bits` next to it) |
|---|---|
| **Find which motor is which / check wiring and spin direction** (type `RR 1150` in PuTTY) | [`01_bench_tests/10_motor_id_test`](firmware/01_bench_tests/10_motor_id_test) |
| **Make all 4 motors push the same** (kitchen-scale sweep, prints trims) | [`01_bench_tests/11_motor_calibration_test`](firmware/01_bench_tests/11_motor_calibration_test) |
| **Find the lowest pulse that lifts the drone** (props on, tied down; `ALL 1300` then `+`) | [`01_bench_tests/12_liftoff_throttle_test`](firmware/01_bench_tests/12_liftoff_throttle_test) |
| Spin **one** motor (ESC 1) slowly | [`01_bench_tests/06_motor_spin_test_esc1_only`](firmware/01_bench_tests/06_motor_spin_test_esc1_only) |
| Spin all 4 motors **one at a time** (check directions) | [`01_bench_tests/05_motor_spin_test_one_at_a_time`](firmware/01_bench_tests/05_motor_spin_test_one_at_a_time) |
| Spin all 4 motors **together** | [`01_bench_tests/08_motor_test_all4_together`](firmware/01_bench_tests/08_motor_test_all4_together) |
| **Calibrate the ESCs** (same 1.0-2.0 ms range on all 4; PuTTY only, through the relay) | [`01_bench_tests/13_esc_calibrate_relay`](firmware/01_bench_tests/13_esc_calibrate_relay) |
| Run 2 ESCs to full speed | [`01_bench_tests/01_esc_full_speed`](firmware/01_bench_tests/01_esc_full_speed) |
| Check the IMU wiring (WHO_AM_I = 0x6C) | [`01_bench_tests/04_imu_wiring_test`](firmware/01_bench_tests/04_imu_wiring_test) |
| Read IMU angles | [`01_bench_tests/03_imu_roll_pitch_yaw_spi`](firmware/01_bench_tests/03_imu_roll_pitch_yaw_spi) |
| Click the relay from the phone (START/STOP) | [`01_bench_tests/09_bluefruit_relay_test`](firmware/01_bench_tests/09_bluefruit_relay_test) |
| Run the seesaw PID | [`02_seesaw/03_SewSaw_step2-3`](firmware/02_seesaw/03_SewSaw_step2-3) |
| Find the liftoff pulse | [`01_bench_tests/12_liftoff_throttle_test`](firmware/01_bench_tests/12_liftoff_throttle_test) |
| Match the 4 motors (trims) | [`01_bench_tests/11_motor_calibration_test`](firmware/01_bench_tests/11_motor_calibration_test) |
| Fly / tune the drone | [`03_drone/v3.1_motor_adjust_buttons`](firmware/03_drone/v3.1_motor_adjust_buttons) |
| **Stop the drone spinning on the tether** (motor adjust buttons) | [`03_drone/v3.1_motor_adjust_buttons/HOW_TO_RUN.md`](firmware/03_drone/v3.1_motor_adjust_buttons/HOW_TO_RUN.md) section 5 |

## Every version, in order

| Date | Version | What changed | Folder |
|---|---|---|---|
| 2026-11-04 | Flight-controller draft | First multi-file flight controller: I2C IMU, Bluefruit Control Pad, roll/pitch/yaw PID, X mixer | [`firmware/00_flight_controller_draft`](firmware/00_flight_controller_draft) |
| 2026-11-11 | PC tuner v1 | PID tuner GUI: gain sliders, setpoint, live and step-test graphs, demo simulator | [`tools/pc_pid_tuner/v1_serial_2026-11-11`](tools/pc_pid_tuner/v1_serial_2026-11-11) |
| 2026-11-12 | Bench: ESC full speed | Both seesaw ESCs to full speed, ESC calibration mode | [`firmware/01_bench_tests/01_esc_full_speed`](firmware/01_bench_tests/01_esc_full_speed) |
| 2026-11-12 | Bench: IMU over SPI | First SPI IMU test (CS on pin 3) | [`firmware/01_bench_tests/02_imu_spi_test`](firmware/01_bench_tests/02_imu_spi_test) |
| 2026-11-12 | Bench: roll/pitch/yaw | Complementary filter on SPI | [`firmware/01_bench_tests/03_imu_roll_pitch_yaw_spi`](firmware/01_bench_tests/03_imu_roll_pitch_yaw_spi) |
| 2026-11-12 | Seesaw single file | Whole seesaw PID in one file (I2C IMU) | [`firmware/02_seesaw/01_seesaw_pid_single_file`](firmware/02_seesaw/01_seesaw_pid_single_file) |
| 2026-11-12 | PC tuner v2 | Tuner for the seesaw code | [`tools/pc_pid_tuner/v2_serial_seesaw_2026-11-12`](tools/pc_pid_tuner/v2_serial_seesaw_2026-11-12) |
| 2026-11-15 | Bench: IMU wiring test | IMU CS moved to pin 16; full SPI wiring test | [`firmware/01_bench_tests/04_imu_wiring_test`](firmware/01_bench_tests/04_imu_wiring_test) |
| 2026-11-16 | SewSaw.X step 0-1 | Multi-file project: IMU on SPI1 mode 3, filter, PuTTY + tuner link | [`firmware/02_seesaw/02_SewSaw_step0-1`](firmware/02_seesaw/02_SewSaw_step0-1) |
| 2026-11-16 | SewSaw.X step 2-3 | motors.c added: PID drives the 2 seesaw motors | [`firmware/02_seesaw/03_SewSaw_step2-3`](firmware/02_seesaw/03_SewSaw_step2-3) |
| 2026-11-17 | Bench: spin tests | Spin direction of every motor (also 06_..._esc1_only) | [`firmware/01_bench_tests/05_motor_spin_test_one_at_a_time`](firmware/01_bench_tests/05_motor_spin_test_one_at_a_time) |
| 2026-11-18 | Bench: ESC calibrate, all 4 | Calibrate all 4 ESCs, then spin all 4 together (08_...) | [`firmware/01_bench_tests/07_esc_calibrate`](firmware/01_bench_tests/07_esc_calibrate) |
| 2026-11-18 | Drone.X v0.1 | SewSaw code moved to the quad: 4 ESCs, front/back pairs, pitch PID | [`firmware/03_drone/v0.1_first_drone_firmware`](firmware/03_drone/v0.1_first_drone_firmware) |
| 2026-11-18 | PC tuner v3 | Tuner for the drone (front/back pulses) | [`tools/pc_pid_tuner/v3_serial_drone_2026-11-18`](tools/pc_pid_tuner/v3_serial_drone_2026-11-18) |
| 2026-11-18 | CAD: case v0, box rev 1, box v2, IMU adapter v1/v2, sensor deck v3 | First printed parts | [`cad`](cad) |
| 2026-11-19 | CAD: box v3 (tray + collar) | Kit right side up, clears the landing gear | [`cad/04_box_v3_tray_and_collar`](cad/04_box_v3_tray_and_collar) |
| 2026-11-20 | Drone.X v0.2 | Partner's Bluetooth STOP/ARM draft (bluefruit.c) | [`firmware/03_drone/v0.2_bluetooth_stop_draft`](firmware/03_drone/v0.2_bluetooth_stop_draft) |
| 2026-11-20 | Drone.X v1.0 | v0.1 + v0.2 merged: Bluetooth, relay on RK4, heartbeat, BLE PC tuner | [`firmware/03_drone/v1.0_ble_integration`](firmware/03_drone/v1.0_ble_integration) |
| 2026-11-20 | Drone.X v1.1 + PC tuner v5 | 9 review fixes (USB sessions, queue, re-entry, live data routing) | [`firmware/03_drone/v1.1_ble_review_fixes`](firmware/03_drone/v1.1_ble_review_fixes) |
| 2026-11-20 | CAD: collar v3.1 | The collar still in use | [`cad/04_box_v3_tray_and_collar`](cad/04_box_v3_tray_and_collar) |
| 2026-11-22 | Drone.X v1.2 | 75 deg tip cutoff; every run recorded first, sent after | [`firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs`](firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs) |
| 2026-11-22 | CAD: sensor deck v3.1 | IMU + Bluefruit deck, true-front orientation | [`cad/sensor_deck/v3_1_CURRENT`](cad/sensor_deck/v3_1_CURRENT) |
| 2026-11-23 | Bench: Bluefruit + relay | Phone START/STOP switches the relay | [`firmware/01_bench_tests/09_bluefruit_relay_test`](firmware/01_bench_tests/09_bluefruit_relay_test) |
| 2026-11-23 | Drone.X v2.0 | Phone app over Bluetooth, relay kill, 500 ms heartbeat, step-test arrays to the phone | [`firmware/03_drone/v2.0_phone_app_relay_heartbeat`](firmware/03_drone/v2.0_phone_app_relay_heartbeat) |
| 2026-11-23 | PC tuner v6 + Android app 2.0 | Tuners for the v2.0 protocol | [`tools`](tools) |
| 2026-11-24 | CAD: tray v4, tray v4.1 | Relay bay in the back lobe, slot for the battery bullet leads | [`cad/06_box_v4_1_tray`](cad/06_box_v4_1_tray) |
| 2026-11-25 | Bench: motor ID test | PuTTY command spins the motor you name (RR 1150): wiring + direction check | [`firmware/01_bench_tests/10_motor_id_test`](firmware/01_bench_tests/10_motor_id_test) |
| 2026-11-25 | Bench: motor calibration test | Kitchen-scale thrust sweep, prints per-motor pulse trims | [`firmware/01_bench_tests/11_motor_calibration_test`](firmware/01_bench_tests/11_motor_calibration_test) |
| 2026-11-25 | Bench: liftoff throttle test | Full-range (to 2000 us) PuTTY control with +/- steps to find the liftoff pulse | [`firmware/01_bench_tests/12_liftoff_throttle_test`](firmware/01_bench_tests/12_liftoff_throttle_test) |
| 2026-11-25 | Bench: ESC calibrate through the relay | PuTTY-only ESC calibration for the assembled drone: 2.0 ms first, then RK4 powers the ESCs | [`firmware/01_bench_tests/13_esc_calibrate_relay`](firmware/01_bench_tests/13_esc_calibrate_relay) |
| 2026-11-25 | Drone.X v3.0 | Professor's notes: roll PID + pitch PID, 4 different pulses, tune one axis at a time, then both; soft start, motor trims | [`firmware/03_drone/v3.0_two_pid_roll_pitch`](firmware/03_drone/v3.0_two_pid_roll_pitch) |
| 2026-11-25 | PC tuner v7 + Android app 3.0 | Axis select, roll and pitch gains, throttle T, Hold row for the demo | [`tools`](tools) |
| 2026-11-25 | **Drone.X v3.1 (current)** | The drone spun counterclockwise on the tether: live + / - per motor (O command), turn CW / CCW buttons | [`firmware/03_drone/v3.1_motor_adjust_buttons`](firmware/03_drone/v3.1_motor_adjust_buttons) |
| 2026-11-25 | PC tuner v8 + Android app 3.1 | Motor adjust buttons for the v3.1 O command | [`tools`](tools) |
| 2026-11-25 | CAD: **tray v5 (current)** | Legs off: bigger honeycomb tray, flat bottom to land on, same collar v3.1 | [`cad/07_box_v5_honeycomb_tray_CURRENT`](cad/07_box_v5_honeycomb_tray_CURRENT) |
| 2026-11-25 | CAD: replacement landing leg v1 | A stock leg broke on the tether: printed copy on the measured bracket, about 20.8 g | [`cad/landing_leg/v1_CURRENT`](cad/landing_leg/v1_CURRENT) |

## Repo layout

```text
Drone-Project/
├── firmware/
│   ├── 00_flight_controller_draft/   Nov 4 practice flight controller (I2C IMU, 400 Hz ESC frame)
│   ├── 01_bench_tests/               one-file test programs: ESCs, motors, IMU, relay, motor ID, calibration, liftoff
│   ├── 02_seesaw/                    the 2-motor seesaw rig (single file, then SewSaw.X)
│   └── 03_drone/                     Drone.X v0.1 -> v3.1 (each a complete MPLAB X project)
├── tools/
│   ├── pc_pid_tuner/                 PC tuner GUI, v1 -> v8 (which firmware each pairs with)
│   └── android_drone_tuner/          phone app, v2.0, v3.0 and v3.1 (APK + source)
├── cad/                              every printed part: STL to print, STEP to edit, renders, CadQuery source
└── docs/                             pinout (every version), wiring diagrams, pinout sheets, history
```

## How the files are laid out

- **Single-file tests** (`01_bench_tests`, `02_seesaw/01_...`) are one `.c` file plus `config.bits`.
  Open the `.c` on GitHub and click **Copy raw file**. Paste it into `main.c` in a new MPLAB X project
  (PIC32MZ2048EFM144, XC32). Copy `config.bits` into the project folder.
  Never add `config.bits` as a source file: `main.c` `#include`s it.
- **Projects** (`*.X` folders) open directly in MPLAB X: **File > Open Project**.
- Each folder has a README with:
  - the version, date and status;
  - the J12 pinout;
  - how to build it;
  - what every file is.
- Function title blocks look like this:

```c
// ---------------------------------------------------------------------------------------------
// float pid(float e, float edot, float Kp, float Ki, float Kd, float *eint, float eintmax, float umax)
//   One PID step: u = Kp*e + Ki*integral(e) + Kd*edot, with anti-windup and an output limit.
// ---------------------------------------------------------------------------------------------
```

## Current J12 pinout (v3.1, same wiring as v2.0)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
  VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g. 9)
  SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

The pinout of every older version is in [`docs/PINOUT.md`](docs/PINOUT.md).

## The math in the code

**ESC pulses (Timer2 + OC1-OC4, from the seesaw on).**

$$f_{T2}=\frac{60\ \text{MHz}}{64}=937{,}500\ \text{ticks/s},\qquad T=\frac{(PR2+1)\cdot 64}{60\ \text{MHz}}=\frac{18750\cdot 64}{60\times10^{6}}=20\ \text{ms}\ (50\ \text{Hz})$$

$$\text{ticks}=937.5\cdot t_{\text{pulse}}[\text{ms}]:\qquad 1.00\ \text{ms}\to 938\ (\text{stop}),\quad 1.25\ \text{ms}\to 1172,\quad 2.00\ \text{ms}\to 1875\ (\text{full})$$

The Nov 4 draft used Timer2 at 1:8 instead. Its frame was $T=\frac{18750\cdot 8}{60\ \text{MHz}}=2.5\ \text{ms}$ (400 Hz), so 1 ms = 7500 ticks.

**Control loop (Timer4 interrupt).**

$$T_{\text{loop}}=\frac{(PR4+1)\cdot 64}{60\ \text{MHz}}=\frac{3750\cdot 64}{60\times10^{6}}=4\ \text{ms}\quad(250\ \text{Hz})$$

**Angle from the accelerometer** (gravity only; signs: pitch $+$ = nose up, roll $+$ = left side up):

$$\theta_{\text{roll,acc}}=\operatorname{atan2}(a_y,\,a_z),\qquad \theta_{\text{pitch,acc}}=\operatorname{atan2}\!\left(a_x,\ \sqrt{a_y^{2}+a_z^{2}}\right)$$

**Complementary filter** (98% gyro, 2% gravity; $\Delta t=0.004\ \text{s}$):

$$\theta_k=\alpha\left(\theta_{k-1}+\omega_k\,\Delta t\right)+(1-\alpha)\,\theta_{\text{acc},k},\qquad \alpha=0.98,\qquad \tau=\frac{\alpha\,\Delta t}{1-\alpha}\approx 0.196\ \text{s}$$

**PID, written the way Kevin Lynch writes it** (`pid.c`):

$$e=r-\theta,\qquad \dot e=\dot r-\dot\theta=-\omega_{\text{gyro}},\qquad E_k=\operatorname{clamp}\!\left(E_{k-1}+e_k\,\Delta t,\ \pm E_{\max}\right)$$

$$u_k=\operatorname{clamp}\!\left(K_p\,e_k+K_i\,E_k+K_d\,\dot e_k,\ \pm u_{\max}\right),\qquad E_{\max}=20\ ^{\circ}\!\cdot\text{s},\quad u_{\max}=500\ \mu\text{s}\ (\text{v2.0, v3.0})$$

**Mixer, v0.1 to v2.0 (pitch only, two PWM groups):**

$$\text{front (ESC 1, 2)}=\text{base}_{\text{front}}+u,\qquad \text{back (ESC 3, 4)}=\text{base}_{\text{back}}-u$$

**Mixer, v3.0 (professor's notes: two PIDs, all four pulses different).** The roll PID gives $RC$, the pitch PID gives $PC$, and $T$ is the throttle (slightly above liftoff):

$$
\begin{aligned}
m_{FL}&=T+PC+RC, & m_{FR}&=T+PC-RC,\\
m_{RL}&=T-PC+RC, & m_{RR}&=T-PC-RC.
\end{aligned}
$$

Tuning one axis at a time sets the other correction to 0: roll only ($PC=0$) is the left pair against the right pair, pitch only ($RC=0$) is the front pair against the back pair, the seesaw both times.
The left pair gets $+RC$ because roll $+$ = left side up: if the left is too high, $e_r<0$, so $RC<0$ and the left motors slow down.

In PID mode each pulse is kept at or above $1100\ \mu\text{s}$ so the props keep turning. Every pulse is capped at $2000\ \mu\text{s}$.
In v3.0 each motor's trim from bench test 11 is applied last: $p'=1000+g\,(p-1000)+o$.
v3.1 then adds the live offset $a_n$ from the app's motor adjust buttons ($|a_n|\le 200\ \mu\text{s}$). The turn buttons move diagonal pairs: $a_{FL}=a_{RR}=+d$, $a_{FR}=a_{RL}=-d$. The left pair and the front pair keep the same sum, so only the yaw twist changes.

**Safety (v2.0 and v3.0).** The motors go off and the relay opens on any of these:

- SW1;
- an IMU error;
- $|\theta_{\text{pitch}}|$ or $|\theta_{\text{roll}}|>75^{\circ}$ ($60^{\circ}$ in v3.0);
- a STOP from the phone or the PC;
- no heartbeat `H` for $500\ \text{ms}$ from the link that armed it.

## Safety rules for every program here

- **Props OFF** for every bench test. **SW1 = STOP** in every program.
- **Never wire J12 pins 8 and 10**: they are the PC serial link (PuTTY/tuner). **Never press SW3**: it is the same pin as the serial transmit.
- Wires are crimped / plug-in only (no cutting and soldering, the class rule).
- The ESC red 5 V wires are not connected.

## Not in this repo

The datasheets and the course material are not in this repo. Use the official copies:

- [LSM6DSO datasheet (ST)](https://www.st.com/resource/en/datasheet/lsm6dso.pdf)
- [PIC32MZ2048EFM144 (Microchip)](https://www.microchip.com/en-us/product/PIC32MZ2048EFM144)
- [PIC32MZ EF Starter Kit, DM320007 (Microchip)](https://www.microchip.com/en-us/development-tool/DM320007)
- [Adafruit Bluefruit LE SPI Friend guide](https://learn.adafruit.com/introducing-the-adafruit-bluefruit-spi-breakout)

The files these were checked against are:

- DS60001320H (PIC32MZ EF datasheet);
- DS70005230B (Starter Kit User's Guide, Table 2-3 "J12 Connections").
