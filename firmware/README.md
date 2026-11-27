# Firmware: every version

All programs are for the **PIC32MZ2048EFM144 EF Starter Kit** (MPLAB X 6.15, XC32). They use the same `config.bits`: 120 MHz SYSCLK, 60 MHz PBCLK.
The folders are in the order they were written:

| Folder | What is in it |
|---|---|
| `00` | The Nov 4 draft. |
| `01` | Small test programs. |
| `02` | The 2-motor seesaw rig. |
| `03` | The drone. |

Every folder has a README with its J12 pinout, its status and how to build it.

| Folder | What | Date | Status |
|---|---|---|---|
| [`00_flight_controller_draft`](00_flight_controller_draft) | First flight-controller draft (multi-file, I2C IMU, Bluefruit app control) | 2026-11-04 | Written as practice code before the hardware arrived. Never flown. Superseded by the SewSaw seesaw code (the IMU moved to SPI). |
| [`01_bench_tests/01_esc_full_speed`](01_bench_tests/01_esc_full_speed) | ESC full-speed test (2 motors) + ESC calibration mode | 2026-11-12 | WORKED on the seesaw: both motors armed and ran to full speed. |
| [`01_bench_tests/02_imu_spi_test`](01_bench_tests/02_imu_spi_test) | Basic IMU test over SPI (WHO_AM_I, accel, gyro, roll/pitch) | 2026-11-12 | Builds. SPI reads were unreliable on this wiring (CS on pin 3); fixed later by moving CS to pin 16. |
| [`01_bench_tests/03_imu_roll_pitch_yaw_spi`](01_bench_tests/03_imu_roll_pitch_yaw_spi) | IMU roll / pitch / yaw over SPI (complementary filter) | 2026-11-12 | Builds. Same SPI wiring problem as 02 at the time (CS on pin 3). |
| [`01_bench_tests/04_imu_wiring_test`](01_bench_tests/04_imu_wiring_test) | IMU wiring test - LSM6DSO over SPI with CS on pin 16 | 2026-11-15 | WORKED: WHO_AM_I reads 0x6C on the real board. Use this first whenever the IMU wiring changes. |
| [`01_bench_tests/05_motor_spin_test_one_at_a_time`](01_bench_tests/05_motor_spin_test_one_at_a_time) | Motor spin-direction test - all 4 ESCs, one motor at a time | 2026-11-17 | WORKED: spin directions verified Nov 18, props off (FL CCW, FR CW, RL CW, RR CCW). Two ESCs did not start at 1.09 ms, which led to 07_esc_calibrate. |
| [`01_bench_tests/06_motor_spin_test_esc1_only`](01_bench_tests/06_motor_spin_test_esc1_only) | Motor spin test - ESC 1 only, slow | 2026-11-17 | Ran on the bench Nov 17: ESC 1 needed 1.25 ms to start (1.15 ms was too low). |
| [`01_bench_tests/07_esc_calibrate`](01_bench_tests/07_esc_calibrate) | ESC throttle calibration - all 4 ESCs at once | 2026-11-18 | Calibration program written Nov 18 (build it with Make and Program Device, not Debug). No result is logged in the project notes. Since the relay went in (Nov 19) it can no longer power the ESCs: use 13_esc_calibrate_relay. |
| [`01_bench_tests/08_motor_test_all4_together`](01_bench_tests/08_motor_test_all4_together) | Motor test - all 4 ESCs, same pulse, same time | 2026-11-18 | Written Nov 18 to run after calibration (ramp to 1.25 ms, hard cap 2.0 ms). No result is logged in the project notes. |
| [`01_bench_tests/09_bluefruit_relay_test`](01_bench_tests/09_bluefruit_relay_test) | Bluefruit + relay test - phone START/STOP switches the motor-power relay | 2026-11-23 | WORKED: the relay clicks on START and off on STOP from the phone. |
| [`01_bench_tests/10_motor_id_test`](01_bench_tests/10_motor_id_test) | Motor ID test - spin the motor you name from PuTTY (RR 1150) to check wiring and direction | 2026-11-25 | New Nov 25. Checked in a PC simulation (17 checks: each name drives the right pin and tick count, slow ramp, relay arming, SW1, bad input). Not yet run on the board. |
| [`01_bench_tests/11_motor_calibration_test`](01_bench_tests/11_motor_calibration_test) | Motor calibration test - kitchen-scale thrust sweep that prints per-motor pulse trims | 2026-11-25 | New Nov 25. Checked in a PC simulation with 4 deliberately mismatched motors: the printed trims bring them within 0.4% of each other at every level; R/X/SW1 checked. Nov 25: Esc/Space stop keys added for the assembled drone (no kit buttons), checked mid-ramp and at a prompt (7 checks). Not yet run on the board. |
| [`01_bench_tests/12_liftoff_throttle_test`](01_bench_tests/12_liftoff_throttle_test) | Liftoff throttle test - full-range (to 2000 us) motor control from PuTTY with +/- steps, to find the lowest pulse that lifts the drone | 2026-11-25 | New Nov 25. Checked in a PC simulation (16 checks: ALL/one motor up to 2000 us, +/- steps, 2000 us cap, instant stop on Esc/Space/X, SW1, relay). Not yet run on the board. Props ON: the drone must be tied down. |
| [`01_bench_tests/13_esc_calibrate_relay`](01_bench_tests/13_esc_calibrate_relay) | ESC throttle calibration from PuTTY, through the motor-power relay - all 4 ESCs at once, no kit buttons | 2026-11-25 | New Nov 25: 07_esc_calibrate redone for the assembled drone: PuTTY commands instead of SW1/SW2, and the relay is closed by the program (2.0 ms first, then RK4). Checked in a PC simulation (26 checks). Not yet run on the board. Props OFF. |
| [`02_seesaw/01_seesaw_pid_single_file`](02_seesaw/01_seesaw_pid_single_file) | Seesaw PID - single file (I2C IMU, 2 motors) | 2026-11-12 | Builds. Still uses the IMU on I2C; replaced by the SPI multi-file SewSaw project. |
| [`02_seesaw/02_SewSaw_step0-1`](02_seesaw/02_SewSaw_step0-1) | SewSaw.X - seesaw firmware, steps 0-1 (IMU on SPI, filter, PC tuner link) | 2026-11-16 | WORKED: step 0 (IMU check) passed and step 1 ran on the board Nov 16. |
| [`02_seesaw/03_SewSaw_step2-3`](02_seesaw/03_SewSaw_step2-3) | SewSaw.X - seesaw firmware, steps 2-3 (motors + PID on the seesaw) | 2026-11-16 | Written and handed over; the seesaw stage was marked done Nov 18 (no separate hardware log). |
| [`03_drone/v0.1_first_drone_firmware`](03_drone/v0.1_first_drone_firmware) | Drone.X v0.1 - first 4-motor firmware (SewSaw code moved to the quad) | 2026-11-18 | Host-checked only; not built in MPLAB as of Nov 18. Base for every later version. |
| [`03_drone/v0.2_bluetooth_stop_draft`](03_drone/v0.2_bluetooth_stop_draft) | Drone.X v0.2 - partner's Bluetooth STOP draft (adds bluefruit.c) | 2026-11-20 | Draft source only (no MPLAB project files). Merged into v1.0. |
| [`03_drone/v1.0_ble_integration`](03_drone/v1.0_ble_integration) | Drone.X v1.0 - BLE integration (Bluetooth STOP/ARM, relay, BLE PC tuner) | 2026-11-20 | Partner's merge of v0.1 + v0.2. Not built with XC32. Replaced by v1.1 the same day. |
| [`03_drone/v1.1_ble_review_fixes`](03_drone/v1.1_ble_review_fixes) | Drone.X v1.1 - review fixes on top of v1.0 | 2026-11-20 | Host tests pass. Not built with XC32. |
| [`03_drone/v1.2_tip_cutoff_and_recorded_runs`](03_drone/v1.2_tip_cutoff_and_recorded_runs) | Drone.X v1.2 - 75 deg tip cutoff + record-then-send step runs | 2026-11-22 | Host tests pass. Not built with XC32. |
| [`03_drone/v2.0_phone_app_relay_heartbeat`](03_drone/v2.0_phone_app_relay_heartbeat) | Drone.X v2.0 - phone app over Bluetooth, relay kill, 500 ms heartbeat | 2026-11-23 | Builds in MPLAB X with XC32 and was flown on the tethered bench (Nov 24). Host tests pass. The copy on the laptop has TIP_LIMIT 60 (this repo copy: 75). Replaced by v3.0 (two PIDs). |
| [`03_drone/v3.0_two_pid_roll_pitch`](03_drone/v3.0_two_pid_roll_pitch) | Drone.X v3.0 - roll PID + pitch PID, 4 different ESC pulses, one axis at a time | 2026-11-25 | Builds in MPLAB X with XC32 (built on the laptop Nov 25). The drone spinning counterclockwise on the tether led to v3.1. Host tests pass (78 checks, including a simulated flight with the real 4 ms loop). |
| [`03_drone/v3.1_motor_adjust_buttons`](03_drone/v3.1_motor_adjust_buttons) | Drone.X v3.1 - v3.0 + live +/- per motor (O command, app buttons) for a yaw spin | 2026-11-25 | Built in MPLAB on the laptop (Nov 25). v3.0 plus the O command. Host tests pass (87 checks). Replaced by v3.2 (same code, Bluefruit CS on pin 40). |
| [`03_drone/v3.2_ble_cs_pin40`](03_drone/v3.2_ble_cs_pin40) | Drone.X v3.2 - v3.1 with the Bluefruit CS on J12 pin 40 (RJ3)  (CURRENT) | 2026-11-26 | CURRENT. v3.1 with the Bluefruit CS moved from pin 13 (RJ13) to pin 40 (RJ3); nothing else changed. Host tests pass (87 checks; the Bluetooth simulation checks CS on RJ3 for every byte). Not yet built in MPLAB. |
