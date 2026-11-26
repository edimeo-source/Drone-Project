# Bench tests

Each test is a single `.c` file, or a small MPLAB project for 05 and 07. They are short programs used to bring up one part at a time.
To use a single-file test:

1. Make a new MPLAB X project (PIC32MZ2048EFM144, XC32).
2. Paste the file into `main.c`.
3. Put `config.bits` from the same folder next to it.

The full pinout is in the comment at the top of the file and in the folder's README.

**Props off. SW1 = STOP in every test.**

| Folder | Test | Date | Status |
|---|---|---|---|
| [`01_esc_full_speed`](01_esc_full_speed) | ESC full-speed test (2 motors) + ESC calibration mode | 2026-11-12 | WORKED on the seesaw: both motors armed and ran to full speed. |
| [`02_imu_spi_test`](02_imu_spi_test) | Basic IMU test over SPI (WHO_AM_I, accel, gyro, roll/pitch) | 2026-11-12 | Builds. SPI reads were unreliable on this wiring (CS on pin 3); fixed later by moving CS to pin 16. |
| [`03_imu_roll_pitch_yaw_spi`](03_imu_roll_pitch_yaw_spi) | IMU roll / pitch / yaw over SPI (complementary filter) | 2026-11-12 | Builds. Same SPI wiring problem as 02 at the time (CS on pin 3). |
| [`04_imu_wiring_test`](04_imu_wiring_test) | IMU wiring test - LSM6DSO over SPI with CS on pin 16 | 2026-11-15 | WORKED: WHO_AM_I reads 0x6C on the real board. Use this first whenever the IMU wiring changes. |
| [`05_motor_spin_test_one_at_a_time`](05_motor_spin_test_one_at_a_time) | Motor spin-direction test - all 4 ESCs, one motor at a time | 2026-11-17 | WORKED: spin directions verified Nov 18, props off (FL CCW, FR CW, RL CW, RR CCW). Two ESCs did not start at 1.09 ms, which led to 07_esc_calibrate. |
| [`06_motor_spin_test_esc1_only`](06_motor_spin_test_esc1_only) | Motor spin test - ESC 1 only, slow | 2026-11-17 | Ran on the bench Nov 17: ESC 1 needed 1.25 ms to start (1.15 ms was too low). |
| [`07_esc_calibrate`](07_esc_calibrate) | ESC throttle calibration - all 4 ESCs at once | 2026-11-18 | Calibration program written Nov 18 (build it with Make and Program Device, not Debug). No result is logged in the project notes. Since the relay went in (Nov 19) it can no longer power the ESCs: use 13_esc_calibrate_relay. |
| [`08_motor_test_all4_together`](08_motor_test_all4_together) | Motor test - all 4 ESCs, same pulse, same time | 2026-11-18 | Written Nov 18 to run after calibration (ramp to 1.25 ms, hard cap 2.0 ms). No result is logged in the project notes. |
| [`09_bluefruit_relay_test`](09_bluefruit_relay_test) | Bluefruit + relay test - phone START/STOP switches the motor-power relay | 2026-11-23 | WORKED: the relay clicks on START and off on STOP from the phone. |
| [`10_motor_id_test`](10_motor_id_test) | Motor ID test - spin the motor you name from PuTTY (RR 1150) to check wiring and direction | 2026-11-25 | New Nov 25. Checked in a PC simulation (17 checks: each name drives the right pin and tick count, slow ramp, relay arming, SW1, bad input). Not yet run on the board. |
| [`11_motor_calibration_test`](11_motor_calibration_test) | Motor calibration test - kitchen-scale thrust sweep that prints per-motor pulse trims | 2026-11-25 | New Nov 25. Checked in a PC simulation with 4 deliberately mismatched motors: the printed trims bring them within 0.4% of each other at every level; R/X/SW1 checked. Nov 25: Esc/Space stop keys added for the assembled drone (no kit buttons), checked mid-ramp and at a prompt (7 checks). Not yet run on the board. |
| [`12_liftoff_throttle_test`](12_liftoff_throttle_test) | Liftoff throttle test - full-range (to 2000 us) motor control from PuTTY with +/- steps, to find the lowest pulse that lifts the drone | 2026-11-25 | New Nov 25. Checked in a PC simulation (16 checks: ALL/one motor up to 2000 us, +/- steps, 2000 us cap, instant stop on Esc/Space/X, SW1, relay). Not yet run on the board. Props ON: the drone must be tied down. |
| [`13_esc_calibrate_relay`](13_esc_calibrate_relay) | ESC throttle calibration from PuTTY, through the motor-power relay - all 4 ESCs at once, no kit buttons | 2026-11-25 | New Nov 25: 07_esc_calibrate redone for the assembled drone: PuTTY commands instead of SW1/SW2, and the relay is closed by the program (2.0 ms first, then RK4). Checked in a PC simulation (26 checks). Not yet run on the board. Props OFF. |
