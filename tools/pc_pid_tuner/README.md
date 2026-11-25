# PC PID tuner: every version

It is a Python GUI. It has:

- Kp / Ki / Kd sliders;
- a setpoint;
- live graphs;
- recorded step tests with rise time, overshoot, settling and error;
- a demo simulator that needs no board.

v1-v3 talk over the USB serial port. v4-v7 can also use Bluetooth LE. Use the version that matches the firmware
(v7 is the only one that can select the axis and send roll gains, for Drone.X v3.0):

| Version | Date | Pairs with firmware |
|---|---|---|
| [`v1_serial_2026-11-11`](v1_serial_2026-11-11) | 2026-11-11 | 00 flight-controller draft, SewSaw step 0-1 |
| [`v2_serial_seesaw_2026-11-12`](v2_serial_seesaw_2026-11-12) | 2026-11-12 | seesaw single-file code, SewSaw step 2-3 |
| [`v3_serial_drone_2026-11-18`](v3_serial_drone_2026-11-18) | 2026-11-18 | Drone.X v0.1 and v0.2 (USB cable) |
| [`v4_ble_2026-11-20`](v4_ble_2026-11-20) | 2026-11-20 | Drone.X v1.0 |
| [`v5_ble_2026-11-20`](v5_ble_2026-11-20) | 2026-11-20 | Drone.X v1.1 and v1.2 |
| [`v6_ble_2026-11-23`](v6_ble_2026-11-23) | 2026-11-23 | Drone.X v2.0 |
| [`v7_ble_two_axis_2026-11-25_CURRENT`](v7_ble_two_axis_2026-11-25_CURRENT) | 2026-11-25 | Drone.X v3.0 (current) |
