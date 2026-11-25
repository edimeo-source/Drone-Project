# Drone firmware: Drone.X v0.1 to v2.0

Each folder is a complete MPLAB X project (`Drone.X`). v0.2 is source only.
v1.1 and later also have **host tests**: they run the firmware logic on a PC with gcc (`sh host_tests/run_tests.sh`).

Control is the professor's first-stage plan:

- one PID on **pitch**;
- front pair (ESC 1 + 2) = base + u;
- back pair (ESC 3 + 4) = base - u.

| Version | Date | What changed | Status |
|---|---|---|---|
| [`v0.1_first_drone_firmware`](v0.1_first_drone_firmware) | 2026-11-18 | SewSaw code moved to the quad: 4 ESCs as a front pair and a back pair, pitch PID, drone orientation, PuTTY keys c z n x h. | Host-checked only; not built in MPLAB as of Nov 18. Base for every later version. |
| [`v0.2_bluetooth_stop_draft`](v0.2_bluetooth_stop_draft) | 2026-11-20 | Partner's draft: adds `bluefruit.c`, phone commands `STOP;` `ARM;` `STATUS`. STOP latches all ESCs at 1000 us. | Draft source only (no MPLAB project files). Merged into v1.0. |
| [`v1.0_ble_integration`](v1.0_ble_integration) | 2026-11-20 | v0.1 and v0.2 merged. Adds the motor-power relay on RK4 (pin 33), the heartbeat and a Bluetooth PC tuner. | Partner's merge of v0.1 + v0.2. Not built with XC32. Replaced by v1.1 the same day. |
| [`v1.1_ble_review_fixes`](v1.1_ble_review_fixes) | 2026-11-20 | 9 review fixes:
- USB sessions can spin the motors.
- The step-test dump no longer blocks.
- An 8-line Bluetooth queue.
- A re-entry guard.
- DISARMED goes to every link. | Host tests pass. Not built with XC32. |
| [`v1.2_tip_cutoff_and_recorded_runs`](v1.2_tip_cutoff_and_recorded_runs) | 2026-11-22 | 75 deg tip cutoff. Every step test is recorded first and sent after the motors stop (the professor's rule). | Host tests pass. Not built with XC32. |
| [`v2.0_phone_app_relay_heartbeat`](v2.0_phone_app_relay_heartbeat) | 2026-11-23 | Phone app over Bluetooth. Simple blocking Bluefruit driver that shares SPI1 IMU-first. Relay kill. 500 ms heartbeat. Step-test arrays go to the phone. **Current.** | CURRENT. Builds in MPLAB X with XC32 and was flown on the tethered bench (Nov 24). Host tests pass. The copy on the laptop has TIP_LIMIT 60 (this repo copy: 75). |
