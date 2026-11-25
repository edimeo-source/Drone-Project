# Drone Tuner - Android app, every version

Phone app for the drone: connects to the Bluefruit over Bluetooth LE (Nordic UART service), ARMs / KILLs the drone,
sends the heartbeat every 100 ms, changes PID gains and runs step tests (the drone records the run, then sends the
arrays to the phone).

| Folder | Date | Pairs with firmware |
|---|---|---|
| [`v2.0_2026-11-23`](v2.0_2026-11-23) | 2026-11-23 | Drone.X v2.0 (pitch PID only) |
| [`v3.0_two_axis_2026-11-25_CURRENT`](v3.0_two_axis_2026-11-25_CURRENT) | 2026-11-25 | Drone.X v3.0 (roll + pitch PIDs) - **current** |
| [`build_scripts`](build_scripts) | | How the APKs were built without Android Studio (javac + dx + aapt2 + a v1/v2/v3 signer). The signing key is **not** in the repo. |

Install: copy `DroneTuner.apk` to the phone and open it (allow "install unknown apps" once).
To build it yourself, open `android_app/` in Android Studio (minSdk 24, targetSdk 33) and press Run.
