# Drone Tuner app v3.0  (2026-11-25)

For **Drone.X v3.0**: two PIDs, tuned one axis at a time (professor's notes).
[v3.1](../v3.1_motor_adjust_2026-11-25_CURRENT) adds the motor adjust buttons and works with firmware 3.0 too.

| File | What it is |
|---|---|
| `DroneTuner.apk` | Ready to install (Android 7.0+). Same signing key as v2.0, so it installs as an update. |
| `android_app/` | Source: `MainActivity.java` (the whole app), `AndroidManifest.xml`, icons, Gradle files. |
| `test/` | PC test (JVM) of the app's Bluetooth run parser and 4-motor mixer against REAL firmware output. |

New in 3.0:
- **Axis** buttons: Roll / Pitch / Both (`A,1` / `A,2` / `A,3`). The highlighted one is what the drone reports.
- **Roll and pitch gains**, each saved on the phone (`KR`, `KP`). Both "in use" lines come back from the drone.
- **Throttle T** for all four motors and the **soft start** rate (`B,T`, `U,rate`).
- **Hold angle** card for the demo (`SR`, `SP`): roll 5, pitch 10, then both.
- Step tests up to 15 s. In Both, the graph shows roll and pitch and all four ESC pulses (FL, FR, RL, RR), with step numbers for each axis.
- The CSV has every column for the axis that ran.

Status: built and signature-checked, and the parser and mixer were tested against the firmware's own output.
Not yet run on a phone against the drone.

Test: run `sh host_tests/run_tests.sh` in [`firmware/03_drone/v3.0_two_pid_roll_pitch`](../../../firmware/03_drone/v3.0_two_pid_roll_pitch) first, then
`ANDROID_JAR=<SDK>/platforms/android-33/android.jar sh test/run_test.sh` here.
