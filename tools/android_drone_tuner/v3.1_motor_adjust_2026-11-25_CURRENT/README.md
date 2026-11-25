# Drone Tuner app v3.1  (2026-11-25)  - CURRENT

For **Drone.X v3.1** (also fine with 3.0, without the motor adjust). Everything in [3.0](../v3.0_two_axis_2026-11-25), plus:

- **Motor adjust** card, right under the angles (close to KILL): + / - on each motor (FL, FR, RL, RR), step 5 / 10 / 25 us.
  Works while flying. Sends `O,fl,fr,rl,rr`; "on the drone:" shows the `OFFS` reply.
- **Turn CW / turn CCW**: FL and RR (CCW props) against FR and RL (CW props), so roll and pitch stay balanced.
  The drone spins counterclockwise seen from above: press turn CW.
- The values are saved on the phone and sent on every connect (the drone starts at 0 at power-up).

| File | What it is |
|---|---|
| `DroneTuner.apk` | Ready to install (Android 7.0+). Same signing key as 2.0 / 3.0, so it installs as an update. |
| `android_app/` | Source: `MainActivity.java` (the whole app), `AndroidManifest.xml`, icons, Gradle files. |
| `test/` | PC test (JVM): run parser, 4-motor mixer and the turn buttons, against REAL firmware 3.1 output. |

Status: built and signature-checked, tested on the PC against the firmware's own output. Not yet run on a phone.
Guide: [`HOW_TO_RUN.md`](../../../firmware/03_drone/v3.1_motor_adjust_buttons/HOW_TO_RUN.md) section 5.

Test: run `sh host_tests/run_tests.sh` in [`firmware/03_drone/v3.1_motor_adjust_buttons`](../../../firmware/03_drone/v3.1_motor_adjust_buttons) first, then
`ANDROID_JAR=<SDK>/platforms/android-33/android.jar sh test/run_test.sh` here.
