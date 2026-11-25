# Drone Tuner - Android app  (v2.0, 2026-11-23)

Phone app for **Drone.X v2.0**: connects to the Bluefruit over Bluetooth LE (Nordic UART service),
ARMs / STOPs the drone, sends the 500 ms heartbeat, changes PID gains and runs step tests
(the drone records the run, then sends the arrays to the phone).

| File | What it is |
|---|---|
| `DroneTuner.apk` | Ready-to-install app (Android 7.0+). Copy it to the phone and open it (allow "install unknown apps"). |
| `android_app/` | Source: `MainActivity.java` (the whole app), `AndroidManifest.xml`, icons, Gradle files. |
| `build_scripts/` | How the APK was built without Android Studio (aapt2 + javac + dx + a v1/v2/v3 signer). The signing key is **not** in the repo. |

Status: built and signature-checked; not yet tested on a real phone against the drone.
To build it yourself, open `android_app/` in Android Studio (minSdk 24, targetSdk 33) and press Run.
