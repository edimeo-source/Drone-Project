# PC PID tuner v7 (USB or Bluetooth, two-axis v3.0 protocol)  (2026-11-25)

Pairs with: **Drone.X v3.0 (current)**

| File | What it is |
|---|---|
| `pid_tuner_ble.py` | The tuner program (Python 3, PySide6 + pyqtgraph GUI). |
| `run_tuner_ble.bat` | Windows launcher: double-click it (installs what it needs the first time). |
| `tests/test_v7.py` | Offscreen test of this tuner: the v3.0 protocol, the axis / gain controls, and REAL firmware output (run the v3.0 `host_tests/run_tests.sh` first; it writes `host_tests/fw_out/`). |

**v7 for firmware 3.0:** pick the axis (Roll only / Pitch only / Both), roll and pitch keep their own gains (KR / KP), one throttle T, soft-start rate, and a Hold row for the demo (roll 5, pitch 10). Bench steps: [`HOW_TO_RUN.md`](../../../firmware/03_drone/v3.0_two_pid_roll_pitch/HOW_TO_RUN.md).

Run it on the PC that is connected to the board (USB serial, or Bluetooth for the BLE versions).
Close PuTTY first: only one program can hold the COM port.
