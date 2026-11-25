# PC PID tuner v8 (USB or Bluetooth, v3.1: two axes + motor adjust buttons)  (2026-11-25)

Pairs with: **Drone.X v3.1 (current)**

| File | What it is |
|---|---|
| `pid_tuner_ble.py` | The tuner program (Python 3, PySide6 + pyqtgraph GUI). |
| `run_tuner_ble.bat` | Windows launcher: double-click it (installs what it needs the first time). |
| `tests/test_v8.py` | Offscreen test of this tuner: the v3.1 protocol, the axis / gain controls, the motor adjust buttons, and REAL firmware output (run the v3.1 `host_tests/run_tests.sh` first; it writes `host_tests/fw_out/`). |

**v8 for firmware 3.1:** everything in v7, plus a **Motor adjust** box: + / - on each motor while flying (step 5 / 10 / 25 us), and turn CW / turn CCW for a yaw spin (diagonal pairs, so roll and pitch stay balanced). Sends `O,fl,fr,rl,rr`. Guide: [`HOW_TO_RUN.md`](../../../firmware/03_drone/v3.1_motor_adjust_buttons/HOW_TO_RUN.md) section 5.

Run it on the PC that is connected to the board (USB serial, or Bluetooth for the BLE versions).
Close PuTTY first: only one program can hold the COM port.
