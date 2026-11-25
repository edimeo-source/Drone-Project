# PC PID tuner v6 (Bluetooth, v2.0 protocol)  (2026-11-23)

Pairs with: **Drone.X v2.0**

| File | What it is |
|---|---|
| `pid_tuner_ble.py` | The tuner program (Python 3, PySide6 + pyqtgraph GUI). |
| `run_tuner_ble.bat` | Windows launcher: double-click it (installs what it needs the first time). |

Run it on the PC that is connected to the board (USB serial, or Bluetooth for the BLE versions).
Close PuTTY first: only one program can hold the COM port.
