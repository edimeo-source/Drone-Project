# PC PID tuner v1 (serial)  (2026-11-11)

Pairs with: **00 flight-controller draft, SewSaw step 0-1**

| File | What it is |
|---|---|
| `pid_tuner.py` | The tuner program (Python 3 + Tkinter/matplotlib GUI). |
| `run_tuner.bat` | Windows launcher: double-click it (installs what it needs the first time). |
| `screenshots/` | What the GUI looks like. |
| `tests/` | PC-side tests of the tuner (simulated board, serial link, UI, speed). |

Run it on the PC that is connected to the board (USB serial, or Bluetooth for the BLE versions).
Close PuTTY first: only one program can hold the COM port.
