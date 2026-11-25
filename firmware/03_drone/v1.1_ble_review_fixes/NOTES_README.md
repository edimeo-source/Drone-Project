# ELC275 PIC32 + Bluefruit BLE integration (revision 1.1, Nov 20 2026)

`Drone.X` is the current flight firmware (pitch PID, 4 ESCs, PID tuner over USB) with:
- Bluetooth LE added as a second link to the PID tuner;
- a relay that cuts ESC power;
- ARM/KILL and a heartbeat failsafe.

`pid_tuner_ble.py` is the PID tuner with a Bluetooth LE backend added; serial (USB) still works. `run_tuner_ble.bat` installs the Python packages and starts it.

Revision 1.0 came from ChatGPT. Revision 1.1 fixes the problems found in review (list at the end).

**Status:**
- The firmware compiles and links with `gcc -Wall -Wextra` against stand-in PIC32 headers.
- Host tests in `host_tests/` pass.
- **It has not been built with XC32 or run on the board.** Build it in MPLAB X (XC32 6.00), then follow the bench test below.

## Hardware

- PIC32MZ2048EFM144 Starter Kit. SYSCLK 120 MHz, PBCLK 60 MHz (`config.bits`, unchanged).
- **IMU (LSM6DSO):** SPI1 mode 3, 2 MHz, CS = RK1 (J12 pin 16).
- **Bluefruit LE SPI Friend:** SPI1 mode 0, 1 MHz, CS = RJ13 (J12 pin 13).
  - Shares the IMU's SPI lines: SCK = RD1 (pin 23), MOSI = RD10 (pin 37), MISO = RA14 (pin 7).
  - Powered from J12 pin 1 (3.3 V) and pin 6 (GND) through the sensor hub. IRQ and RST are not connected.
- **Relay:** CH1 = RK4 (J12 pin 33), active-HIGH. HIGH = ESC power on. LOW, reset, or PIC unpowered = ESC power off. Relay board ground is pin 34.
- **ESC signals:** pins 11, 12, 21 and 19 (OC1–OC4, 50 Hz, 1000–2000 µs).
- **J12 pin 2** is the 5 V **input** from the 7805. Pins 8 and 10 are UART2 to the PC: never wire them.
- **Stops:** SW1 is a software kill (the PIC reads it). Pulling the battery XT60 is the only stop that works without the PIC.

## How it behaves

1. **Startup.** RK4 is driven LOW before anything else, and all four ESC outputs start at 1000 µs.
2. **ARM.** The tuner's ARM button sends `ARM` over the link you are connected with. The PIC accepts it only if all of these hold:
   - the IMU is calibrated with no error;
   - SW1 is up;
   - pitch and roll are within 45°;
   - the motors are off;
   - over Bluetooth, the link is fresh.

   If accepted, the PIC writes 1000 µs, closes the relay, and replies `ARMED,USB: ...` or `ARMED,BLE: ...`. A refusal is answered with `DISARMED,ARM refused - ...`.
3. **Starting.** Starts (`G`, `M`, `R`) are refused for 2 s after ARM, so the ESCs can finish their startup beeps. After that:
   - `G` and `M` work over either link;
   - `R` (step test) is USB only.

   `M` is accepted again while already running, because the tuner resends it every time a manual slider moves.
4. **Heartbeat.** While armed, the tuner sends `H` every 100 ms on the same link. The 4 ms interrupt only compares timestamps. If the last `H` is older than 350 ms, it sets 1000 µs and opens the relay. For a Bluetooth session, a stale link check (`AT+GAPGETCONN`, 750 ms) also stops the motors.
5. **Any stop** latches the motors off and opens the relay: `X`, SW1, IMU error, tip limit, phone `STOP`, heartbeat or link loss, calibrate, or the noise test.
   - The PIC then sends `DISARMED,<reason>` on USB and, if connected, on Bluetooth. The tuner stops its heartbeat and shows the reason.
   - Reconnecting never re-arms.
6. **Live data** goes back to the link that sent `L,1`: 125 lines/s on USB, 5 lines/s on Bluetooth. A connected Bluetooth device no longer takes the live graph away from a USB session.
7. **Step tests** record every 4 ms sample with nothing transmitted while recording (the professor's rule).
   - The recording is then sent over USB **one line per 4 ms loop**, so `main()` keeps reading commands.
   - `H` and `X` still work during the roughly 20 s it takes to send a 20 s recording.
8. **Bluetooth sending:**
   - Replies and status lines wait in an 8-line queue.
   - Live data is only queued when nothing else is waiting.
   - Before sending, the PIC asks the module how much room its transmit buffer has (`AT+BLEUARTFIFO=TX`). Adafruit's docs say `AT+BLEUARTTX` blocks 200 ms and then answers `ERROR` when that buffer is full. With no room, live lines are dropped and status lines wait.
   - A refused send is not treated as a dead module.
   - On module firmware older than 0.6.7 (no FIFO command), it sends without asking.
9. **Phone** (Bluefruit Connect, UART screen): `STOP`, `KILL`, `X` and `STATUS` work. A phone can't send the heartbeat, so an `ARM` from a phone is cut off after 350 ms.

## PC app

- Run `run_tuner_ble.bat`. It installs PySide6, pyqtgraph, numpy, pyserial and bleak on first run.
- Select **Bluetooth LE → Bluefruit**.
  - Leave the name blank to find the module by its UART service, when only one is nearby.
  - Otherwise enter its name or address. The module keeps the name "Adafruit Bluefruit LE" unless renamed with `AT+GAPDEVNAME=PIC32-DRONE` and then `ATZ`.
- Nordic UART Service: service `6E400001-...`, write `6E400002-...`, notify `6E400003-...`.
- ARM asks for confirmation. KILL (or Esc) sends `X`. Heartbeats run while armed and stop on `DISARMED`.
- Serial (USB) mode works as before, plus ARM and the heartbeat. A missing or unconnected Bluefruit does not affect a USB session.

## Build in MPLAB X

Open the `Drone.X` folder as a project. `bluefruit.c/.h` are already in `nbproject/configurations.xml`. Keep `config.bits` excluded from the build (`main.c` includes it). Don't add a second set of configuration pragmas.

## Bench test (props removed)

1. **Battery/ESC power disconnected.** Program the board, then check:
   - the PIC boots and the IMU calibrates;
   - all four PWM outputs are 1000 µs;
   - RK4 is LOW.
2. **RK4 goes LOW after each of these:** reset, `X`, SW1, tilting past 45° while armed, ending the tuner process (Task Manager) while armed, so the heartbeat stops, and calibrate. Each one should print `DISARMED,<reason>` in the tuner console.
3. **USB session.** ARM → relay clicks, pulses stay 1000 µs. A start inside 2 s is refused. After 2 s, Run manual at a low pulse and move the sliders: the pulses follow, with no Bluetooth connected.
4. **Step test.** Run a short one over USB. Press KILL while the data is still arriving: the motors stop and the relay opens right away.
5. **Bluetooth.** Repeat steps 3–4 over Bluetooth (no step test). Then walk out of range or turn off the laptop's Bluetooth: the relay opens within about 0.35–0.75 s.
6. **Loop timing.** With Bluetooth connected and live data on, scope LED3 (RH2). It should still pulse every 4 ms.
7. **Before any powered propeller test:**
   - a DC-rated relay for the full ESC current is installed. Hover is about $4 \times 4.4\ \text{A} = 17.6\ \text{A}$; the bench relay is rated 10 A.
   - the XT60 pull has been practiced.

## Limitations

- Bluetooth throughput has not been measured on this board. Measure it before raising the 5 Hz live rate.
- A Bluetooth STOP is best-effort and depends on range. The heartbeat is the backup. The XT60 is the only stop that works without the PIC.
- The firmware can't detect welded relay contacts.
- Only pitch is controlled (tethered first stage).

## Changes in revision 1.1

| # | File | Problem in 1.0 | Fix |
|---|---|---|---|
| 1 | `tuner.c` | Did not compile: `ble_connected` and `bluetooth_send_text()` used without `#include "bluefruit.h"` | Include added |
| 2 | `motors.c` | `set_motor()` forced 1000 µs unless a Bluetooth central was connected, so a USB session could never spin the motors | Gate is now `motor_mode==0 \|\| kill_latched \|\| !control_ready`. Link loss is handled per link by the 4 ms interrupt. |
| 3 | `control.c` | `control_can_start()` refused starts while the motors ran, so moving the manual sliders (which resends `M`) was blocked | `motor_mode!=0` check removed. All safety checks remain. |
| 4 | `tuner.c` | The step-test dump blocked `main()` for about 20 s with the motors still running. `H` wasn't read, so every USB step test tripped the heartbeat and the relay opened under load. | Dump sends one line per 4 ms loop. Commands are read between lines. |
| 5 | `uart.c` | Receive buffer overwrote unread characters when full | New characters are dropped when it is full. Old ones are kept. |
| 6 | `main.c`, `pid_tuner_ble.py` | Stops caused by the PIC itself (tip, SW1, IMU, heartbeat) were only printed on USB. The app kept showing "armed". | `DISARMED,<reason>` goes to every link. The app stops its heartbeat and shows the reason. ARM replies start with `ARMED`/`DISARMED`. |
| 7 | `main.c`, `tuner.c` | Live data went to Bluetooth whenever a central was connected, blanking a USB live graph | Live data goes to the link that sent `L,1` |
| 8 | `bluefruit.c` | One outgoing slot that silently dropped replies (for example the `A,` gain acknowledgement). A full module TX buffer caused a 200 ms block and then `ERROR`, which was treated as a dead module (Bluetooth session killed). | 8-line queue, a free-space check before sending, live data dropped when there is no room, and `ERROR` replies are not fatal |
| 9 | `bluefruit.c` | `putU2()` calls `bluetooth_service()`. A Bluetooth `C` (calibrate) prints, so the service could run inside itself and overwrite the reply it was still reading. | Re-entry guard |
| 10 | `main.c`, `bluefruit.c` | Help and notices described the old phone `ARM;` scheme | Text updated |
