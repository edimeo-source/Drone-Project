# ELC275 PIC32 + Bluefruit BLE integration (merged working tree)

## What this package is

`Drone.X` is based on the supplied `A_current_firmware_Drone.X` project and merges the Bluetooth/SDEP work from folder B. It keeps the existing IMU SPI1 driver, Timer4 250 Hz PID loop, ESC PWM, UART2/USB tuner path, and command protocol. It adds the Bluefruit BLE UART transport, BLE command routing, relay output, explicit ARM/KILL handling, and heartbeat failsafe.

The PC app is `pid_tuner_ble.py`, based on the supplied PID tuner with BLE support. `run_tuner_ble.bat` installs dependencies and launches it.

## Hardware assumptions taken from the supplied Nov 20 handoff

- PIC32MZ2048EFM144; XC32 6.00; SYSCLK 120 MHz; PBCLK 60 MHz.
- IMU: SPI1 mode 3, BRG 14, CS RK1/J12-16.
- Bluefruit: SPI1 mode 0, BRG 29 (1 MHz), CS RJ13/J12-13. Shared SCK RD1/J12-23, MOSI RD10/J12-37, MISO RA14/J12-7. IRQ/RST not connected.
- Bluefruit power in the current harness: J12-1 3.3 V and J12-6 GND. J12-2 is the 5 V input from the 7805 and is NOT the Bluefruit VIN in this harness.
- Relay: active-HIGH on RK4/J12-33. HIGH closes relay; LOW opens it. Relay board ground J12-34.
- SW1 RB12 remains the local software kill. Battery XT60 is the physical disconnect.

## How the firmware behaves

1. Relay output is initialized LOW before any motor operation.
2. ESC PWM is initialized to 1000 us on all four outputs.
3. IMU startup and calibration must succeed before ARM is accepted.
4. `ARM` from BLE or USB closes the relay only after minimum ESC pulses are written. Motor start commands remain blocked for 2 seconds to allow ESC startup.
5. `H` heartbeat is sent by the PC app every 100 ms. The 4 ms control ISR checks only a timestamp (no SPI, no BLE calls). If heartbeat age exceeds 350 ms, it commands 1000 us and opens RK4. BLE additionally requires a fresh `AT+GAPGETCONN` check.
6. `STOP`, `KILL`, `X`, SW1, IMU failure, tip limit, and link timeout all latch motors off and open the relay. Reconnection never automatically re-arms.
7. After ARM, use the existing PID tuner controls to issue `G`, `M`, or `R` (step test only on USB). BLE `R` is intentionally rejected; full-rate run recording remains on UART2/USB. BLE live telemetry starts at 5 Hz.

## PC app

- `run_tuner_ble.bat` installs PySide6, pyqtgraph, numpy, pyserial and bleak.
- Select `Bluetooth LE → Bluefruit`. Leave the BLE name blank for service-based auto-discovery when one Nordic UART device is nearby, or enter the advertised name/address.
- The app uses Nordic UART Service: service `6E400001-...`, write `6E400002-...`, notify `6E400003-...`.
- ARM asks for explicit confirmation. KILL sends `X`. Heartbeats are sent while armed. If the PC disconnects, the PIC heartbeat timeout opens the relay.
- USB/Serial mode remains available and also supports ARM/H heartbeat in this merged firmware. A missing/faulted Bluefruit does not kill a separately armed USB session.

## Build in MPLAB X

Open `Drone.X/nbproject/project.xml` (or open the `Drone.X` folder as an MPLAB X project). The project metadata includes `bluefruit.c/.h`. The original `config.bits` is retained. Do not add a second set of configuration pragmas or change the existing PPS assignments.

The code has been source-reviewed and the Python file syntax-checked in this environment, but this environment does not contain Microchip XC32 or the PIC32MZ DFP build tools. Therefore **the C project has not been compiled here**. Build with the project's XC32 6.00 toolchain and resolve any device-pack/compiler-specific diagnostics before loading it.

## Required bench test sequence — props removed, ESC power isolated initially

1. Build and program with battery/ESC power disconnected. Verify PIC32 boots, IMU calibrates, and all four PWM outputs are 1000 us.
2. Verify RK4 is LOW after reset, startup, calibration, SW1, `X`, IMU error, and link loss.
3. With props removed and the correct DC-rated relay installed, connect the ESC bus. Verify ARM closes relay while PWM remains 1000 us. Wait at least 2 seconds; verify motor-start command is rejected before then.
4. Verify KILL immediately sets all four PWM outputs to 1000 us and opens the relay.
5. Verify pulling the PC out of BLE range / disconnecting it causes the 350 ms heartbeat failsafe to open the relay. Repeat over USB.
6. Only after those tests should PID telemetry be checked. Do not perform powered propeller tests until the control system and failsafe are independently validated.

## Relay current warning

The previous 10 A relay rating is not established as adequate for the quad's measured/current motor draw. Select a relay/contactor with a DC voltage rating and continuous, make/break, and inrush ratings suitable for the total ESC current. The diagram's estimated hover current already exceeds 10 A. This firmware cannot compensate for undersized contacts or welded contacts.

## Known limitations

- No hardware IRQ or reset wire is connected to Bluefruit; the firmware polls SDEP and uses the SDEP initialize command.
- BLE throughput is not assumed to support full-rate step recordings. Live telemetry starts at 5 Hz; measure actual throughput before increasing it.
- Emergency stop over BLE is not instantaneous or guaranteed at arbitrary range. The heartbeat failsafe is a backup; SW1 is a software stop and XT60 remains the physical disconnect.
- The relay contact path itself must be tested with the chosen relay and load. Firmware cannot detect welded contacts unless additional feedback hardware is added.
