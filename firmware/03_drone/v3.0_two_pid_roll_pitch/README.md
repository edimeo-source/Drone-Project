# Drone.X v3.0 - roll PID + pitch PID, 4 different ESC pulses, one axis at a time  (CURRENT)

| | |
|---|---|
| **Version** | v3.0 |
| **Date** | 2026-11-25 |
| **Status** | CURRENT. Built from v2.0 (which builds and flew on the tether). Host tests pass (78 checks, including a simulated flight with the real 4 ms loop). Not yet built in MPLAB or flown. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
  VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g. 9)
  SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. Download the repo (Code > Download ZIP) and copy the `Drone.X` folder somewhere (e.g. `MPLABXProjects`).
2. In MPLAB X: **File > Open Project** and pick `Drone.X`. It is already set up for the PIC32MZ2048EFM144 and XC32.
3. Build and program.

**Host tests** (PC only, needs gcc): `sh host_tests/run_tests.sh` from this folder. They test the logic, not the real hardware.

## Files

| File | What it is |
|---|---|
| `HOW_TO_RUN.md` | Notes that came with this version. |
| `Drone.X/BLE_INTEGRATION_VERSION.txt` | One-line version note. |
| `Drone.X/Makefile` | MPLAB X project makefile. |
| `Drone.X/bluefruit.c` | Adafruit Bluefruit LE SPI Friend driver (SDEP over SPI1, shared with the IMU): the phone link. |
| `Drone.X/bluefruit.h` | Bluetooth functions and shared link state. |
| `Drone.X/config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `Drone.X/control.c` | The 4 ms control loop (Timer4 interrupt): IMU -> filter -> roll PID + pitch PID -> 4-motor mixer, plus arming, soft start and every safety check. |
| `Drone.X/control.h` | Shared flight state (axis, throttle, roll and pitch gains and targets, RC, PC, the 4 pulses) and the control-loop functions. |
| `Drone.X/delay.c` | Timer3 millisecond delays. |
| `Drone.X/delay.h` | Delay functions. |
| `Drone.X/imu.c` | LSM6DSO IMU driver over SPI1 and the complementary filter (roll, pitch, yaw). |
| `Drone.X/imu.h` | IMU angles/rates and the IMU functions. |
| `Drone.X/main.c` | Program entry: start-up, then the main loop (PuTTY keys, tuner/phone commands, status). |
| `Drone.X/motors.c` | ESC pulse outputs: Timer2 at 50 Hz with OC1-OC4, the motor-power relay pin, and the per-motor trims. |
| `Drone.X/motors.h` | Pulse limits (1000/2000 us), the per-motor trim #defines and the motor functions. |
| `Drone.X/noise.c` | 60 s IMU noise test (RMS noise, jitter, drift) with PASS/FAIL. |
| `Drone.X/noise.h` | Noise-test functions and flag. |
| `Drone.X/pid.c` | PID math (one step, with anti-windup). |
| `Drone.X/pid.h` | PID function prototype and the loop time. |
| `Drone.X/tuner.c` | PID-tuner protocol (PC or phone): axis, roll and pitch gains and targets, throttle, soft start, recorded step tests, live data. |
| `Drone.X/tuner.h` | Tuner functions and the shared step-test state. |
| `Drone.X/uart.c` | UART2 link to the PC (PuTTY and the PC tuner) with an interrupt-driven receive buffer. |
| `Drone.X/uart.h` | UART2 functions. |
| `host_tests/fly_test.c` | Host test: flies a simulated drone (physics model in place of the IMU) with the real 4 ms loop: levels each axis, holds the demo angles, a wrong sign ends at the tip cutoff. |
| `host_tests/fw_test.c` | Host test: arming, relay, heartbeat, motor output and PC data (runs on a PC with gcc). |
| `host_tests/mix_test.c` | Host test: the two PIDs, the 4-motor mixer signs, one axis at a time, every v3.0 command, and the step-test dumps. |
| `host_tests/run_tests.sh` | Runs every host test with gcc (Linux, macOS or WSL). |
| `host_tests/sim_ble.c` | Host test: the Bluetooth driver against a simulated Bluefruit module. |
| `host_tests/stubs_ble.c` | Host test: empty Bluetooth functions for the tests that do not use Bluetooth. |
| `host_tests/trim_test.c` | Host test: the per-motor trims (built with test trim values). |
| `host_tests/v12_test.c` | Host test: the 60 deg tip cutoff and the end of a recorded run. |
| `Drone.X/nbproject/configurations.xml` |  |
| `Drone.X/nbproject/project.xml` |  |
| `host_tests/stub/defs.c` | Host test support: the stand-in PIC32 registers (plain variables). |
| `host_tests/stub/p32xxxx.h` | Host test support: stand-in PIC32 register declarations. |
| `host_tests/stub/xc.h` | Host test support: stand-in for <xc.h> so the firmware compiles with gcc. |
| `host_tests/stub/sys/attribs.h` | Host test support: stand-in for <sys/attribs.h> (__ISR becomes nothing). |
