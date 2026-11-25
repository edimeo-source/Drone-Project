# SewSaw.X - seesaw firmware, steps 0-1 (IMU on SPI, filter, PC tuner link)

| | |
|---|---|
| **Version** | SewSaw v1 |
| **Date** | 2026-11-16 |
| **Status** | WORKED: step 0 (IMU check) passed and step 1 ran on the board Nov 16. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC pins 11/12 are reserved for the seesaw motors but not driven yet (motors.c arrives in step 2-3)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. Download the repo (Code > Download ZIP) and copy the `SewSaw.X` folder somewhere (e.g. `MPLABXProjects`).
2. In MPLAB X: **File > Open Project** and pick `SewSaw.X`. It is already set up for the PIC32MZ2048EFM144 and XC32.
3. Build and program.

## Files

| File | What it is |
|---|---|
| `SewSaw.X/Makefile` | MPLAB X project makefile. |
| `SewSaw.X/config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `SewSaw.X/control.c` | The 4 ms control loop (Timer4 interrupt): IMU -> filter -> PID -> motors, plus arming and every safety check. |
| `SewSaw.X/control.h` | Shared flight state (armed, setpoint, gains, motor pulses) and the control-loop functions. |
| `SewSaw.X/delay.c` | Timer3 millisecond delays. |
| `SewSaw.X/delay.h` | Delay functions. |
| `SewSaw.X/imu.c` | LSM6DSO IMU driver over SPI1 and the complementary filter (roll, pitch, yaw). |
| `SewSaw.X/imu.h` | IMU angles/rates and the IMU functions. |
| `SewSaw.X/main.c` | Program entry: start-up, then the main loop (PuTTY keys, tuner/phone commands, status). |
| `SewSaw.X/noise.c` | 60 s IMU noise test (RMS noise, jitter, drift) with PASS/FAIL. |
| `SewSaw.X/noise.h` | Noise-test functions and flag. |
| `SewSaw.X/pid.c` | PID math (one step, with anti-windup). |
| `SewSaw.X/pid.h` | PID function prototype and the loop time. |
| `SewSaw.X/tuner.c` | PID-tuner protocol (PC or phone): gains, setpoint, recorded step tests, live data. |
| `SewSaw.X/tuner.h` | Tuner functions and the shared step-test state. |
| `SewSaw.X/uart.c` | UART2 link to the PC (PuTTY and the PC tuner) with an interrupt-driven receive buffer. |
| `SewSaw.X/uart.h` | UART2 functions. |
| `SewSaw.X/nbproject/Makefile-default.mk` |  |
| `SewSaw.X/nbproject/Makefile-genesis.properties` |  |
| `SewSaw.X/nbproject/Makefile-impl.mk` |  |
| `SewSaw.X/nbproject/Makefile-local-default.mk` |  |
| `SewSaw.X/nbproject/Makefile-variables.mk` |  |
| `SewSaw.X/nbproject/configurations.xml` |  |
| `SewSaw.X/nbproject/project.xml` |  |
