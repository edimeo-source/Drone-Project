# Drone.X v0.2 - partner's Bluetooth STOP draft (adds bluefruit.c)

| | |
|---|---|
| **Version** | v0.2 draft |
| **Date** | 2026-11-20 |
| **Status** | Draft source only (no MPLAB project files). Merged into v1.0. |
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
Bluefruit SPI Friend (shares SPI1 with the IMU)
  VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
  CS pin 13 (RJ13)   IRQ and RST not connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

This folder is source only (no MPLAB project files). Make a new MPLAB X project for the PIC32MZ2048EFM144 with XC32, add every `.c` as a Source File and every `.h` as a Header File, and put `config.bits` in the project folder without adding it as a source.

## Files

| File | What it is |
|---|---|
| `Drone.X/bluefruit.c` | Adafruit Bluefruit LE SPI Friend driver (SDEP over SPI1, shared with the IMU): the phone link. |
| `Drone.X/bluefruit.h` | Bluetooth functions and shared link state. |
| `Drone.X/config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `Drone.X/control.c` | The 4 ms control loop (Timer4 interrupt): IMU -> filter -> PID -> motors, plus arming and every safety check. |
| `Drone.X/control.h` | Shared flight state (armed, setpoint, gains, motor pulses) and the control-loop functions. |
| `Drone.X/delay.c` | Timer3 millisecond delays. |
| `Drone.X/delay.h` | Delay functions. |
| `Drone.X/imu.c` | LSM6DSO IMU driver over SPI1 and the complementary filter (roll, pitch, yaw). |
| `Drone.X/imu.h` | IMU angles/rates and the IMU functions. |
| `Drone.X/main.c` | Program entry: start-up, then the main loop (PuTTY keys, tuner/phone commands, status). |
| `Drone.X/motors.c` | ESC pulse outputs: Timer2 at 50 Hz with OC1-OC4 (and the motor-power relay pin in v1.0+). |
| `Drone.X/motors.h` | Pulse limits (1000/2000 us) and the motor functions. |
| `Drone.X/noise.c` | 60 s IMU noise test (RMS noise, jitter, drift) with PASS/FAIL. |
| `Drone.X/noise.h` | Noise-test functions and flag. |
| `Drone.X/pid.c` | PID math (one step, with anti-windup). |
| `Drone.X/pid.h` | PID function prototype and the loop time. |
| `Drone.X/tuner.c` | PID-tuner protocol (PC or phone): gains, setpoint, recorded step tests, live data. |
| `Drone.X/tuner.h` | Tuner functions and the shared step-test state. |
| `Drone.X/uart.c` | UART2 link to the PC (PuTTY and the PC tuner) with an interrupt-driven receive buffer. |
| `Drone.X/uart.h` | UART2 functions. |
