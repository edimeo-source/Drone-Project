# IMU roll / pitch / yaw over SPI (complementary filter)

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-12 |
| **Status** | Builds. Same SPI wiring problem as 02 at the time (CS on pin 3). |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
IMU LSM6DSO (SPI1, CS on pin 3 in this era)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 3 (RA3)
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. In MPLAB X: **File > New Project > Standalone Project**, device **PIC32MZ2048EFM144**, compiler **XC32**.
2. Open `imu_roll_pitch_yaw_spi.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `imu_roll_pitch_yaw_spi.c` | IMU roll / pitch / yaw over SPI (complementary filter) |
