# IMU wiring test - LSM6DSO over SPI with CS on pin 16

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-15 |
| **Status** | WORKED: WHO_AM_I reads 0x6C on the real board. Use this first whenever the IMU wiring changes. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. In MPLAB X: **File > New Project > Standalone Project**, device **PIC32MZ2048EFM144**, compiler **XC32**.
2. Open `imu_wiring_test.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `imu_test.hex` | Ready-built program: MPLAB X IPE can program this straight onto the board. |
| `imu_wiring_test.c` | IMU wiring test - LSM6DSO over SPI with CS on pin 16 |
