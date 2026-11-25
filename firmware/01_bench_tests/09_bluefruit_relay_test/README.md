# Bluefruit + relay test - phone START/STOP switches the motor-power relay

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-23 |
| **Status** | WORKED: the relay clicks on START and off on STOP from the phone. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
Bluefruit SPI Friend
  VIN pin 2 (+5 V)   GND pin 6   SCK pin 23 (RD1)   MOSI pin 37 (RD10)   MISO pin 7 (RA14)
  CS pin 13 (RJ13)   IRQ and RST not connected
IMU CS pin 16 (RK1) is held HIGH so the IMU stays off the bus
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
PuTTY . UART2 over the J11 USB port, 9600 baud 8N1 (this test runs on the 8 MHz FRC clock)
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. In MPLAB X: **File > New Project > Standalone Project**, device **PIC32MZ2048EFM144**, compiler **XC32**.
2. Open `bluefruit_relay_test.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. This file has its own `#pragma config` lines: **do not** add config.bits.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `bluefruit_relay_test.c` | Bluefruit + relay test - phone START/STOP switches the motor-power relay |
