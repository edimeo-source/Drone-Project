# ESC full-speed test (2 motors) + ESC calibration mode

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-12 |
| **Status** | WORKED on the seesaw: both motors armed and ran to full speed. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. In MPLAB X: **File > New Project > Standalone Project**, device **PIC32MZ2048EFM144**, compiler **XC32**.
2. Open `esc_full_speed.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `esc_full_speed.c` | ESC full-speed test (2 motors) + ESC calibration mode |
