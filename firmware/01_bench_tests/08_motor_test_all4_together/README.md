# Motor test - all 4 ESCs, same pulse, same time

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-18 |
| **Status** | Written Nov 18 to run after calibration (ramp to 1.25 ms, hard cap 2.0 ms). No result is logged in the project notes. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. In MPLAB X: **File > New Project > Standalone Project**, device **PIC32MZ2048EFM144**, compiler **XC32**.
2. Open `motor_test_all4_together.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `motor_test_all4_together.c` | Motor test - all 4 ESCs, same pulse, same time |
