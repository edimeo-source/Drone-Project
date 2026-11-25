# Motor spin-direction test - all 4 ESCs, one motor at a time

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-17 |
| **Status** | WORKED: spin directions verified Nov 18, props off (FL CCW, FR CW, RL CW, RR CCW). Two ESCs did not start at 1.09 ms, which led to 07_esc_calibrate. |
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

1. Download the repo (Code > Download ZIP) and copy the `MotorSpinTest.X` folder somewhere (e.g. `MPLABXProjects`).
2. In MPLAB X: **File > Open Project** and pick `MotorSpinTest.X`. It is already set up for the PIC32MZ2048EFM144 and XC32.
3. Build and program.

## Files

| File | What it is |
|---|---|
| `MotorSpinTest.X/Makefile` | MPLAB X project makefile. |
| `MotorSpinTest.X/config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `MotorSpinTest.X/main.c` | Motor spin-direction test - all 4 ESCs, one motor at a time |
| `MotorSpinTest.X/nbproject/configurations.xml` |  |
| `MotorSpinTest.X/nbproject/project.xml` |  |
