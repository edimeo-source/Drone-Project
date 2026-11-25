# ESC throttle calibration - all 4 ESCs at once

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-18 |
| **Status** | Calibration program written Nov 18 (build it with Make and Program Device, not Debug). No result is logged in the project notes. Since the relay went in (Nov 19) it can no longer power the ESCs: use 13_esc_calibrate_relay. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. Download the repo (Code > Download ZIP) and copy the `EscCalibrate.X` folder somewhere (e.g. `MPLABXProjects`).
2. In MPLAB X: **File > Open Project** and pick `EscCalibrate.X`. It is already set up for the PIC32MZ2048EFM144 and XC32.
3. Build and program.

## Files

| File | What it is |
|---|---|
| `EscCalibrate.X/Makefile` | MPLAB X project makefile. |
| `EscCalibrate.X/config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `EscCalibrate.X/main.c` | ESC throttle calibration - all 4 ESCs at once |
| `EscCalibrate.X/nbproject/configurations.xml` |  |
| `EscCalibrate.X/nbproject/project.xml` |  |
