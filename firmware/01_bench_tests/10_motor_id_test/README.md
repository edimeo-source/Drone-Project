# Motor ID test - spin the motor you name from PuTTY (RR 1150) to check wiring and direction

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-25 |
| **Status** | New Nov 25. Checked in a PC simulation (17 checks: each name drives the right pin and tick count, slow ramp, relay arming, SW1, bad input). Not yet run on the board. |
| **Board** | PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK, 60 MHz PBCLK) |

## J12 pinout for this version

```text
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

Every `.c` and `.h` file starts with the same pinout in a comment, and every function has a title block above it.

## How to use it

1. In MPLAB X: **File > New Project > Standalone Project**, device **PIC32MZ2048EFM144**, compiler **XC32**.
2. Open `motor_id_test.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `motor_id_test.c` | Motor ID test - spin the motor you name from PuTTY (RR 1150) to check wiring and direction |

## Using it (PuTTY: the kit's USB COM port, 115200 baud, 8N1)

**Props off.** Type a command and press Enter:

| Command | What happens |
|---|---|
| `RR 1150` | Rear-right motor ramps to 1150 us. The other 3 stop. |
| `FL 1100` / `FR ...` / `RL ...` | Same for the other motors (`BL` / `BR` also work for the back two). |
| `ALL 1120` | All 4 motors at the same pulse. |
| `S`, `X`, `STOP`, or just Enter | Every motor to 1000 us (stop). The relay stays on. |
| `OFF` / `ON` | Relay off (motor power cut) / relay on. |
| `?` | Help. |
| **SW1** on the kit | Stop all and relay off, any time. |

- Pulses from 1000 to 1500 us are accepted.
- The motor ramps up at 200 us per second. Going down is immediate.
- The first motor command closes the relay. The ESCs then get 4 s of 1000 us pulses to arm (listen for the beeps) before anything spins.
- Every reply says which ESC and J12 pin it drove, and which way the motor should turn.

Seen from above, the motors should turn this way:

| Motor | Direction | Prop nut |
|---|---|---|
| FL | CCW | silver |
| FR | CW | black |
| RL | CW | black |
| RR | CCW | silver |

If something is wrong:

- **The wrong motor moves:** that ESC's signal wire is on the wrong J12 pin.
- **The right motor turns the wrong way:** swap any 2 of its 3 motor wires at the ESC.
