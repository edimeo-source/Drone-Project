# Liftoff throttle test - full-range (to 2000 us) motor control from PuTTY with +/- steps, to find the lowest pulse that lifts the drone

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-25 |
| **Status** | New Nov 25. Checked in a PC simulation (16 checks: ALL/one motor up to 2000 us, +/- steps, 2000 us cap, instant stop on Esc/Space/X, SW1, relay). Not yet run on the board. Props ON: the drone must be tied down. |
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
2. Open `liftoff_throttle_test.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `liftoff_throttle_test.c` | Liftoff throttle test - full-range (to 2000 us) motor control from PuTTY with +/- steps, to find the lowest pulse that lifts the drone |

## Using it (PuTTY: the kit's USB COM port, 115200 baud, 8N1)

**Props on. Tie the drone down and stand clear.** This test has no leveling: all 4 motors get the same pulse, so the drone tips toward its weakest motor as it lifts. Run `11_motor_calibration_test` first to see how far apart the motors are.

| Command | What happens |
|---|---|
| `ALL 1300` | All 4 ramp to 1300 us (200 us per second). |
| `+` / `-` | The running motors go up / down 10 us. |
| `+ 25` / `- 50` | Any step size. |
| `RR 1300` | Only that motor (FL, FR, RL, RR, BL, BR). |
| `S` or Enter | Stop all. The relay stays on. |
| `OFF` / `ON` | Relay off / on. |
| **Esc**, or **Space / X as the first key** | **Instant stop, no Enter needed.** |
| **SW1** | Stop all and relay off. |

- Pulses go from 1000 to 2000 us (full throttle).
- Each change prints the new pulse, and prints `... at N us now` when the ramp gets there.

**Finding the minimum flying pulse**

1. Type `ALL 1300`.
2. Press `+` and Enter, one step at a time.
3. Stop at the first pulse where the tether goes slack and the drone lifts. That is your minimum.
4. Press Esc.

Write the pulse down with the battery voltage. A sagging battery needs a higher pulse.

**Relay current.** Near liftoff the 4 motors pull about $4\times 4.4\ \text{A}\approx 18\ \text{A}$, and full throttle about 50 A. A 10 A relay module is overloaded, so keep runs short or use the 30 A relay. A stop still works even if the relay contacts stick, because the stop sets the ESCs back to 1000 us.
