# ESC throttle calibration from PuTTY, through the motor-power relay - all 4 ESCs at once, no kit buttons

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-25 |
| **Status** | New Nov 25: 07_esc_calibrate redone for the assembled drone: PuTTY commands instead of SW1/SW2, and the relay is closed by the program (2.0 ms first, then RK4). Checked in a PC simulation (26 checks). Not yet run on the board. Props OFF. |
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
2. Open `esc_calibrate_relay.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `esc_calibrate_relay.c` | ESC throttle calibration from PuTTY, through the motor-power relay - all 4 ESCs at once, no kit buttons |

## Why this one and not 07

`07_esc_calibrate` expects the ESCs to get power the moment you plug the battery in, and it is stepped with the kit's SW2 button. Since Nov 19 the battery reaches the ESCs only through the relay (J12 pin 33 = RK4, HIGH = on), and 07 never turns RK4 on, so the ESCs never power up. This version closes the relay itself at the right moment, and runs **entirely from PuTTY**, so it works with the drone fully assembled and the kit buttons out of reach.

## What calibration fixes (and what it doesn't)

Each ESC has its own idea of which pulse means "stop" and which means "full". Calibration teaches all 4 the same range, 1.0 ms = stop and 2.0 ms = full, so the same pulse asks every ESC for the same fraction of throttle. The ESC saves the range, so this is done once.

It does **not** make the motors push the same: motors, props and ESCs still differ a little. That is what `11_motor_calibration_test` measures and fixes with trims. Run this one first, then 11.

## Using it (PuTTY: the kit's USB COM port, 115200 baud, 8N1)

**Setup**

- **Props off.** If an ESC misses the calibration window, 2.0 ms is full throttle.
- Battery plugged in: the 7805 (tapped before the relay, J12 pin 2) powers the kit, and the relay keeps the ESCs off until the right moment. USB from the PC to the kit for PuTTY.

**Steps**

1. Type `CAL`, then `Y` to confirm the props are off.
2. The board sends 2.0 ms to all 4, waits 0.5 s, then closes the relay. The ESCs play their power-up tune, then 2 short beeps.
3. **Press Enter right after the 2 beeps** (within 2 s). Enter in the first second after the relay closes is ignored (too early to be the beeps).
4. The board sends 1.0 ms. Every ESC beeps (3 for a 3S battery), then 1 long beep = range saved. A long tune instead means programming mode: type `OFF`, then `CAL` again.
5. Type `T` for a test spin: all 4 ramp to 1100 us together, spin 3 s, stop. They should start together. `T 1150` uses another pulse (1050-1200 us).

**Stopping without the kit buttons**

- `Esc`, `Space` or `X` while a step is waiting or a test spin is running = all 4 at 1000 us **and the relay off**. Any other key also stops a test spin (relay stays on).
- If nothing is typed for 20 s after the relay closes, it aborts by itself, so 2.0 ms is never held for long.
- `S` or Enter at the prompt = stop all; `OFF` / `ON` = relay off / on. SW1 still works as a backup.

## Checked

PC simulation, 26 checks: nothing is sent before `Y`; step 2 sends 1875 ticks (2.0 ms) with the relay still off, then closes it; an early Enter and a leftover line feed are ignored; Enter after the beeps sends 938 ticks (1.0 ms); `CAL` with the relay already on opens it and waits 3 s first; Esc, Space, the 20 s timeout and SW1 each give 1000 us + relay off; `T` and `T 1150` ramp to 1031 / 1078 ticks, `T 1300` is refused; `T` with the relay off arms for 4 s first. Not yet run on the board.
