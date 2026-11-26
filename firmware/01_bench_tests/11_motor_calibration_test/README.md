# Motor calibration test - kitchen-scale thrust sweep that prints per-motor pulse trims

| | |
|---|---|
| **Version** | bench test |
| **Date** | 2026-11-25 |
| **Status** | New Nov 25. Checked in a PC simulation with 4 deliberately mismatched motors: the printed trims bring them within 0.4% of each other at every level; R/X/SW1 checked. Nov 25: Esc/Space stop keys added for the assembled drone (no kit buttons), checked mid-ramp and at a prompt (7 checks). Not yet run on the board. |
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
2. Open `motor_calibration_test.c` here on GitHub, click **Copy raw file**, paste it into a new `main.c` in the project.
3. Copy `config.bits` from this folder into the project folder (next to main.c). Do **not** add it as a source file.
4. Build and program (the starter kit's debug USB, J1).

## Files

| File | What it is |
|---|---|
| `config.bits` | Configuration bits (120 MHz SYSCLK, 60 MHz PBCLK). `#include`d by main.c, never added as a source file. |
| `motor_calibration_test.c` | Motor calibration test - kitchen-scale thrust sweep that prints per-motor pulse trims |

## Using it (PuTTY: the kit's USB COM port, 115200 baud, 8N1)

The PIC can't measure thrust, so you read a kitchen scale and type the number in. The program does the math.

**Setup**

- Run `13_esc_calibrate_relay` first, so all 4 ESCs share the same 1.0-2.0 ms range (07 doesn't work with the relay in).
- Use a full battery.
- **Props on.** Put the drone centered on the scale (on a board if the legs don't fit). Nothing should be under the props or touching the drone.
- Let the USB cable hang loose.
- Any unit works if you use it for every reading. Taring with the drone on the scale is fine: the readings go negative.

**Steps**

1. Type `GO`. For each pulse level (1150, 1200, 1250, 1300, 1350 us):
   - It asks for the scale reading with all motors stopped (the baseline).
   - It spins FL, FR, RL and RR one at a time at that pulse. Each motor ramps slowly, then gets 3 s to settle. Type each reading.
   - $\text{thrust} = \text{baseline} - \text{reading}$.
   - All 4 motors are measured at one level before the next level, so the battery slowly draining affects every motor the same way.
   - At a reading prompt: `R` spins that motor again, `X` aborts.
   - **No kit buttons needed:** `Esc` (or `Space` as the first key) stops all motors and opens the relay at any time, also while a motor is still spinning up. A reading typed before its prompt is kept.
2. It prints:
   - the thrust table and the spread at each level;
   - each motor's start pulse and strength;
   - the trims and `#define` lines to paste into the flight code.
3. Type `V 1250` to check. Each motor runs at its trimmed pulse; type the readings and it prints the new spread. Under about 5% is good.

**The math.** Thrust goes with the propeller speed squared, and speed rises about in a straight line with the pulse. So each motor $n$ is fitted (least squares, ignoring readings where it wasn't pushing yet) as

$$\sqrt{T_n}=a_n\,(p-z_n)$$

$z_n$ is where the motor starts to push, and $a_n$ is how strong it is. The average motor is $a_{\text{ref}}=\tfrac14\sum a_n$, $z_{\text{ref}}=\tfrac14\sum z_n$. Motor $n$ makes the average motor's thrust at

$$p_n=z_n+\frac{a_{\text{ref}}}{a_n}\,(p-z_{\text{ref}})=1000+g_n\,(p-1000)+o_n,\qquad g_n=\frac{a_{\text{ref}}}{a_n},\quad o_n=z_n-1000-g_n\,(z_{\text{ref}}-1000)$$

Apply the trims only to pulses above 1000 us: a stop must stay exactly 1000 us.
