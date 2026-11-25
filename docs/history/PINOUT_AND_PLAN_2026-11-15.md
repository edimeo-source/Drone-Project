# ELC275 self-leveling quad: pinout and build plan

Ethan DiMeo, ELC 275-501. Written Nov 15, 2026. Demo: Wed Dec 2, 2026.

Pin source: Microchip DS70005230B (PIC32MZ EF Starter Kit User's Guide), Table 2-3 "J12 Connections",
checked against the XC32 device header for PIC32MZ2048EFM144 (which pins are remappable, which have analog modes).
Every remap code below (`SDI1R=13`, `RPD10R=5`, `RPD0R=12`, `RPF2R=11`, `RPF4R=11`, `RPF5R=11`) was checked against
the PIC32MZ EF datasheet DS60001320H, Table 12-2 (input pin selection) and Table 12-3 (output pin selection).

## 1. Hard facts that decide the pinout

- **IMU = LSM6DSO, on SPI1** (not I2C). WHO_AM_I = 0x6C. 3.3 V only.
  The breakout is NOT the one in the spreadsheet/CAD (SparkFun SEN-18020); measure its board and holes before printing the TPU pad.
- **Only 7 J12 pins can take a remappable function** (PWM output, SPI data): **7, 11, 12, 15, 19, 21, 37**.
  Every other GPIO on J12 (ports H, J, K) is plain on/off only.
  Those 7 go to: SPI1 data (7, 37), four ESC signals (11, 12, 19, 21), battery sensing (15).
- **Pin 3 (RA3) is I2C2 SDA**, the only I2C on J12. The four VL53L0X distance sensors need it,
  so the IMU's CS moves from pin 3 to **pin 16 (RK1)**.
- **Pins 8 and 10 (RG6/RB14) are the PC serial link** (UART2 → MCP2221 → J11 USB → PuTTY / PID tuner). Never wire them.
  SW3 is the same pin as U2TX (RB14): **never press SW3**.
- None of the J12 GPIO pins used below have analog modes, so no ANSEL lines are needed for them (only RB3 battery uses analog).
- Power the kit through **J4 micro-USB** from the PDB 5 V BEC (the guide lists J1 debug USB, J4, and J2 as the only power inputs; J12 +5 V is not an official input).

## 2. J12 header, final assignment

**The picture version of this table is `J12_PINOUT.html` (in the SewSaw.X project). It is the wiring rule for the whole project.**
The two must always match. A pin change is made in `J12_PINOUT.html` first, then here, in `CLAUDE.md` and in the code.

| J12 | PIC32 | Assigned to | Phase |
|---|---|---|---|
| 1 | +3.3 V | IMU 3V3 | now |
| 2 | +5 V | Bluefruit VIN | later |
| 3 | RA3 / SDA2 | VL53L0X SDA ×4 (I2C2). **Was IMU CS: move that wire to pin 16** | step 0 / later |
| 4 | +5 V | spare (buzzer if it is a 5 V type) | later |
| 5 | RA2 / SCL2 | VL53L0X SCL ×4 (I2C2) | later |
| 6 | GND | IMU GND | now |
| 7 | RA14 → SDI1 (`SDI1R=13`) | SPI1 MISO: IMU SDO + Bluefruit MISO | now |
| 8 | RG6 = U2RX | **reserved**: PC serial | never |
| 9 | GND | Bluefruit GND | later |
| 10 | RB14 = U2TX | **reserved**: PC serial | never |
| 11 | RD0 → OC1 (`RPD0R=12`) | ESC 1 front-left signal (seesaw motor A) | PWM phase |
| 12 | RF2 → OC2 (`RPF2R=11`) | ESC 2 front-right signal (seesaw motor B) | PWM phase |
| 13 | RJ13 | Bluefruit CS | later |
| 14 | GND | ESC 1 signal ground | PWM phase |
| 15 | RB3 / AN3 | battery divider (33 kΩ / 10 kΩ + 0.1 µF) | later |
| 16 | RK1 | **IMU CS** | step 0 |
| 17 | +3.3 V | VL53L0X VIN ×4 | later |
| 18 | RK2 | VL53L0X #1 XSHUT (front) | later |
| 19 | RF5 → OC4 (`RPF5R=11`) | ESC 4 back-right signal | full drone |
| 20 | GND | ESC 2 signal ground | PWM phase |
| 21 | RF4 → OC3 (`RPF4R=11`) | ESC 3 back-left signal | full drone |
| 22 | RK3 | VL53L0X #2 XSHUT (right) | later |
| 23 | RD1 = SCK1 | SPI1 clock: IMU SCL + Bluefruit SCK | now |
| 24 | RJ5 | VL53L0X #3 XSHUT (back) | later |
| 25 | GND | ESC 3 signal ground | full drone |
| 26 | RJ7 | VL53L0X #4 XSHUT (left) | later |
| 27, 28 | no connect | — | |
| 29 | RH7 | green LED + 330 Ω | later |
| 30 | GND | ESC 4 signal ground | full drone |
| 31 | RH9 | yellow LED + 330 Ω | later |
| 32 | RH11 | spare | |
| 33 | RK4 | red LED + 330 Ω | later |
| 34 | GND | LED + buzzer ground | later |
| 35 | RK5 | buzzer (2N3904 + 1 kΩ driver if weak) | later |
| 36 | RK6 | spare | |
| 37 | RD10 ← SDO1 (`RPD10R=5`) | SPI1 MOSI: IMU SDA + Bluefruit MOSI | now |
| 38 | RH15 | spare | |
| 39 | GND | VL53L0X ground | later |
| 40 | RJ3 | spare | |

## 3. By device (wiring view)

**IMU, LSM6DSO (SPI1, mode 3, 2 MHz):** 3V3 → 1 · GND → 6 · SCL → 23 · SDA → 37 · SDO → 7 · CS → **16** · INT1/INT2 not connected.

**ESCs (signal side only; red 5 V wire not connected):**
ESC1 FL 11 + GND 14 · ESC2 FR 12 + GND 20 · ESC3 BL 21 + GND 25 · ESC4 BR 19 + GND 30.
Seesaw uses ESC1 (pin 11) and ESC2 (pin 12) only.

**Bluefruit LE SPI Friend (SPI1 shared, mode 0, 1 MHz):** VIN → 2 · GND → 9 · SCK → 23 · MOSI → 37 · MISO → 7 · CS → 13 · IRQ/RST not connected.
Sharing rule: each driver writes its own `SPI1CON` before pulling its CS low; the Bluefruit code turns off the 4 ms control interrupt only while its CS is low.

**VL53L0X ×4 (I2C2, 100 kHz):** VIN → 17 · GND → 39 · SDA → 3 · SCL → 5 · XSHUT front 18, right 22, back 24, left 26.

**Status:** green 29 · yellow 31 · red 33 · buzzer 35 · ground 34.

**Battery sense:** PDB battery + → 33 kΩ → node → 10 kΩ → GND; node → pin 15; 0.1 µF pin 15 → GND. $V_{AN3} = V_{bat}\cdot 10/43$ (12.6 V → 2.93 V).

**On the kit (no wiring):** SW1 RB12 = kill (pull-up) · SW2 RB13 = recalibrate · SW3 = do not use ·
LED1 RH0 = armed · LED2 RH1 = IMU OK / calibrated · LED3 RH2 = high during each 4 ms loop (scope it).

**Power:** 3S LiPo → PDB (470 µF across the battery pads) → ESCs. PDB 5 V BEC → USB cable → kit J4. Unplug J4 power whenever the PC is on the debug USB.

## 4. Peripherals and timers

| Block | Use | Setting |
|---|---|---|
| SPI1 | IMU + Bluefruit | IMU: `SPI1CON=0x8060`, `SPI1BRG=14` (mode 3, 2 MHz). Bluefruit: `0x8120`, `SPI1BRG=29` (mode 0, 1 MHz) |
| UART2 | PC serial (PuTTY / PID tuner) | 115200, `U2BRG=129`, interrupt-driven TX buffer |
| Timer2 | ESC PWM frame for OC1–OC4 | decided in the PWM phase (400 Hz plan: 1:8, `PR2=18749`) |
| Timer3 | `delay_ms` | 1:256, 234 ticks = 1 ms |
| Timer4 | 4 ms control loop interrupt | 1:64, `PR4=3749`, priority 5 |
| I2C2 | VL53L0X ×4 | `I2C2BRG=292` (100 kHz, proven in Lab 5) |
| ADC | battery on AN3 | later |

## 5. Plan, one step at a time

Each step ends with a check. Don't start the next step until the check passes.

**Step 0: rewire (5 min).** Move the IMU CS jumper from J12 pin 3 to pin 16. Nothing else moves.
*Rewired Nov 15.* Check: flash `imu_test.hex` (test program, CS now on pin 16); PuTTY must show WHO_AM_I = 0x6C and live data.
**Passed Nov 15**, after pressing on the IMU's SDO pin. The SDO contact (pin 7 ↔ IMU SDO) is flaky: when it's bad, SPI reads
0x7F (pull-up) / 0x00 (pull-down) even though the I2C wire-by-wire test says every wire is GOOD. Before flying, solder that
header pin properly (and use a fresh jumper or a soldered lead), because vibration will break a weak contact.

**Step 1: gyro data you can trust (no motors).**
1. New MPLAB project with separate files: `main.c`, `imu.c/.h` (SPI), `uart.c/.h`, `delay.c/.h`, `control.c/.h` (4 ms Timer4 loop), `config.bits` (included as `#include "config.bits"`, same as the class labs).
2. IMU at 833 Hz, ±4 g, ±500 dps, with the chip's own low-pass filters on; read once every 4 ms in the Timer4 interrupt.
3. Calibrate with the still-check; complementary filter ($\alpha = 0.98$, $\Delta t = 0.004$ s).
4. The firmware prints a measured noise report. Pass marks:
   - WHO_AM_I = 0x6C
   - loop period 4.000 ms (LED3 on the scope)
   - at rest for 60 s: gyro noise ≤ 0.1 °/s RMS, angle jitter ≤ ±0.1°, drift ≤ 0.2°
   - signs: left side up → roll +, nose up → pitch +
   - known tilts (0°, 15°, 30°) read within ±1°
   - after quick ±45° tilts, back to level within ±0.5° in 1 s

*Written Nov 15: folder `Step1_Gyro/`.* Files: `main.c`, `imu.c/.h`, `control.c/.h`, `pid.c/.h`, `tuner.c/.h`, `noise.c/.h`,
`uart.c/.h`, `delay.c/.h`, `config.bits`. It builds with XC32 v5.00 (`Step1_Gyro.hex`). It already includes the Step 2 PID tuner link.
Settings: gyro ±500 °/s with a 49 Hz low-pass filter; accel ±4 g with an 18.5 Hz filter; both at 833 Hz; read every 4 ms.

How to test Step 1 (PuTTY, 115200 baud):
1. MPLAB: open `C:\Users\Ethan\MPLABXProjects\SewSaw.X` (set up Nov 16: all 8 `.c`, 7 `.h` and `config.bits` are in it, and it builds).
   `config.bits` is marked "excluded from build" because `main.c` already includes it; compiling it twice breaks the link.
   Build, then program the board.
2. At startup you should see WHO_AM_I 0x6C, then 7 settings reading `OK`, then "Calibrated". Lay the board level before it calibrates.
3. Signs: lift the left side → ROLL goes +. Lift the nose (the X arrow end) → PITCH goes +.
4. Known tilts: prop the board at 15° and at 30° (a phone protractor app on the board works). ROLL should read within ±1°.
5. Tip it about 45° quickly and set it back down level. ROLL/PITCH should return within ±0.5° of 0 in about 1 s.
6. Press `n` and don't touch the table for 60 s. The report must say `RESULT: PASS`
   (loop 3990..4010 µs, gyro noise ≤ 0.10 °/s RMS, jitter ≤ ±0.10°, drift ≤ 0.20°).
7. Optional: scope LED3 (RH2). It should be a pulse every 4.000 ms, about 0.1 ms wide.

**Step 2: seesaw + PID tuner, still no motors.**
1. Seesaw rig: one beam, **two identical motors, one on each end**, pivoting on a rod through the middle.
   Set it up like this (stand in front of it):
   - beam runs **left to right**: motor A on your **left** (ESC on pin 11), motor B on your **right** (ESC on pin 12)
   - the pivot rod points **straight away from you**
   - IMU lies flat at the middle of the beam, its text the right way up for you to read, so its **X arrow points away from you, along the rod**
   - then the seesaw angle is **roll**, and **left end up = roll +**
2. `tuner.c/.h`: speaks the PID tuner's protocol, the zip version (`tools/pid-tuner/pid_tuner.py`):
   receives `K,kp,ki,kd` (replies `A,kp,ki,kd`), `S,deg`, `R,from,to,ms`, `L,1` / `L,0`, `X`;
   sends `D,t_ms,angle,setpoint,motor_us,P,I,D` and `RUN_BEGIN` … `RUN_END`.
   The Desktop copy of the tuner uses the same commands, so the firmware works with either.
3. Practice in the tuner's Demo mode first.
4. PIC32 mode: tilt the seesaw by hand. Check: the graph follows your hand, and motor_us goes UP for the low side and DOWN for the high side. `X` and SW1 kill.

**Step 3: PWM to the ESCs (props OFF).**
1. Confirm the ESC signal (standard 1000–2000 µs; rate 400 Hz, or 50 Hz if they refuse).
2. `motors.c/.h`: OC1 (pin 11) and OC2 (pin 12) on Timer2; start at 1000 µs so the ESCs arm.
3. Scope pins 11/12; then spin each motor slowly from the tuner; check direction.
   Pass: clean pulses at the chosen rate, motors arm, spin smoothly, and stop on `X` / SW1.

**Step 4: seesaw tuning with motors (props ON, rig clamped).**
Seesaw motors: A (left) = base + u (pin 11), B (right) = base − u (pin 12).
Left end up → roll + → error $e = 0 - \text{roll} < 0$ → $u < 0$ → A slows, B speeds up → left end comes down. Tune Lynch's way: raise $K_p$ until it oscillates, back off, add $K_d$ until damped, small $K_i$ last.
Pass: back to level within about 1 s after a push, no growing wobble.

**Step 5: full drone.** 4 ESCs + X mixer → tether test → short hover.

**Step 6: extras, if time.** Bluefruit arm/kill, LEDs + buzzer, battery ADC, VL53L0X ×4.

## 6. Corrections to the handoff files

1. IMU bus is **SPI1**, not I2C2 (`imu.c`, wiring page and CLAUDE.md say I2C2 on pins 3/5).
2. IMU CS moves to **pin 16**, so pins 3/5 stay I2C2 for the distance sensors.
3. Report draft still says ISM330DHCX: change to LSM6DSO.
4. The IMU breakout is not the SparkFun SEN-18020 in the spreadsheet/CAD: re-measure the board for the TPU pad and tech box.
5. Pitch sign: decided **nose up = +** everywhere (handoff + mixer convention). The test program was changed to match on Nov 15.
6. Two PID tuner versions exist: the zip's `tools/pid-tuner/pid_tuner.py` (1,767 lines) and the Desktop copy (954 lines, saved after the zip was made).
   **Decision (Nov 15): use the zip version.** Both use the same commands, so the firmware works with either.
7. J12 +5 V pins are not an official power input; power the kit through J4.
8. The PIC32MZ datasheet in the zip (`DS60001320H`) is fine. It's the full datasheet saved as plain text with a `.pdf` name, which is why a PDF reader won't open it.
