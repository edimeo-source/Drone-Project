# ELC275 Drone v3.1: two PIDs (roll + pitch), tuned one axis at a time, plus motor adjust buttons
Ethan DiMeo, Nov 25 2026

Firmware 3.0 follows the professor's PID notes: **PID 1 = roll, PID 2 = pitch**, each motor gets its own pulse, you tune one axis at a time with the other correction at 0 (like the seesaw), then run both together.

**v3.1 adds live + / - buttons for each motor** (and turn CW / turn CCW), for the drone spinning on the tether. See section 5. Everything else is the same as 3.0.

## What is in this folder

| Folder / file | What it is |
|---|---|
| `Drone.X/` | MPLAB X project, firmware 3.1 (XC32, `config.bits`, 120 MHz). Built from 3.0: same pins, relay, heartbeat, Bluetooth. |
| `host_tests/` | PC tests of the firmware (`sh run_tests.sh`): 87 checks, including flying a simulated drone with the real 4 ms loop. |
| `HOW_TO_RUN.md` | This file. |

The tools for 3.1:
- **Phone app 3.1:** [`tools/android_drone_tuner/v3.1_motor_adjust_2026-11-25_CURRENT`](../../../tools/android_drone_tuner/v3.1_motor_adjust_2026-11-25_CURRENT) (`DroneTuner.apk`).
- **PC tuner v8:** [`tools/pc_pid_tuner/v8_ble_motor_adjust_2026-11-25_CURRENT`](../../../tools/pc_pid_tuner/v8_ble_motor_adjust_2026-11-25_CURRENT) (USB serial or Bluetooth).

App 3.0 and tuner v7 still work with 3.1, but they have no motor adjust buttons. The 2.0 app and tuner v6 can't select an axis or send roll gains.

**Tests on a PC:** run `sh host_tests/run_tests.sh` first. It writes the real firmware output to `host_tests/fw_out/`, and the tuner and app tests read it:
- tuner, from its folder: `python tests/test_v8.py`
- app, from its folder: `ANDROID_JAR=<SDK>/platforms/android-33/android.jar sh test/run_test.sh`

## 1. What changed from 2.0

| | 2.0 | 3.0 |
|---|---|---|
| Controllers | 1 PID (pitch) | 2 PIDs: roll (RC) and pitch (PC) |
| Motor outputs | 2 groups (front pair, back pair) | 4 different pulses |
| Throttle | base front + base back | one throttle $T$ for all four (`B,T`) |
| Tuning | pitch only | `A,1` roll only, `A,2` pitch only, `A,3` both |
| Gains | `K` | `KR` roll, `KP` pitch (`K` = the selected axis) |
| Targets | `S,deg` | `SR,deg`, `SP,deg`, `S,roll,pitch` in Both |
| Start-up | top limit climbs 4 µs per loop | soft start at `U,rate` µs/s (default 100), also used by manual mode |
| Motor matching | none | per-motor trims in `motors.h` (from bench test 11) |
| Tip cutoff | 75° | 60° |
| Phone step test | up to 5 s | up to 15 s |

## 2. The professor's plan, line by line

### Two PIDs
Each PID is the same Lynch form as before ($e$ = error, $\dot e = -\omega$ so the derivative never kicks at a setpoint step):

$$RC = K_{p,r}\,e_r + K_{i,r}\!\int e_r\,dt + K_{d,r}\,\dot e_r, \qquad e_r = \phi_{sp} - \phi$$

$$PC = K_{p,p}\,e_p + K_{i,p}\!\int e_p\,dt + K_{d,p}\,\dot e_p, \qquad e_p = \theta_{sp} - \theta$$

$\phi$ = roll, $\theta$ = pitch. Each integral is limited to $\pm 20$ and each output to $\pm 500$ µs.

### The mixer (all four pulses different)
ESC 1 = FL (J12 pin 11), ESC 2 = FR (pin 12), ESC 3 = RL (pin 21), ESC 4 = RR (pin 19).

$$
\begin{aligned}
m_{FL} &= T + PC + RC \\
m_{FR} &= T + PC - RC \\
m_{RL} &= T - PC + RC \\
m_{RR} &= T - PC - RC
\end{aligned}
$$

Each pulse is held at 1100 µs or more (props keep turning) and 2000 µs or less, then the motor's trim is applied.

- **Pitch:** front pair $+PC$, back pair $-PC$. The professor's "1 +, 2 −, 3 −, 4 +" is the same thing with motors numbered around the frame (his 1 and 4 are the front).
- **Roll:** left pair $+RC$, right pair $-RC$. The professor drew the minus on the left. Which side gets the plus depends on the IMU's sign, and ours is: **roll + = left side up** (`imu.c`). Check: if the left side is too high, $\phi > \phi_{sp}$, so $e_r < 0$, so $RC < 0$, so the left motors slow down and the left side drops. With the other sign the PID pushes the wrong way and the drone flips (the host test shows this ends at the 60° cutoff). **That is why section 4B checks the signs with the props off.**

### One axis at a time = the seesaw
- **`A,1` Roll only:** $PC = 0$, so $m_{FL} = m_{RL} = T + RC$ and $m_{FR} = m_{RR} = T - RC$. The left pair against the right pair is exactly the seesaw.
- **`A,2` Pitch only:** $RC = 0$, so the front pair against the back pair.
- **`A,3` Both:** both PIDs run, all four pulses differ.

The professor's example: liftoff at 30 %, so 28 % on one pair and 32 % on the other tilts it. In pulses that is $T = 1500$, $RC = -20$: left pair 1480 µs, right pair 1520 µs.

The axis can only change while the motors are off. The idle PID's integral is held at 0, and each integral only runs once the soft start has reached $T$ (no wind-up on the ground).

### Throttle $T$ "slightly above liftoff"
$T$ is the average pulse. Find the suspend point first (section 4C), then set $T$ a little above it. The soft start ramps from 1000 µs to $T$ at the `U` rate, so the time to reach $T$ is

$$t_{ramp} = \frac{T - 1000\ \mu s}{\text{rate}}, \qquad \text{e.g. } \frac{1500 - 1000}{100\ \mu s/s} = 5\ \text{s}.$$

## 3. Commands (PuTTY 115200, PC tuner v8, or phone app 3.1)

| Command | What it does |
|---|---|
| `A,1` / `A,2` / `A,3` | Axis: roll only / pitch only / both. Motors must be off. Reply `AXIS,n,NAME`. |
| `KR,kp,ki,kd` / `KP,kp,ki,kd` | Roll / pitch gains. Reply `AR,...` / `AP,...`. `K,...` sets the selected axis (both in Both). |
| `SR,deg` / `SP,deg` | Roll / pitch target. `S,deg` = the selected axis, `S,r,p` in Both. Reply `SETPOINT,r,p`. |
| `B,T` | Throttle for all four. Reply `BASE,T,T`. |
| `U,rate` | Soft start, 25 to 1000 µs/s. Reply `RAMP,...`. |
| `M,t` / `M,fl,fr,rl,rr` | Manual pulses (no PID), climbing at the `U` rate. Use it to find liftoff. |
| `O,fl,fr,rl,rr` / `O` | **3.1:** live + / - µs on each motor, any time (also while flying), each within ±200. Reply `OFFS,fl,fr,rl,rr`. `O` alone = show them. |
| `G` | PID hold: soft start to $T$, then hold the targets. |
| `R,from,to,ms` | Step test on the selected axis (Both: roll and pitch step together). |
| `ARM`, `H`, `X` | Relay on (needs the heartbeat), heartbeat, KILL. |
| `Z`, `C`, `STATUS`, `?` | Zero both angles, calibrate the gyro, status, help. |

PuTTY keys `c`, `z`, `n`, `x`, `h` work as before.

## 4. Step by step

### A. Flash and check (props OFF)
1. MPLAB X → open `Drone.X` → Build → Program. PuTTY at 115200 shows the banner **ELC275 drone v3.1 - roll PID + pitch PID**, `WHO_AM_I = 0x6C`, `Bluefruit OK`, the 2 s calibration, then `Bluefruit ready`.
2. Install `DroneTuner.apk` **3.1** on the phone. It installs over 3.0 and 2.0 (same signing key) and keeps your Bluetooth permission.
3. Do the relay meter check from the [2.0 guide](../v2.0_phone_app_relay_heartbeat/HOW_TO_RUN.md): hold reset, ESC power pads must read 0 V.

### B. Sign check (props OFF, relay open, no ARM needed)
The PIDs run even with the motors off, and PuTTY shows RC and PC.
1. Send `A,3`, `KR,10,0,0`, `KP,10,0,0` (capitals, Enter after each).
2. Hold the drone level and send `Z` (capital: zeroes roll and pitch).
3. Press lowercase `z` (no Enter). Commands pause the PuTTY lines, and any lowercase key brings them back (`z` only zeroes yaw).
4. **Left side down:** the roll column goes negative and **RC must go positive** (left motors would speed up).
5. **Nose down:** the pitch column goes negative and **PC must go positive** (front motors would speed up).
6. If either is backwards, stop and fix the sign before any props go on.
7. Optional, still props off: `ARM`, wait 2 s, `B,1200`, `G`, then tilt by hand. The low side's motors audibly speed up. `X` to stop.

### C. Liftoff throttle (props ON, tethered)
The professor's first step: all four the same pulse, raised slowly until it just lifts and hangs on the tether.
- **Standalone:** bench test [`12_liftoff_throttle_test`](../../01_bench_tests/12_liftoff_throttle_test) does this with `+` / `-` steps.
- **In 3.1:** `A,3` → `ARM` → wait 2 s → `M,1300` → `M,1350` → ... in 25–50 µs steps. Write down the pulse where it just lifts.
- Set $T$ about 20–40 µs above that: `B,T`.
- If one corner always lifts first, run bench test [`11_motor_calibration_test`](../../01_bench_tests/11_motor_calibration_test) and copy its `#define TRIM_...` lines into `motors.h`.

### D. Tune roll (`A,1`)
1. Motors off. `A,1`. `KR,1,0,0` (P only first, as the professor said). `SR,0`.
2. `ARM`, wait 2 s, then either:
   - **`G`** to hold level and watch, or
   - **step test** `R,0,5,3000`: hold 0° for 1 s after the soft start, step to 5°, record 3 s.
3. Kp too small: slow, never reaches 5°. Too big: oscillates. Raise Kp until it oscillates, back off to about half.
4. Add Kd to stop the overshoot, then a little Ki to remove the steady error.
5. `X` (KILL) any time. The motors stop at the end of every step test, and the data comes back.

### E. Tune pitch (`A,2`)
Same as D with `KP` and `SP`: `A,2`, `KP,1,0,0`, `SP,0`, step test `R,0,10,3000`.

### F. Both (`A,3`)
1. Motors off. `A,3`. Both gain sets are already on the drone. `S,0,0`.
2. `ARM`, wait 2 s, `G`. It should hold level on both axes with all four pulses different.
3. Step test in Both steps roll and pitch together: `R,0,5,3000`.

### G. The demo
Tethered, KILL ready. In Both:
1. **Flat:** `SR,0`, `SP,0`, `G`.
2. **Nose up 10°:** `SP,10`.
3. **Roll 5°:** `SP,0`, `SR,5`.
4. **Both:** `SR,5`, `SP,10`.
5. Cut power as soon as each attitude is shown (`X`). The professor said a free run isn't allowed in the demo room, and a big angle makes it drift on the tether.

On the phone: Axis = Both, then the **Hold angle** card (Set targets) and **PID hold (G)**. On the PC tuner: Axis = Both and the **Hold** row.

## 5. Motor adjust buttons (v3.1): when it spins on the tether

### Why it spins
Each prop pushes air sideways as it turns, and the air twists the frame the other way. Seen from above (motor ID test 10):
- **FL and RR spin CCW**, which twists the frame CW;
- **FR and RL spin CW**, which twists the frame CCW.

At equal speeds the twists cancel. If the drone turns **counterclockwise**, the FR + RL pair is twisting harder than the FL + RR pair: those motors are stronger, or their props drag more.
The professor's plan has no yaw PID (only roll and pitch), so nothing corrects it by itself. The buttons give it a fixed correction.

### Check this first (props OFF)
A fast spin with weak lift usually means a wrong part, not a small mismatch:
- Run `10_motor_id_test`. Each motor must turn the way it prints: FL CCW, FR CW, RL CW, RR CCW.
- Each prop must match its motor's direction. A CW prop on a CCW motor gives almost no lift.
- Make sure the tether line itself isn't twisted and winding up.

### The fix: ↻ turn CW
On the phone: **Motor adjust** card, step **10 µs**, press **↻ turn CW** once, then watch for a few seconds. It sends:

$$\text{FL} + d,\quad \text{FR} - d,\quad \text{RL} - d,\quad \text{RR} + d \qquad (\texttt{O,10,-10,-10,10}\ \text{for}\ d = 10)$$

Each diagonal pair moves together, so the left pair and the front pair keep the same total. That means roll and pitch don't change:

$$(m_{FL} + d) + (m_{RL} - d) = m_{FL} + m_{RL}, \qquad (m_{FL} + d) + (m_{FR} - d) = m_{FL} + m_{FR}$$

- Still turning CCW, but slower: press it again.
- Faster: press **↺ turn CCW** (your props are the other way round).
- When it holds its heading, stop.

### The rest of the card
- **+ / − on each motor:** add or take away the step (5, 10 or 25 µs) on that motor only. With the PID running (`G`), roll and pitch will partly fight a single-motor change, because it tilts the drone. In manual (`M,t`) you see the change directly. For a spin, use the turn buttons.
- **Limits:** each motor ±200 µs. The PID's 1100 µs idle floor still holds, a stopped motor stays stopped, and KILL works as always.
- **Persistence:** the drone starts at 0 at every power-up. The phone (and the PC tuner) saves the values and sends them again on every connect. "on the drone:" shows what it is really using.
- **Permanent:** once it holds, you can copy the four numbers into `TRIM_OFFSET_FL` / `_FR` / `_RL` / `_RR` in `motors.h`, then press **Zero all**.
- **PuTTY:** `O,10,-10,-10,10` does the same, and `O` shows the values.

## 6. How long the step-test data takes over Bluetooth
Samples recorded: $N = 50 + \frac{\text{ms}}{4}$ (the 50 are the 0.2 s before the step). Measured in the host test: 8.4 bytes per sample for one axis, 15.1 for Both. At 100–300 B/s:

$$N = 50 + \frac{3000}{4} = 800, \qquad \text{one axis: } \frac{800 \times 8.4}{300} \approx 22\ \text{s} \;\text{to}\; \frac{800 \times 8.4}{100} \approx 67\ \text{s}$$

$$\text{Both: } \frac{800 \times 15.1}{300} \approx 40\ \text{s} \;\text{to}\; \frac{800 \times 15.1}{100} \approx 121\ \text{s}$$

The phone allows up to 15 s ($N = 3800$), which in Both is about 3 to 10 minutes. Keep step tests at 3 s unless you need more. Over the USB cable (PC tuner, Serial) there is no such wait.

## 7. Safety (unchanged from 2.0 unless noted)
- KILL (`X`), SW1, no heartbeat for 500 ms, phone disconnect, IMU error, or **±60°** tip: motors off, relay open, ARM again.
- ARM needs: calibrated, SW1 up, within 60°, motors off.
- Axis changes are refused while the motors run.
- Every step test ends with the motors off. Tether the drone for every powered test.

## 8. What the host tests show (`sh run_tests.sh`)
- **Mixer signs and isolation:** in Roll only, PC stays 0. In Pitch only, RC stays 0. Each integral only runs on its own axis, and only after the ramp.
- **Simulated flight** (thrust $\propto x^2$, arm 0.159 m, $I = 0.0075\ \text{kg·m}^2$, real 4 ms loop):
  - levels in Roll, Pitch, and Both;
  - Ki corrects a weak FL motor and a 4 mm center-of-mass offset;
  - holds the demo targets (roll 5°, pitch 10°, both);
  - a wrong roll sign flips it into the 60° cutoff.
- **Data:** every Bluetooth sample arrives exactly (one axis and Both). The phone's mixer matches the firmware's four ESC pulses on real samples.
- **Motor adjust (3.1):**
  - `O,10,-10,-10,10` during PID hold gives FL 1510, FR 1490, RL 1490, RR 1510 on the next loop, and RC and PC stay 0.
  - Limits: ±200 µs, the 1100 µs idle floor, and a stopped motor stays stopped. KILL keeps the values.
  - The phone's and the PC tuner's turn CW button sends exactly that command.

The host tests use a physics model, not your drone: real gains come from sections D–F on the tether.
