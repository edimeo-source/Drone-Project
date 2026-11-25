# PIC32 drone firmware: notes for merging the Bluetooth module

Board: PIC32MZ2048EFM144 Starter Kit (DM320007). XC32 v6.00, MPLAB X 6.15.
Clocks (from `config.bits`): SYSCLK 120 MHz, PBCLK 60 MHz. Multi-vector interrupts.

## What is in this zip

| Folder | What it is | Status |
|---|---|---|
| `A_current_firmware_Drone.X/` | The firmware on the board now: pitch PID on 4 ESCs, driven from the PC PID tuner over UART2. Full MPLAB project (sources, `config.bits`, `Makefile`, `nbproject/`). | Built with XC32 and running on the board with the tethered pitch rig. No Bluetooth. |
| `B_bluetooth_stop_draft/` | Folder A plus the Bluetooth motor-stop code from `Drone_Bluetooth_Code.md` (Nov 18): new `bluefruit.c/.h`, and new versions of `control.c/.h`, `motors.c`, `main.c`, `tuner.c`, `uart.c`, `delay.c`, `imu.c`. All other files are unchanged from A. | Passed a host syntax check only. **Never built with XC32 and never run on hardware.** |
| `C_pc_pid_tuner/` | The PC side of the UART2 link (`pid_tuner.py`, Python). | In use with folder A. |

Start from folder A when merging. Folder B shows one way the Bluetooth stop was already wired into A.

## Reply to the Nov 20 handoff ("ELC275 Drone Bluetooth Integration — Project Handoff")

### Priority 1 items

| Requested | Where it is |
|---|---|
| PIC32 main C source | `A/main.c` |
| PID controller | `A/control.c` (Timer4 interrupt, every 4 ms) and `A/pid.c` |
| ESC PWM | `A/motors.c`, `A/motors.h` |
| SPI1 driver `spi1_devices.c/.h` | **No file by that name exists in this project, now or earlier.** `initSPI()` in `imu.c` sets up SPI1 for the IMU. The only Bluefruit SPI/SDEP code that matches this firmware is `bluefruit.c` in folder B (`bus_take()`, `bus_release()`, packet send/receive). `drone_ble_link.c` needs to be ported onto that code, or bring its own SDEP layer. |
| IMU driver | `A/imu.c`, `A/imu.h` |
| `config_bits.h` | The file is named `config.bits`. `main.c` includes it, and it is excluded from the build. |
| Wiring diagram | The J12 table further down is current and includes the Nov 19 relay change. The wiring PNGs on file are older than that change, so they are left out. |

### Corrections to the handoff

- **Pins.** All six pins in the handoff's table match the firmware.
  - MISO is confirmed as RA14 → SDI1 (`SDI1R = 13`), J12 pin 7.
  - SCK is RD1, pin 23. MOSI is RD10 (`RPD10R = 5`), pin 37.
  - The CS lines are RK1 (pin 16) and RJ13 (pin 13). The relay is RK4 (pin 33).
- **Bluefruit SPI clock.** The firmware runs the Bluefruit at 1 MHz (`SPI1BRG = 29`, mode 0), not 500 kHz. That setting worked in Labs 5–6, and SDEP allows up to 4 MHz. The IMU runs at 2 MHz in mode 3, so every Bluefruit transaction has to switch the mode and clock, then restore them.
- **Relay.** It is not an SSR. It is an electromechanical relay module (RM1HLE, 10 A contacts). It is active-HIGH on CH1, driven straight from RK4, and was bench-tested Nov 19. The 10 A warning stands.
- **Device name.** The module advertises its default name, not `PIC32-DRONE`. To rename it, send `AT+GAPDEVNAME=PIC32-DRONE` and then `ATZ` once. The name is stored in the module's flash and stays with the module until `AT+FACTORYRESET`. Alternatively, the tuner can find the module by the UART service UUID instead of the name.
- **Physical disconnect.** SW1 is a software kill (the PIC reads it). The only stop that works without the PIC and without Bluetooth is pulling the battery XT60. A hands-off physical disconnect would need extra hardware.
- **Live telemetry during step tests.** The handoff asks for live telemetry independent of step recording. That conflicts with the professor's rule built into the firmware: nothing is transmitted while a step test records, so printing can never cost a 4 ms sample. Keep that rule for transmitting. Receiving (`H`, `X`) must still be polled during recording.

### BLE bandwidth: the plan's biggest risk

A `D` line is about 46 bytes, for example `D,123456,-1.23,0.00,1523,-12.3,0.0,-3.4,1477\r\n`. One SDEP packet carries at most 16 data bytes, so each line takes 3 packets.

$$\text{Live at 50 Hz: } 50 \times 46 \approx 2300\ \tfrac{\text{B}}{\text{s}}$$

$$\text{20 s step test: } 5050 \times 46 \approx 232\ \text{kB}$$

The earlier estimate for the Bluefruit SPI Friend's UART service is roughly 100–300 B/s sustained, and a few kB/s at best. It has **not** been measured on this board. At 300 B/s:

- the 50 Hz stream needs about 8 times that rate;
- a 20 s dump takes $232\,000 / 300 \approx 770\ \text{s} \approx 13\ \text{min}$.

Even at 2 kB/s the dump takes about 2 minutes.

Recommended order:

1. **Measure first.** Have the PIC send numbered lines as fast as SDEP allows. Have the tuner count bytes per second and missing line numbers. Set the rates from that number.
2. **Until then, BLE carries only:** commands, ARM/KILL, the heartbeat and a decimated live stream (start at 5–10 Hz). Step-test dumps stay on USB (UART2), which already works.
3. **If dumps move to BLE:** use shorter tests, and send without blocking (one SDEP packet per service call, polling RX between packets).

### Heartbeat and failsafe

- **The dump happens with the motors still running.** After a step test finishes recording, `motor_mode` stays 1, so the PID keeps flying while the data goes out. `tuner_send_run()` blocks `main()` while it prints (about 20 s on UART2). A blocking dump over BLE would stop RX polling, the heartbeat would time out, and the motors would be killed in the middle of the dump. The BLE sender has to be non-blocking.
- **Put the timeout check in the 4 ms interrupt.** This also removes the handoff's limitation that the failsafe depends on the foreground code still running:
  - `main()` stores the core-timer count each time it parses an `H` line.
  - The interrupt only compares that count with the current one: no SPI, no Bluetooth calls. If the drone is armed and the last `H` is older than 350 ms, it calls `motors_off()` and drives RK4 low.
  - A stuck `main()` stops refreshing the count, so the interrupt still kills the motors.
  - Use unsigned subtraction. The core timer wraps every $2^{32} / 60\ \text{MHz} \approx 71.6\ \text{s}$.
- **RX polling sets the real timeout.** The PIC only sees an `H` when it polls UART RX over SDEP (command `0x0A02`). Poll RX at least every 50 ms, and give it priority over telemetry TX. With a 100 ms heartbeat and a 350 ms timeout, three missed heartbeats trip the stop. Test for false trips caused by Windows BLE write stalls.
- **Pick one arming scheme.** Folder B uses `ARM;` from the phone plus a 750 ms `AT+GAPGETCONN` link check. The handoff uses `ARM` plus an `H` heartbeat with a 350 ms timeout. The heartbeat is the better test, because it proves the whole laptop → BLE → PIC path. Keep only one `ARM` syntax.
- **Two command sources.** USB and BLE lines should both go to the same `tuner_command()`. Accept `X` from either source, and decide whether USB is allowed to `ARM`.
- **Relay.** It still needs adding (code below): RK4 low in `motors_off()` and in every interrupt stop path, and high only after ARM with 1000 µs pulses already going out.

## The 4 ms control loop (`control.c`)

Everything that flies runs in **one Timer4 interrupt** (`Controller`, IPL5). `main()` only handles the PC link, buttons and printing, so printing can never delay the loop.

$$T_{loop} = \frac{(PR4 + 1)\cdot 64}{f_{PB}} = \frac{3750 \cdot 64}{60\times10^{6}\ \text{Hz}} = 4\ \text{ms} \quad (250\ \text{Hz})$$

Order inside the interrupt, every 4 ms:

1. `imu_update()`: read the LSM6DSO over SPI1, complementary filter ($\alpha = 0.98$, $dt = 0.004\ \text{s}$) → `roll`, `pitch`, `pitch_rate`.
2. Safety. Any of these sets `motor_mode = 0` (off) and `off_reason`:
   - SW1 pressed (RB12 low) → `OFF_SW1`
   - IMU stopped answering → `OFF_IMU`
   - pitch or roll beyond $\pm 45^\circ$ → `OFF_TIP`
3. Soft start: the highest allowed pulse `ramp_us` climbs $4\ \mu\text{s}$ per loop, so 1000 → 2000 µs takes 1 s. It resets to 1000 µs whenever the motors are off.
4. `tuner_setpoint_step()` (step tests from the PC tuner may move the setpoint).
5. PID (below).
6. Mixer and ESC outputs (below).
7. `tuner_record()` and `noise_sample()`: store this sample in RAM arrays. Nothing is transmitted from inside the interrupt.

`loop_count` goes up by 1 every loop. `main()` watches it to do once-per-loop work outside the interrupt.

## PID (`pid.c`, called from `control.c`)

Lynch form, one axis (pitch). Only pitch is controlled right now; roll is held by tethers and only checked for the tip limit.

$$e = r - s = \text{setpoint} - \text{angle}$$

$$e_{int} \leftarrow e_{int} + e\,dt, \qquad |e_{int}| \le 20\ ^\circ\!\cdot\text{s}$$

$$\dot e = -\,\omega_{pitch} \quad \text{(gyro rate, not a difference of angles, so a setpoint step gives no derivative kick)}$$

$$u = K_p e + K_i e_{int} + K_d \dot e, \qquad |u| \le 500\ \mu\text{s}$$

- Gains are `volatile float Kp, Ki, Kd` in `control.c` (start 1, 0, 0). The tuner sets them with `K,kp,ki,kd`.
- $e_{int}$ is held at 0 unless the PID is flying and the soft start is finished (no wind-up while off or ramping).
- Sign: pitch + = nose up. Nose up → $e < 0$ → $u < 0$ → front slows, back speeds up.

Mixer (`motor_mode == 1`):

$$\text{front} = \text{base\_front} + u, \qquad \text{back} = \text{base\_back} - u$$

Each is floored at 1100 µs (`PULSE_IDLE`, props keep turning), then capped by `ramp_us` and by 2000 µs. Front pair = ESC 1 + ESC 2, back pair = ESC 3 + ESC 4. `motor_mode == 2` is manual (fixed pulses from the tuner), `0` is off (1000 µs).

## ESC PWM (`motors.c`, `motors.h`)

Output Compare 1–4 in PWM mode, all on Timer2, 1:64.

$$T_{PWM} = \frac{(PR2 + 1)\cdot 64}{60\times10^{6}\ \text{Hz}} = \frac{18750 \cdot 64}{60\times10^{6}} = 20\ \text{ms} \quad (50\ \text{Hz})$$

$$N_{ticks} = 0.9375 \cdot t_{\mu s}: \qquad 1000\ \mu\text{s} \to 938, \qquad 2000\ \mu\text{s} \to 1875$$

| ESC | Position | J12 pin | PIC32 pin | OC | Remap | Signal ground |
|---|---|---|---|---|---|---|
| 1 | front-left | 11 | RD0 | OC1 | `RPD0R = 12` | pin 14 |
| 2 | front-right | 12 | RF2 | OC2 | `RPF2R = 11` | pin 20 |
| 3 | rear-left | 21 | RF4 | OC3 | `RPF4R = 11` | pin 25 |
| 4 | rear-right | 19 | RF5 | OC4 | `RPF5R = 11` | pin 30 |

- `initMotors()` runs first in `main()` and starts all four at 1000 µs so the ESCs arm and stay stopped.
- `set_motor(n, us)` clamps to 1000–2000 µs and writes `OCxRS`. The new width takes effect at the next 20 ms frame, so writing it at any time is safe.
- `motors_off()` (control.c) sets `motor_mode = 0` and writes 1000 µs to all four **immediately**, not at the next loop. Use it for every stop.
- Do not call `set_motor()` from `main()` while the loop runs: the interrupt overwrites all four outputs every 4 ms.

## Resources already used

| Resource | Used for |
|---|---|
| Timer2 | ESC PWM, 50 Hz |
| Timer3 | `delay_ms()` (1:256, busy wait) |
| Timer4 | 4 ms control loop, IPL5 |
| Core timer | loop timing (`loop_period_us`, `loop_busy_us`); folder B also uses it for Bluetooth timeouts |
| UART2, 115200 baud | PC link through the kit's USB (J11): PuTTY or the PID tuner. RX interrupt IPL3 into a 256-byte ring buffer. RG6 = U2RX, RB14 = U2TX = J12 pins 8 and 10: never wire those pins. |
| SPI1 | LSM6DSO IMU: SPI mode 3, 2 MHz (`SPI1CON = 0x8060`, `SPI1BRG = 14`), CS = RK1 (J12 pin 16) |
| SPI1 (shared) | Bluefruit LE SPI Friend: SPI mode 0, 1 MHz (`SPI1CON = 0x8120`, `SPI1BRG = 29`), CS = RJ13 (J12 pin 13). IRQ and RST not connected. |
| RB12 / RB13 | SW1 = KILL, SW2 = recalibrate (SW3 is on the UART transmit pin: never press it) |
| RH0 / RH1 / RH2 | LED1 = motors on, LED2 = IMU calibrated, LED3 = high during the interrupt (scope it to see the 4 ms loop) |
| RAM | Step-test recording: 7 float arrays of 5050 samples, about 141 KB |

## Where the Bluetooth module should hook in

**SPI1 is shared, so this is the main rule.** The IMU is read inside the 4 ms interrupt in mode 3. A Bluefruit transaction (mode 0) must:

- disable the Timer4 interrupt (`IEC0bits.T4IE = 0`) for the length of one SDEP packet;
- set mode 0 and 1 MHz, hold CS 13 low for the whole packet (Bluefruit needs at least 100 µs after CS falls);
- restore `SPI1CON` and `SPI1BRG` for the IMU **before** re-enabling `T4IE`.

Folder B does this in `bus_take()` / `bus_release()` in `bluefruit.c`. It also sends at most one packet per call (about 0.3 ms) and only starts one when `TMR4 <= 2800`:

$$\frac{2800}{60\ \text{MHz}/64} = \frac{2800}{937.5\ \text{kHz}} \approx 2.99\ \text{ms}$$

so a packet always ends before the next 4 ms tick. Never wait for a Bluefruit reply with the interrupt disabled.

**Commands from the phone.** Parse them in `main()`'s loop, never in the interrupt. To stop, call `motors_off()`. To start, set the variables the interrupt reads (`motor_mode`, `base_front`, `base_back`, `setpoint`, `Kp`/`Ki`/`Kd`); the interrupt applies them on its next pass.

**Telemetry every 4 ms.** Call the sampling function from the interrupt at step 7, next to `tuner_record()`. It must only copy values into a RAM buffer: no SPI, UART or Bluetooth calls inside the interrupt. Transmit from `main()`.

Note: a BLE link cannot carry a 4 ms stream live. The Bluefruit SPI Friend's UART service manages roughly 100–300 bytes per second in practice. Even at 8 bytes per sample:

$$250\ \tfrac{\text{samples}}{\text{s}} \times 8\ \tfrac{\text{B}}{\text{sample}} = 2000\ \tfrac{\text{B}}{\text{s}}$$

That is several times what the link carries. Options: sample every 4 ms but send a decimated stream (about 5–10 Hz) over BLE, or record a full run in RAM and dump it afterwards (the way `tuner.c` does over UART). The PC tuner keeps the full-rate data on USB/UART2. Even UART2 at 115200 baud (about 11.5 kB/s) carries only every second sample live: the `D` lines go out every 8 ms.

**Failsafe.** The link check has to run even if `main()` stalls. Folder B calls `bluetooth_watchdog()` from the interrupt, before the safety checks. The main loop asks the module `AT+GAPGETCONN` every 100 ms. If no good answer has come back for 750 ms, the watchdog calls `motors_off()` (`off_reason = OFF_BLE`). After any stop (tuner `X`, SW1, tip, IMU error, phone STOP, link loss, calibrate, noise test) the motors stay off until a fresh `ARM;` arrives from the phone. `ARM;` only gives permission: the PC tuner still has to start the motors.

## Relay (motor power kill): not in folder A or B yet

The battery reaches the ESCs only through a relay. Bench-tested Nov 19: RK4 toggling once a second for a minute, no misses; reset and USB unplug both released it.

- Pin: **RK4 = J12 pin 33**, active-HIGH. HIGH = relay closed = motors powered. LOW, reset (pin becomes an input) or no PIC power = relay open.
- RK4 is digital only (PORTK has no ANSEL). Init order, before anything else can arm:

```c
LATKbits.LATK4=0;       // output latch low first
TRISKbits.TRISK4=0;     // then make it an output -> relay stays open
```

- In `motors_off()`: `LATKbits.LATK4=0;`. In folder A, the safety branches in the interrupt set `motor_mode = 0` directly instead of calling `motors_off()`, so they also need `LATKbits.LATK4=0;`. In folder B they already call `motors_off()`.
- On ARM: the ESCs must already be seeing 1000 µs. Then set `LATK4 = 1`, and keep 1000 µs until the ESC arming beeps finish before allowing any throttle.
- A full kill = all four pulses to 1000 µs **and** RK4 low. Folder B's phone STOP only does the pulses.

## J12 wiring (current harness, 18 pins used)

| J12 | Signal | J12 | Signal |
|---|---|---|---|
| 1 | +3.3 V out → sensor hub (IMU and Bluefruit VIN) | 16 | IMU CS (RK1) |
| 2 | +5V_EXT **in** from the 7805 (powers the kit) | 19 | ESC 4 signal (OC4) |
| 6 | GND → sensor hub | 20 | ESC 2 ground |
| 7 | MISO (SDI1, RA14), IMU and Bluefruit | 21 | ESC 3 signal (OC3) |
| 11 | ESC 1 signal (OC1) | 23 | SCK1 (RD1), IMU and Bluefruit |
| 12 | ESC 2 signal (OC2) | 25 | ESC 3 ground |
| 13 | Bluefruit CS (RJ13) | 30 | ESC 4 ground |
| 14 | ESC 1 ground | 33 | Relay CH1 (RK4) |
| | | 34 | GND → relay board |
| | | 37 | MOSI (SDO1, RD10), IMU and Bluefruit |

Leave pins 8 and 10 empty (UART2 to the PC).

Folder B's comments (and `Drone_Bluetooth_Code.md`) say Bluefruit VIN = pin 2, GND = pin 9. That is out of date. In the current harness, the Bluefruit is powered from pin 1 and pin 6 through the sensor hub, and pin 2 is the 5 V **input** from the 7805. Only the comments are affected; the code doesn't use those pins.

Power: battery + → relay COM → NO → power board → 4 ESCs. The 7805 is tapped from battery + before the relay and feeds J12 pin 2. A kill cuts only the motors; the PIC32, IMU and Bluefruit stay powered. The ESC red (BEC) wires are not connected.

## PC tuner protocol (UART2, folder A, `tuner.c`)

Tuner → PIC, one line each:
- `K,kp,ki,kd` new gains (PIC answers `A,...`)
- `S,deg` setpoint
- `R,from,to,ms` step test
- `L,1` / `L,0` live data on / off
- `X` kill
- `G` PID on
- `M,f,b` manual pulses
- `B,f,b` PID base pulses
- `Z` zero the angle
- `C` calibrate
- `?` help

PIC → tuner:
- `D,t_ms,angle,setpoint,front_us,P,I,D,back_us` one sample
- `RUN_BEGIN` … `RUN_END` a recorded step test, sent after the recording finishes

Folder B blocks `R`, `G` and `M` unless the phone has armed (`control_can_start()`). Phone commands in folder B: `STOP`, `ARM;`, `STATUS`.

## Open items before a powered flight

1. Folder B (or the merged code) has to be built with XC32 and bench-tested with the props off and motor power disconnected. Check the pulse widths first, then that every stop path brings all four ESCs to 1000 µs.
2. The relay output (above) is not written in either folder.
3. The relay module on the bench is rated 10 A. Hover is about $4 \times 4.4\ \text{A} = 17.6\ \text{A}$, so flight needs a 30 A (or larger) module. The 10 A one is for low-throttle bench tests only.
4. ESC throttle calibration has not been done.
5. Only pitch is controlled (tethered first stage). Free flight needs roll (and yaw) loops and a 4-motor mixer.
