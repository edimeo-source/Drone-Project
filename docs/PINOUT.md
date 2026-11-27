# J12 pinout, every version

All connections are on the **J12 header of the PIC32MZ EF Starter Kit**. Pin functions come from:

- DS70005230B, Table 2-3;
- the remap codes in DS60001320H, Tables 12-2 / 12-3.

This page is generated from the same tables that are printed at the top of every source file.

## Current wiring (v3.2: v2.0 with the Bluefruit CS on pin 40)

| J12 pin | PIC32 pin | Connected to |
|---|---|---|
| 1 | +3.3 V | IMU 3V3 |
| 2 | +5V_EXT (input) | +5 V from the 7805 on the power/relay board (powers the kit) and Bluefruit VIN |
| 6 | GND | IMU GND |
| 7 | RA14 -> SDI1 (`SDI1R=13`) | SPI1 MISO: IMU SDO + Bluefruit MISO |
| 8 | RG6 = U2RX | **PC serial, never wire** |
| 9 | GND | Bluefruit GND (any GND pin works) |
| 10 | RB14 = U2TX (= SW3) | **PC serial, never wire, never press SW3** |
| 11 | RD0 -> OC1 (`RPD0R=12`) | ESC 1 front-left signal |
| 12 | RF2 -> OC2 (`RPF2R=11`) | ESC 2 front-right signal |
| 14 | GND | ESC 1 signal ground |
| 16 | RK1 | IMU CS |
| 19 | RF5 -> OC4 (`RPF5R=11`) | ESC 4 back-right signal |
| 20 | GND | ESC 2 signal ground |
| 21 | RF4 -> OC3 (`RPF4R=11`) | ESC 3 back-left signal |
| 23 | RD1 = SCK1 | SPI1 clock: IMU SCL + Bluefruit SCK |
| 25 | GND | ESC 3 signal ground |
| 30 | GND | ESC 4 signal ground |
| 33 | RK4 | Relay CH1: HIGH = motor power on, LOW / reset = power cut |
| 34 | GND | Power/relay board GND |
| 37 | RD10 -> SDO1 (`RPD10R=5`) | SPI1 MOSI: IMU SDA + Bluefruit MOSI |
| 40 | RJ3 (digital only) | Bluefruit CS (v3.2; pin 13 up to v3.1) |

Kit parts used by the code:

- LED1-LED3 = RH0-RH2;
- SW1 = RB12 (STOP everywhere);
- SW2 = RB13;
- SW3 = RB14 (never press it).

Front = the end opposite the battery pads. White arms = left, red arms = right.
The IMU's X arrow points to the true front.

## Wiring by version (exactly what each file's header says)

### First flight-controller draft (multi-file, I2C IMU, Bluefruit app control)  (2026-11-04)

[`firmware/00_flight_controller_draft`](../firmware/00_flight_controller_draft)

```text
IMU LSM6DSO (I2C2)
  3V3 pin 1   GND pin 6   SDA pin 3 (RA3)   SCL pin 5 (RA2)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU)
  VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
  CS pin 13 (RJ13)   IRQ and RST not connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### ESC full-speed test (2 motors) + ESC calibration mode  (2026-11-12)

[`firmware/01_bench_tests/01_esc_full_speed`](../firmware/01_bench_tests/01_esc_full_speed)

```text
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Basic IMU test over SPI (WHO_AM_I, accel, gyro, roll/pitch)  (2026-11-12)

[`firmware/01_bench_tests/02_imu_spi_test`](../firmware/01_bench_tests/02_imu_spi_test)

```text
IMU LSM6DSO (SPI1, CS on pin 3 in this era)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 3 (RA3)
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

### IMU roll / pitch / yaw over SPI (complementary filter)  (2026-11-12)

[`firmware/01_bench_tests/03_imu_roll_pitch_yaw_spi`](../firmware/01_bench_tests/03_imu_roll_pitch_yaw_spi)

```text
IMU LSM6DSO (SPI1, CS on pin 3 in this era)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 3 (RA3)
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

### IMU wiring test - LSM6DSO over SPI with CS on pin 16  (2026-11-15)

[`firmware/01_bench_tests/04_imu_wiring_test`](../firmware/01_bench_tests/04_imu_wiring_test)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
```

### Motor spin-direction test - all 4 ESCs, one motor at a time  (2026-11-17)

[`firmware/01_bench_tests/05_motor_spin_test_one_at_a_time`](../firmware/01_bench_tests/05_motor_spin_test_one_at_a_time)

```text
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Motor spin test - ESC 1 only, slow  (2026-11-17)

[`firmware/01_bench_tests/06_motor_spin_test_esc1_only`](../firmware/01_bench_tests/06_motor_spin_test_esc1_only)

```text
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### ESC throttle calibration - all 4 ESCs at once  (2026-11-18)

[`firmware/01_bench_tests/07_esc_calibrate`](../firmware/01_bench_tests/07_esc_calibrate)

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

### Motor test - all 4 ESCs, same pulse, same time  (2026-11-18)

[`firmware/01_bench_tests/08_motor_test_all4_together`](../firmware/01_bench_tests/08_motor_test_all4_together)

```text
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Bluefruit + relay test - phone START/STOP switches the motor-power relay  (2026-11-23)

[`firmware/01_bench_tests/09_bluefruit_relay_test`](../firmware/01_bench_tests/09_bluefruit_relay_test)

```text
Bluefruit SPI Friend
  VIN pin 2 (+5 V)   GND pin 6   SCK pin 23 (RD1)   MOSI pin 37 (RD10)   MISO pin 7 (RA14)
  CS pin 13 (RJ13)   IRQ and RST not connected
IMU CS pin 16 (RK1) is held HIGH so the IMU stays off the bus
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
PuTTY . UART2 over the J11 USB port, 9600 baud 8N1 (this test runs on the 8 MHz FRC clock)
```

### Motor ID test - spin the motor you name from PuTTY (RR 1150) to check wiring and direction  (2026-11-25)

[`firmware/01_bench_tests/10_motor_id_test`](../firmware/01_bench_tests/10_motor_id_test)

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

### Motor calibration test - kitchen-scale thrust sweep that prints per-motor pulse trims  (2026-11-25)

[`firmware/01_bench_tests/11_motor_calibration_test`](../firmware/01_bench_tests/11_motor_calibration_test)

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

### Liftoff throttle test - full-range (to 2000 us) motor control from PuTTY with +/- steps, to find the lowest pulse that lifts the drone  (2026-11-25)

[`firmware/01_bench_tests/12_liftoff_throttle_test`](../firmware/01_bench_tests/12_liftoff_throttle_test)

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

### ESC throttle calibration from PuTTY, through the motor-power relay - all 4 ESCs at once, no kit buttons  (2026-11-25)

[`firmware/01_bench_tests/13_esc_calibrate_relay`](../firmware/01_bench_tests/13_esc_calibrate_relay)

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

### Seesaw PID - single file (I2C IMU, 2 motors)  (2026-11-12)

[`firmware/02_seesaw/01_seesaw_pid_single_file`](../firmware/02_seesaw/01_seesaw_pid_single_file)

```text
IMU LSM6DSO (I2C2)
  3V3 pin 1   GND pin 6   SDA pin 3 (RA3)   SCL pin 5 (RA2)
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
Backup: motor B's pulse also comes out on pin 21 (RF4 -> OC3)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### SewSaw.X - seesaw firmware, steps 0-1 (IMU on SPI, filter, PC tuner link)  (2026-11-16)

[`firmware/02_seesaw/02_SewSaw_step0-1`](../firmware/02_seesaw/02_SewSaw_step0-1)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC pins 11/12 are reserved for the seesaw motors but not driven yet (motors.c arrives in step 2-3)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### SewSaw.X - seesaw firmware, steps 2-3 (motors + PID on the seesaw)  (2026-11-16)

[`firmware/02_seesaw/03_SewSaw_step2-3`](../firmware/02_seesaw/03_SewSaw_step2-3)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC red 5 V wires . NOT connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v0.1 - first 4-motor firmware (SewSaw code moved to the quad)  (2026-11-18)

[`firmware/03_drone/v0.1_first_drone_firmware`](../firmware/03_drone/v0.1_first_drone_firmware)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v0.2 - partner's Bluetooth STOP draft (adds bluefruit.c)  (2026-11-20)

[`firmware/03_drone/v0.2_bluetooth_stop_draft`](../firmware/03_drone/v0.2_bluetooth_stop_draft)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU)
  VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
  CS pin 13 (RJ13)   IRQ and RST not connected
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v1.0 - BLE integration (Bluetooth STOP/ARM, relay, BLE PC tuner)  (2026-11-20)

[`firmware/03_drone/v1.0_ble_integration`](../firmware/03_drone/v1.0_ble_integration)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU)
  VIN pin 1 (3.3 V, shared with the IMU)   GND pin 6   SCK pin 23   MOSI pin 37   MISO pin 7
  CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v1.1 - review fixes on top of v1.0  (2026-11-20)

[`firmware/03_drone/v1.1_ble_review_fixes`](../firmware/03_drone/v1.1_ble_review_fixes)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU)
  VIN pin 1 (3.3 V, shared with the IMU)   GND pin 6   SCK pin 23   MOSI pin 37   MISO pin 7
  CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v1.2 - 75 deg tip cutoff + record-then-send step runs  (2026-11-22)

[`firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs`](../firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU)
  VIN pin 1 (3.3 V, shared with the IMU)   GND pin 6   SCK pin 23   MOSI pin 37   MISO pin 7
  CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v2.0 - phone app over Bluetooth, relay kill, 500 ms heartbeat  (2026-11-23)

[`firmware/03_drone/v2.0_phone_app_relay_heartbeat`](../firmware/03_drone/v2.0_phone_app_relay_heartbeat)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
  VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g. 9)
  SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v3.0 - roll PID + pitch PID, 4 different ESC pulses, one axis at a time  (2026-11-25)

[`firmware/03_drone/v3.0_two_pid_roll_pitch`](../firmware/03_drone/v3.0_two_pid_roll_pitch)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
  VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g. 9)
  SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v3.1 - v3.0 + live +/- per motor (O command, app buttons) for a yaw spin  (2026-11-25)

[`firmware/03_drone/v3.1_motor_adjust_buttons`](../firmware/03_drone/v3.1_motor_adjust_buttons)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
  VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g. 9)
  SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 13 (RJ13)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

### Drone.X v3.2 - v3.1 with the Bluefruit CS on J12 pin 40 (RJ3)  (CURRENT)  (2026-11-26)

[`firmware/03_drone/v3.2_ble_cs_pin40`](../firmware/03_drone/v3.2_ble_cs_pin40)

```text
IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
  3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
  SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
  VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g. 9)
  SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 40 (RJ3, moved from pin 13 in v3.2)   IRQ and RST not connected
Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34 (GND)
PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
  pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
```

## Pin history in one line each

- **IMU:** I2C2 on pins 3/5 (Nov 4 draft, seesaw single file).
  SPI1 with CS on pin 3 (Nov 12 tests).
  SPI1 with **CS on pin 16** from Nov 15 on (pin 3 was kept free for I2C2).
- **Bluefruit:**
  - CS on pin 13 up to v3.1; **pin 40 (RJ3) from v3.2** (Nov 26).
  - VIN on pin 2 (5 V) in the Nov 4 draft, v0.2 and v2.0.
  - VIN on pin 1 (3.3 V) in v1.0-v1.2.
- **ESCs:** pins 11/12 (2 seesaw motors), then 11/12/21/19 for the quad. Grounds on 14/20/25/30.
- **Relay:** RK4 = pin 33, from v1.0 on.
