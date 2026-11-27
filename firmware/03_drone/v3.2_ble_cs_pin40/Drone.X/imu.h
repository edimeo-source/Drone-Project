/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.2_ble_cs_pin40/Drone.X/imu.h
 *  What    : IMU angles/rates and the IMU functions.
 *  Version : v3.2   |   Date: 2026-11-26
 *  Status  : CURRENT. v3.1 with the Bluefruit CS moved from pin 13 (RJ13) to pin 40 (RJ3);
 *            nothing else changed. Host tests pass (87 checks; the Bluetooth simulation checks
 *            CS on RJ3 for every byte). Not yet built in MPLAB.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
 *      VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g.
 *      9)
 *      SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 40 (RJ3, moved from pin 13 in v3.2)
 *      IRQ and RST not connected
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    initSPI()  -  Sets up SPI1 (pin remaps, mode, clock speed) and the chip-select pin for
 *        this version's wiring.
 *    imu_who_am_i()  -  Reads the WHO_AM_I register (an LSM6DSO always answers 0x6C).
 *    initIMU()  -  Resets the LSM6DSO and writes every setting, reading each one back.
 *        Returns how many did not stick (0 = all good).
 *    imu_calibrate()  -  Averages the IMU while the drone sits still and level to find the
 *        gyro and accel offsets (retries if it moved).
 *    imu_reset_angles()  -  Starts roll and pitch from the accelerometer (gravity) and sets
 *        yaw to 0.
 *    imu_update()  -  Runs every 4 ms: reads the IMU and updates roll, pitch, yaw with the
 *        complementary filter (alpha 0.98).
 * ============================================================================================ */

#ifndef IMU_H
#define IMU_H

// Updated every 4 ms by imu_update() (called from the control interrupt).
// Signs, with the IMU's X arrow pointing to the front and Z up:
//   roll  + = left side up      pitch + = nose up      yaw + = nose turns left
extern volatile float roll, pitch, yaw;                  // degrees
extern volatile float roll_rate, pitch_rate, yaw_rate;   // degrees per second, from the gyro
extern volatile float ax, ay, az;                        // accelerometer, g
extern volatile float temp_c;                            // chip temperature, C
extern volatile int imu_error;                           // 1 = the IMU stopped answering

void initSPI(void);
int imu_who_am_i(void);         // should be 0x6C
int initIMU(void);              // writes the settings, returns how many did NOT stick (0 = good)
int imu_calibrate(void);        // board still and level for 2 s, returns 1 if it worked
void imu_reset_angles(void);    // roll/pitch from gravity, yaw = 0
void imu_update(void);          // read + complementary filter, every 4 ms

#endif
