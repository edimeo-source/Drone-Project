/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/02_SewSaw_step0-1/SewSaw.X/imu.h
 *  What    : IMU angles/rates and the IMU functions.
 *  Version : SewSaw v1   |   Date: 2026-11-16
 *  Status  : WORKED: step 0 (IMU check) passed and step 1 ran on the board Nov 16.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC pins 11/12 are reserved for the seesaw motors but not driven yet (motors.c arrives
 *      in step 2-3)
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
