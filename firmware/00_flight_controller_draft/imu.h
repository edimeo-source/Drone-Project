/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/imu.h
 *  What    : IMU angles/rates and the IMU functions.
 *  Version : FC draft   |   Date: 2026-11-04
 *  Status  : Written as practice code before the hardware arrived. Never flown. Superseded by
 *            the SewSaw seesaw code (the IMU moved to SPI).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (I2C2)
 *      3V3 pin 1   GND pin 6   SDA pin 3 (RA3)   SCL pin 5 (RA2)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Bluefruit SPI Friend (shares SPI1 with the IMU)
 *      VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    initIMU()  -  Starts I2C2, reads WHO_AM_I and writes the IMU settings. Returns
 *        WHO_AM_I (should be 0x6C).
 *    imu_calibrate()  -  Averages the IMU while the drone sits still and level to find the
 *        gyro and accel offsets (retries if it moved).
 *    imu_update()  -  Runs every 4 ms: reads the IMU and updates roll, pitch, yaw with the
 *        complementary filter (alpha 0.98).
 * ============================================================================================ */

#ifndef IMU_H
#define IMU_H

// Angles in degrees, rates in degrees per second (updated every 4 ms by imu_update)
extern volatile float roll, pitch;                       // + roll = left side up, + pitch = nose up
extern volatile float roll_rate, pitch_rate, yaw_rate;   // gyro, bias removed
extern volatile int imu_error;                           // 1 = the IMU stopped answering

int initIMU(void);          // returns WHO_AM_I (should be 0x6C)
void imu_calibrate(void);   // drone must sit still, about 1 second
void imu_update(void);      // read + complementary filter, called from the control interrupt

#endif
