/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/noise.h
 *  What    : Noise-test functions and flag.
 *  Version : SewSaw v2   |   Date: 2026-11-16
 *  Status  : Written and handed over; the seesaw stage was marked done Nov 18 (no separate
 *            hardware log).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC red 5 V wires . NOT connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    noise_start()  -  Starts the 60 s noise test (the 4 ms loop begins collecting
 *        samples).
 *    noise_sample()  -  Runs every 4 ms during the noise test: adds this sample to the sums
 *        and min/max.
 *    noise_report()  -  Prints the noise test results (RMS noise, jitter, drift) with
 *        PASS/FAIL for each.
 * ============================================================================================ */

#ifndef NOISE_H
#define NOISE_H

extern volatile int noise_on;       // 1 while the 60 s test runs
extern volatile long noise_n;       // samples so far (250 per second)

void noise_start(void);             // main: start the test
void noise_sample(void);            // the 4 ms interrupt: add one sample
void noise_report(void);            // main: print the results and PASS / FAIL

#endif
