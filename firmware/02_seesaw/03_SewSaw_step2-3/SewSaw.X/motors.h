/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/motors.h
 *  What    : Pulse limits (1000/2000 us) and the motor functions.
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
 *    initMotors()  -  Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC
 *        pulses (all start at 1000 us = stop).
 *    set_motor()  -  Sets the ESC pulse for motor n (1-4); forced to 'stop' unless the
 *        drone is armed and running.
 * ============================================================================================ */

#ifndef MOTORS_H
#define MOTORS_H

// ESC pulses on Output Compare, Timer2 (50 Hz, the rate that already spun your ESCs)
//   motor 1 = A = left  = ESC 1 signal on J12 pin 11 (RD0 = OC1), ground pin 14
//   motor 2 = B = right = ESC 2 signal on J12 pin 12 (RF2 = OC2), ground pin 20

#define PULSE_OFF  1000.0f  // us, motor stopped (also what the ESC needs to arm)
#define PULSE_IDLE 1100.0f  // us, slowest a motor goes while the PID runs (keeps it spinning)
#define PULSE_MAX  2000.0f  // us, full throttle

void initMotors(void);              // both outputs start at 1000 us so the ESCs arm
void set_motor(int n, float us);    // n = 1 (A) or 2 (B), us is clamped to 1000..2000

#endif
