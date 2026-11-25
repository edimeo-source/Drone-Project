/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/uart.h
 *  What    : UART2 functions.
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
 *    initU2()  -  Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC
 *        tuner.
 *    putU2()  -  Sends one character out UART2 to the PC (waits while the transmit buffer
 *        is full).
 *    putsU2()  -  Sends a text line to the PC over UART2, followed by a new line.
 *    getU2()  -  Returns the next character the PC sent (from the receive buffer), or -1 if
 *        there is none.
 *    fstr()  -  Turns a float into text with a fixed number of decimals (XC32's printf may
 *        leave out %f).
 * ============================================================================================ */

#ifndef UART_H
#define UART_H

void initU2(void);
int putU2(int c);
void putsU2(char *s);                           // sends the text, then a new line
int getU2(void);                                // next character from the PC, or -1 if none
char *fstr(char *buf, float v, int decimals);   // float -> text, like "-12.34"

#endif
