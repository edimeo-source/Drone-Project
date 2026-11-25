/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/uart.h
 *  What    : UART2 functions.
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
 *    initU2()  -  Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC
 *        tuner.
 *    putU2()  -  Sends one character out UART2 to the PC (waits while the transmit buffer
 *        is full).
 *    putsU2()  -  Sends a text line to the PC over UART2, followed by a new line.
 * ============================================================================================ */

#ifndef UART_H
#define UART_H

void initU2(void);
int putU2(int c);
void putsU2(char *s);

#endif
