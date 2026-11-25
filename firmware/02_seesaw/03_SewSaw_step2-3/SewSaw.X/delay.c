/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/delay.c
 *  What    : Timer3 millisecond delays.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initDelay(), delay_ms()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "delay.h"

// Delays use Timer3 (same as Lab 6).
// Timer2 is saved for the ESC pulses (Step 3) and Timer4 is the 4 ms control loop.
// 60 MHz / 256 = 234,375 counts per second -> 234 counts = 1 ms

// ---------------------------------------------------------------------------------------------
// void initDelay(void)
//   Starts Timer3 (1:256 prescale) so delay_ms() can count milliseconds.
// ---------------------------------------------------------------------------------------------
void initDelay(void)
{
    T3CON=0x8070;       // Timer3 ON, 1:256
}

// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
void delay_ms(int n)
{
    for (int i=0;i<n;i++){
        TMR3=0;
        while(TMR3<234);
    }
}
