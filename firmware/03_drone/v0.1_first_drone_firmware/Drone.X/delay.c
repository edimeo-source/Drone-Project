/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v0.1_first_drone_firmware/Drone.X/delay.c
 *  What    : Timer3 millisecond delays.
 *  Version : v0.1   |   Date: 2026-11-18
 *  Status  : Host-checked only; not built in MPLAB as of Nov 18. Base for every later version.
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
