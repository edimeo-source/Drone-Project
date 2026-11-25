/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/motors.c
 *  What    : ESC pulse outputs: Timer2 at 50 Hz with OC1-OC4 (and the motor-power relay pin in
 *            v1.0+).
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initMotors(), set_motor()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "motors.h"

// ESC pulses, Output Compare in PWM mode on Timer2.
// Same settings as esc_full_speed.c, which spun both of your ESCs.
//
// Timer2: 60 MHz / 64 = 937,500 ticks per second = 0.9375 ticks per us
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms between pulses = 50 Hz
//   1000 us = 938 ticks (stop)     2000 us = 1875 ticks (full)
//
// To try 400 Hz later (faster ESC response), change two lines:
//   T2CON=0x8030 (1:8)  and  TICKS_PER_US 7.5f   (PR2 stays 18749 -> 2.5 ms)

#define TICKS_PER_US 0.9375f

// ---------------------------------------------------------------------------------------------
// void initMotors(void)
//   Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC pulses (all start at
//   1000 us = stop).
// ---------------------------------------------------------------------------------------------
void initMotors(void)
{
    TRISDbits.TRISD0=0;
    RPD0R=12;               // RD0 = OC1 -> J12 pin 11 = ESC 1 = motor A (left)
    TRISFbits.TRISF2=0;
    RPF2R=11;               // RF2 = OC2 -> J12 pin 12 = ESC 2 = motor B (right)

    T2CON=0x0000;           // Timer2 off while setting up
    TMR2=0;
    PR2=18749;              // 20 ms frame
    OC1R=938;               // 1000 us = motor off, so the ESCs arm
    OC1RS=938;
    OC2R=938;
    OC2RS=938;
    OC1CON=0x8006;          // OC1 ON, PWM mode, uses Timer2
    OC2CON=0x8006;          // OC2 ON, PWM mode, uses Timer2
    T2CON=0x8060;           // Timer2 ON, 1:64
}

// ---------------------------------------------------------------------------------------------
// void set_motor(int n, float us)
//   Sets the ESC pulse for motor n (1-4); forced to 'stop' unless the drone is armed and
//   running.
// ---------------------------------------------------------------------------------------------
void set_motor(int n, float us)
{
    int ticks;

    if (us<PULSE_OFF) us=PULSE_OFF;     // never outside 1000..2000 us
    if (us>PULSE_MAX) us=PULSE_MAX;
    ticks=(int)(us*TICKS_PER_US+0.5f);  // microseconds -> timer ticks

    if (n==1) OC1RS=ticks;              // new width starts at the next 20 ms frame
    if (n==2) OC2RS=ticks;
}
