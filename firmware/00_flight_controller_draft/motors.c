/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/motors.c
 *  What    : ESC pulse outputs: Timer2 at 50 Hz with OC1-OC4 (and the motor-power relay pin in
 *            v1.0+).
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initMotors(), set_motor(),
 *    motors_off()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "motors.h"

// 4 ESC signal wires, Output Compare in PWM mode on Timer2
//ESC 1 front left:  pin 11 (RD0 -> OC1)
//ESC 2 front right: pin 12 (RF2 -> OC2)
//ESC 3 back left:   pin 21 (RF4 -> OC3)
//ESC 4 back right:  pin 19 (RF5 -> OC4)
//ESC grounds: pins 14, 20, 25, 30. ESC red 5 V wires NOT connected.
//
// Timer2: 60 MHz / 8 = 7.5 MHz -> 7.5 ticks per microsecond
// period = (PR2+1) x 8 / 60 MHz = 18750 x 8 / 60 MHz = 2.5 ms (400 Hz)
// pulse: 1000 us = 7500 ticks (stop), 2000 us = 15000 ticks (full)

#define TICKS_PER_US 7.5f

// ---------------------------------------------------------------------------------------------
// void initMotors(void)
//   Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC pulses (all start at
//   1000 us = stop).
// ---------------------------------------------------------------------------------------------
void initMotors(void)
{
    TRISDbits.TRISD0=0;
    RPD0R=12;           //RD0 = OC1
    TRISFbits.TRISF2=0;
    RPF2R=11;           //RF2 = OC2
    TRISFbits.TRISF4=0;
    RPF4R=11;           //RF4 = OC3
    TRISFbits.TRISF5=0;
    RPF5R=11;           //RF5 = OC4

    T2CON=0x0000;
    TMR2=0;
    PR2=18749;          // 2.5 ms
    OC1R=7500;          // start every ESC at 1000 us so they arm safely
    OC2R=7500;
    OC3R=7500;
    OC4R=7500;
    OC1RS=7500;
    OC2RS=7500;
    OC3RS=7500;
    OC4RS=7500;
    OC1CON=0x8006;      // PWM mode, Timer2
    OC2CON=0x8006;
    OC3CON=0x8006;
    OC4CON=0x8006;
    T2CON=0x8030;       // Timer2 ON, 1:8
}

// ---------------------------------------------------------------------------------------------
// void set_motor(int n, float us)
//   Sets the ESC pulse for motor n (1-4); forced to 'stop' unless the drone is armed and
//   running.
// ---------------------------------------------------------------------------------------------
void set_motor(int n, float us)
{
    int ticks;
    if (us<PULSE_OFF) us=PULSE_OFF;     // never send a pulse outside 1-2 ms
    if (us>PULSE_MAX) us=PULSE_MAX;
    ticks=us*TICKS_PER_US+0.5f;   // +0.5 rounds to the nearest tick
    if (n==1) OC1RS=ticks;
    if (n==2) OC2RS=ticks;
    if (n==3) OC3RS=ticks;
    if (n==4) OC4RS=ticks;
}

// ---------------------------------------------------------------------------------------------
// void motors_off(void)
//   KILL: every ESC to 1000 us, latch the kill, and (with a relay) open the motor-power relay.
// ---------------------------------------------------------------------------------------------
void motors_off(void)
{
    set_motor(1, PULSE_OFF);
    set_motor(2, PULSE_OFF);
    set_motor(3, PULSE_OFF);
    set_motor(4, PULSE_OFF);
}
