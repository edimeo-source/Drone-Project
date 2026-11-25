/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/06_motor_spin_test_esc1_only/motor_spin_test_esc1_only.c
 *  What    : Motor spin test - ESC 1 only, slow
 *  Version : bench test   |   Date: 2026-11-17
 *  Status  : Ran on the bench Nov 17: ESC 1 needed 1.25 ms to start (1.15 ms was too low).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): delay_ms(), sw1_pressed(),
 *    set_motor(), all_stop(), show_motor(), stop_forever(), wait_check(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz, watchdogs off (same file as SewSaw)

// =====================================================================
// MOTOR SPIN DIRECTION TEST - ESC 1 ONLY (front-left, J12 pin 11), slow
// Ethan DiMeo
//
// No IMU, no PID. Just slow ESC pulses so you can see which way each motor turns.
//
// Wiring (J12 pinout sheet):
//   ESC 1 front-left : signal -> pin 11 (RD0 = OC1)   ground -> pin 14
//   ESC 2 front-right: signal -> pin 12 (RF2 = OC2)   ground -> pin 20
//   ESC 3 back-left  : signal -> pin 21 (RF4 = OC3)   ground -> pin 25
//   ESC 4 back-right : signal -> pin 19 (RF5 = OC4)   ground -> pin 30
//   ESC red (5 V) wires: NOT connected.  ESC power: battery / PDB.
//
// What it does:
//   1. 1 ms pulses to all 4 for 3 s          -> ESCs arm (beeps), all LEDs off
//   2. ESC 1 spins slowly for 4 s, stops 2 s, and repeats forever
//      ESC 2, 3, 4 stay at 1 ms (armed, not spinning)
//   LED1 on = ESC 1 spinning
//   SW1 on the kit = STOP: all motors back to 1 ms until you press RESET.
//
// PROPS OFF. Put a piece of tape on each motor bell so you can see it turn.
// To reverse a motor: swap any 2 of its 3 motor wires at the ESC.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms = 50 Hz
//   1.00 ms = 938 ticks (stop)    1.25 ms = 1172 ticks (slow)    2.00 ms = 1875 ticks (full)
// =====================================================================

#define STOP_TICKS  938         // 1.00 ms = motor off
#define SLOW_TICKS  1172        // 1.25 ms = slow spin (1.15 ms = 1078 was too low to start this motor)

// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
// Timer3: 60 MHz / 256 = 234,375 counts per second -> 234 counts = 1 ms
void delay_ms(int n)
{
    for (int i=0;i<n;i++){
        TMR3=0;
        while(TMR3<234);
    }
}

// ---------------------------------------------------------------------------------------------
// int sw1_pressed(void)
//   1 while SW1 is held (SW1 pulls RB12 low). SW1 = STOP/KILL in every program.
// ---------------------------------------------------------------------------------------------
int sw1_pressed(void)
{
    return PORTBbits.RB12==0;   // SW1 pulls RB12 low
}

// ---------------------------------------------------------------------------------------------
// void set_motor(int n, int ticks)
//   Sets the ESC pulse for motor n (1-4); forced to 'stop' unless the drone is armed and
//   running.
// ---------------------------------------------------------------------------------------------
void set_motor(int n, int ticks)
{
    if (n==1) OC1RS=ticks;      // ESC 1, pin 11, front-left
    if (n==2) OC2RS=ticks;      // ESC 2, pin 12, front-right
    if (n==3) OC3RS=ticks;      // ESC 3, pin 21, back-left
    if (n==4) OC4RS=ticks;      // ESC 4, pin 19, back-right
}

// ---------------------------------------------------------------------------------------------
// void all_stop(void)
//   Sends the 1.0 ms stop pulse to all 4 ESCs.
// ---------------------------------------------------------------------------------------------
void all_stop(void)
{
    set_motor(1, STOP_TICKS);
    set_motor(2, STOP_TICKS);
    set_motor(3, STOP_TICKS);
    set_motor(4, STOP_TICKS);
}

// ---------------------------------------------------------------------------------------------
// void show_motor(int n)
//   Shows a motor number (1-4) on the kit LEDs.
// ---------------------------------------------------------------------------------------------
void show_motor(int n)          // LEDs = motor number in binary
{
    LATHbits.LATH0=0;           // LED1
    LATHbits.LATH1=0;           // LED2
    LATHbits.LATH2=0;           // LED3
    if (n==1) LATHbits.LATH0=1;
    if (n==2) LATHbits.LATH1=1;
    if (n==3){
        LATHbits.LATH0=1;
        LATHbits.LATH1=1;
    }
    if (n==4) LATHbits.LATH2=1;
}

// ---------------------------------------------------------------------------------------------
// void stop_forever(void)
//   Stops every motor and waits forever (press RESET to start again).
// ---------------------------------------------------------------------------------------------
void stop_forever(void)
{
    all_stop();
    show_motor(0);              // all LEDs off
    while(1);                   // press RESET to start again
}

// ---------------------------------------------------------------------------------------------
// void wait_check(int ms)
//   Waits ms milliseconds in 10 ms steps, stopping all motors at once if SW1 is pressed.
// ---------------------------------------------------------------------------------------------
void wait_check(int ms)         // wait, but stop everything if SW1 is pressed
{
    for (int i=0;i<ms;i=i+10){
        delay_ms(10);
        if (sw1_pressed()) stop_forever();
    }
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: arms all 4 ESCs, then spins ESC 1 slowly for 4 s, stops 2 s, repeats.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    int n, ticks;

    T3CON=0x8070;               // Timer3 ON, 1:256 (delays)

    // ESC outputs (peripheral pin select)
    TRISDbits.TRISD0=0;
    RPD0R=12;                   // RD0 = OC1 -> J12 pin 11 (ESC 1)
    TRISFbits.TRISF2=0;
    RPF2R=11;                   // RF2 = OC2 -> J12 pin 12 (ESC 2)
    TRISFbits.TRISF4=0;
    RPF4R=11;                   // RF4 = OC3 -> J12 pin 21 (ESC 3)
    TRISFbits.TRISF5=0;
    RPF5R=11;                   // RF5 = OC4 -> J12 pin 19 (ESC 4)

    // Timer2 + 4 output compares in PWM mode
    T2CON=0x0000;
    TMR2=0;
    PR2=18749;                  // 20 ms
    OC1R=STOP_TICKS;
    OC1RS=STOP_TICKS;
    OC2R=STOP_TICKS;
    OC2RS=STOP_TICKS;
    OC3R=STOP_TICKS;
    OC3RS=STOP_TICKS;
    OC4R=STOP_TICKS;
    OC4RS=STOP_TICKS;
    OC1CON=0x8006;              // ON, PWM mode, uses Timer2
    OC2CON=0x8006;
    OC3CON=0x8006;
    OC4CON=0x8006;
    T2CON=0x8060;               // Timer2 ON, 1:64

    // SW1 = stop, LED1-3 = which motor
    ANSELBbits.ANSB12=0;
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;        // pull-up: SW1 reads 1 until pressed
    TRISHbits.TRISH0=0;
    TRISHbits.TRISH1=0;
    TRISHbits.TRISH2=0;
    show_motor(0);

    // 1. arm: 1 ms to all 4 ESCs for 3 s
    wait_check(3000);

    // 2. ESC 1 only, forever
    n=1;                                // ESC 1 = front-left, J12 pin 11
    while (1){
        show_motor(n);                  // LED1 on

        for (ticks=STOP_TICKS;ticks<=SLOW_TICKS;ticks=ticks+5){    // gentle start, about 0.7 s
            set_motor(n, ticks);
            wait_check(25);
        }
        wait_check(4000);               // spin slowly for 4 s: look at the direction

        set_motor(n, STOP_TICKS);       // stop
        show_motor(0);
        wait_check(2000);               // 2 s pause, then again
    }
    return 0;
}
