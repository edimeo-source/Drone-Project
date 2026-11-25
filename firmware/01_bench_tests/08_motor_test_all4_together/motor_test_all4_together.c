/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/08_motor_test_all4_together/motor_test_all4_together.c
 *  What    : Motor test - all 4 ESCs, same pulse, same time
 *  Version : bench test   |   Date: 2026-11-18
 *  Status  : Written Nov 18 to run after calibration (ramp to 1.25 ms, hard cap 2.0 ms). No
 *            result is logged in the project notes.
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
 *    set_all(), leds(), stop_forever(), wait_check(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz, watchdogs off (same file as SewSaw)

// =====================================================================
// MOTOR TEST - ALL 4 ESCs, SAME PULSE, SAME TIME
// Ethan DiMeo
//
// No IMU, no PID. All 4 motors get the exact same pulse at the exact same time.
//
// Wiring (J12 pinout sheet):
//   ESC 1 front-left : signal -> pin 11 (RD0 = OC1)   ground -> pin 14
//   ESC 2 front-right: signal -> pin 12 (RF2 = OC2)   ground -> pin 20
//   ESC 3 back-left  : signal -> pin 21 (RF4 = OC3)   ground -> pin 25
//   ESC 4 back-right : signal -> pin 19 (RF5 = OC4)   ground -> pin 30
//   ESC red (5 V) wires: NOT connected.  ESC power: battery / PDB.
//
// What it does:
//   1. 1 ms pulses to all 4 for 3 s           -> ESCs arm (beeps), LEDs off
//   2. all 4 ramp up together to SPIN_TICKS (about 1.2 s), spin 4 s, stop 2 s, repeat
//   LED1 + LED2 + LED3 on = motors spinning
//   SW1 on the kit = STOP: all motors back to 1 ms until you press RESET.
//
// PROPS OFF.
// Same pulse = same speed ONLY if the ESCs are calibrated to the same range.
// If one motor is slower or starts later, that ESC needs calibrating, not different code.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms = 50 Hz
//   ticks = 937.5 x (pulse in ms)
//   1.00 ms = 938 (stop)   1.15 ms = 1078   1.25 ms = 1172   1.50 ms = 1406   2.00 ms = 1875 (FULL)
//   Never above 1875: anything past 2.00 ms is past full throttle.
// =====================================================================

#define STOP_TICKS  938         // 1.00 ms = motor off
#define SPIN_TICKS  1172        // 1.25 ms = slow, all 4 should start here
#define MAX_TICKS   1875        // 2.00 ms = full throttle, hard limit

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
// void set_all(int ticks)
//   Sets the same pulse (in timer ticks) on all 4 ESCs, limited to 1.0-2.0 ms.
// ---------------------------------------------------------------------------------------------
void set_all(int ticks)         // same pulse to all 4 ESCs at once
{
    if (ticks>MAX_TICKS) ticks=MAX_TICKS;   // never past full throttle
    if (ticks<STOP_TICKS) ticks=STOP_TICKS;
    OC1RS=ticks;                // ESC 1, pin 11, front-left
    OC2RS=ticks;                // ESC 2, pin 12, front-right
    OC3RS=ticks;                // ESC 3, pin 21, back-left
    OC4RS=ticks;                // ESC 4, pin 19, back-right
}

// ---------------------------------------------------------------------------------------------
// void leds(int on)
//   Turns LED1-LED3 all on or all off.
// ---------------------------------------------------------------------------------------------
void leds(int on)               // all 3 kit LEDs on or off
{
    LATHbits.LATH0=on;          // LED1
    LATHbits.LATH1=on;          // LED2
    LATHbits.LATH2=on;          // LED3
}

// ---------------------------------------------------------------------------------------------
// void stop_forever(void)
//   Stops every motor and waits forever (press RESET to start again).
// ---------------------------------------------------------------------------------------------
void stop_forever(void)
{
    set_all(STOP_TICKS);
    leds(0);
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
//   Program entry: arms all 4 ESCs, then ramps all 4 to the same pulse together, spins 4 s,
//   stops 2 s, repeats.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    int ticks;

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

    // SW1 = stop, LEDs = spinning
    ANSELBbits.ANSB12=0;
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;        // pull-up: SW1 reads 1 until pressed
    TRISHbits.TRISH0=0;
    TRISHbits.TRISH1=0;
    TRISHbits.TRISH2=0;
    leds(0);

    // 1. arm: 1 ms to all 4 ESCs for 3 s
    wait_check(3000);

    // 2. all 4 together, forever
    while (1){
        leds(1);
        for (ticks=STOP_TICKS;ticks<=SPIN_TICKS;ticks=ticks+5){    // gentle start, all 4 together
            set_all(ticks);
            wait_check(25);
        }
        wait_check(4000);               // all 4 spin at the same pulse for 4 s

        set_all(STOP_TICKS);            // all 4 stop
        leds(0);
        wait_check(2000);               // 2 s pause, then again
    }
    return 0;
}
