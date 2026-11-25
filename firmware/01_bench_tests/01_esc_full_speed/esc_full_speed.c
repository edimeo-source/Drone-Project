/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/01_esc_full_speed/esc_full_speed.c
 *  What    : ESC full-speed test (2 motors) + ESC calibration mode
 *  Version : bench test   |   Date: 2026-11-12
 *  Status  : WORKED on the seesaw: both motors armed and ran to full speed.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC red 5 V wires . NOT connected
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): delay_ms(), set_both(),
 *    sw1_pressed(), stop_forever(), calibrate_escs(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz

// =====================================================================
// ESC FULL SPEED TEST - both motors, hard coded
// Ethan DiMeo
//
// ESC 1 (FL): signal -> J12 pin 11 (RD0 = OC1)   ground -> J12 pin 14
// ESC 2 (FR): signal -> J12 pin 12 (RF2 = OC2)   ground -> J12 pin 20
// ESC red 5 V wires: NOT connected.  ESC power: 12 V bench supply.
//
// What it does:
//   1. 1 ms pulses for 3 s     -> ESCs arm (beeps)          LED1 off
//   2. ramps 1 ms -> 2 ms in 3 s (no sudden jump)          LED1 on
//   3. holds 2 ms = FULL SPEED forever
//   SW1 on the kit = STOP: both motors back to 1 ms until you press reset.
//
// ESC CALIBRATION MODE (do once, so both ESCs start at the same pulse):
//   PROPS OFF. 12 V supply OFF.
//   1. Hold SW1 and press RESET (keep holding SW1 ~1 s)  -> LED1 + LED2 on
//      The PIC now sends 2 ms (max) to both ESCs.
//   2. Turn the 12 V supply ON. ESCs play a tune, then beep-beep (max learned).
//   3. Press SW1.  The PIC sends 1 ms (min). ESCs beep (min learned) + long beep.
//      LED1 off, LED2 stays on = calibration done.
//   4. Press RESET (without SW1) to run the normal full-speed test.
//
// PROPS ON = the seesaw will push hard. Clamp it, safety glasses, hands clear.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms = 50 Hz
//   1 ms = 938 ticks (stop)      2 ms = 1875 ticks (full speed)
// =====================================================================

#define STOP_TICKS  938         // 1.0 ms
#define FULL_TICKS  1875        // 2.0 ms

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
        WDTCONbits.WDTCLRKEY=0x5743;    // clear the watchdog (config.bits turns it on)
    }
}

// ---------------------------------------------------------------------------------------------
// void set_both(int ticks)
//   Sets the same pulse (in timer ticks) on ESC 1 and ESC 2.
// ---------------------------------------------------------------------------------------------
void set_both(int ticks)
{
    OC1RS=ticks;                // ESC 1, pin 11
    OC2RS=ticks;                // ESC 2, pin 12
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
// void stop_forever(void)
//   Stops every motor and waits forever (press RESET to start again).
// ---------------------------------------------------------------------------------------------
void stop_forever(void)
{
    set_both(STOP_TICKS);
    LATHbits.LATH0=0;           // LED1 off
    while(1) WDTCONbits.WDTCLRKEY=0x5743;
}

// ---------------------------------------------------------------------------------------------
// void calibrate_escs(void)
//   ESC calibration: sends 2 ms (max) until SW1 is pressed, then 1 ms (min), so both ESCs learn
//   the range.
// ---------------------------------------------------------------------------------------------
// ESC throttle calibration: show the ESCs max (2 ms), then min (1 ms)
void calibrate_escs(void)
{
    set_both(FULL_TICKS);       // max - the ESCs must be powered AFTER this
    LATHbits.LATH0=1;           // LED1 + LED2 on = "sending max, turn 12 V on now"
    LATHbits.LATH1=1;

    while (sw1_pressed()) delay_ms(10);     // wait for you to let go of SW1
    delay_ms(200);                          // ignore button bounce
    while (!sw1_pressed()) delay_ms(10);    // 12 V on, beep-beep... then press SW1

    set_both(STOP_TICKS);       // min
    LATHbits.LATH0=0;           // LED1 off, LED2 on = done
    while(1) WDTCONbits.WDTCLRKEY=0x5743;
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: arms both ESCs at 1 ms, ramps to 2 ms (full) and holds it; hold SW1 at reset
//   for ESC calibration.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    int ticks;

    T3CON=0x8070;               // Timer3 ON, 1:256 (delays)

    // ESC outputs
    TRISDbits.TRISD0=0;
    RPD0R=12;                   // RD0 = OC1 (J12 pin 11)
    TRISFbits.TRISF2=0;
    RPF2R=11;                   // RF2 = OC2 (J12 pin 12)

    T2CON=0x0000;
    TMR2=0;
    PR2=18749;                  // 20 ms
    OC1R=STOP_TICKS;
    OC1RS=STOP_TICKS;
    OC2R=STOP_TICKS;
    OC2RS=STOP_TICKS;
    OC1CON=0x8006;              // PWM mode, Timer2
    OC2CON=0x8006;
    T2CON=0x8060;               // Timer2 ON, 1:64

    // SW1 = stop, LED1 = motors running
    ANSELBbits.ANSB12=0;
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;
    TRISHbits.TRISH0=0;
    LATHbits.LATH0=0;
    TRISHbits.TRISH1=0;         // LED2 = calibration mode
    LATHbits.LATH1=0;

    delay_ms(20);               // let the pull-up settle before reading SW1
    if (sw1_pressed()) calibrate_escs();    // SW1 held at reset = calibration mode

    // 1. arm
    delay_ms(3000);

    // 2. ramp up: 10 ticks every 30 ms, 938 -> 1875 takes about 2.8 s
    LATHbits.LATH0=1;
    for (ticks=STOP_TICKS; ticks<=FULL_TICKS; ticks=ticks+10){
        set_both(ticks);
        delay_ms(30);
        if (sw1_pressed()) stop_forever();
    }

    // 3. full speed
    set_both(FULL_TICKS);
    while (1){
        WDTCONbits.WDTCLRKEY=0x5743;
        if (sw1_pressed()) stop_forever();
    }
    return 0;
}
