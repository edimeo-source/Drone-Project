/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/13_esc_calibrate_relay/esc_calibrate_relay.c
 *  What    : ESC throttle calibration through the motor-power relay - all 4 ESCs at once
 *  Version : bench test   |   Date: 2026-11-25
 *  Status  : New Nov 25: 07_esc_calibrate with the relay added (2.0 ms first, then RK4 closes
 *            the relay). Checked in a PC simulation (8 checks). Not yet run on the board. Props
 *            OFF.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): delay_ms(), initU2(), putU2(),
 *    putsU2(), set_all(), show_step(), sw1_pressed(), sw2_pressed(), stop_forever(),
 *    wait_check(), wait_sw2(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz, watchdogs off (same file as SewSaw)

// =====================================================================
// ESC THROTTLE CALIBRATION WITH THE MOTOR-POWER RELAY - all 4 ESCs at the same time
// Ethan DiMeo, Nov 25 2026
//
// Same calibration as 07_esc_calibrate (Nov 18), for the drone as it is now: the battery
// reaches the ESCs only through the relay (J12 pin 33 = RK4, HIGH = motor power on).
// So the battery can stay plugged in: the board sends 2.0 ms FIRST, then closes the relay,
// and the ESCs power up already seeing full throttle - that is what starts calibration.
//
// Why: in the spin test ESC 2 and ESC 3 needed a longer pulse to start than ESC 1 and 4,
// so the 4 ESCs don't agree on what the pulses mean. Calibrating teaches every ESC the
// same range: 1.0 ms = stop, 2.0 ms = full. After that the same pulse = the same speed.
// The ESCs save it, so this only has to be done once.
//
// BEFORE YOU START
//   - PROPS OFF (2.0 ms is full throttle if an ESC doesn't go into calibration).
//   - Battery plugged in is fine: the relay keeps the ESCs off until step 2.
//     The kit can run from the battery (7805 -> J12 pin 2) or the PC USB.
//   - Wiring as on the drone: ESC signals J12 pins 11, 12, 21, 19 (grounds 14, 20, 25, 30),
//     relay CH1 on J12 pin 33.
//   - PuTTY on the kit's serial port, 115200 baud: every step prints there (the LEDs show it too).
//
// STEPS (SW2 = next step)
//   1. LED1  No pulses, relay OFF.                    -> press SW2
//   2. LED2  Board sends 2.0 ms (full) to all 4, then 0.5 s later closes the relay.
//            ESCs play their power-up tune, then 2 short beeps (full throttle saved)
//                                                    -> press SW2 RIGHT AWAY (within 2 s)
//   3. LED3  Board sends 1.0 ms (stop).
//            ESCs beep (usually 3 beeps for a 3S battery) then 1 long beep = range saved.
//   4. Test: SW2 -> all 4 ramp up to 1.10 ms together, spin 3 s, stop. SW2 again = repeat.
//
//   SW1 = STOP at any time: 1.0 ms to all 4 and the relay OFF (motor power cut), then
//   nothing until you press RESET.
//   Never press SW3 (same pin as the serial transmit).
//   If an ESC plays a long tune instead of the beeps in step 3, it went into its
//   programming mode (2.0 ms held too long): press SW1 (relay off), RESET, start over.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms = 50 Hz
//   1.00 ms = 938 ticks (stop)    1.10 ms = 1031 ticks (test)    2.00 ms = 1875 ticks (full)
// =====================================================================

#define NO_PULSE    0           // OCxRS = 0 -> pin stays low, no pulse at all
#define MIN_TICKS   938         // 1.00 ms = stop
#define MAX_TICKS   1875        // 2.00 ms = full
#define TEST_TICKS  1031        // 1.10 ms = slow test spin

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

// ---------- serial (UART2 to PuTTY through the kit USB, transmit only) ----------
// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
void initU2(void)
{
    TRISBbits.TRISB14=0;        // RB14 = U2TX
    RPB14R=2;                   // U2TX -> RB14
    U2BRG=129;                  // 60 MHz / (4 x 130) = 115,384 baud (BRGH = 1)
    U2MODE=0x8008;              // UART ON, BRGH = 1, 8 bits, no parity, 1 stop bit
    U2STA=0x0400;               // transmitter ON
}

// ---------------------------------------------------------------------------------------------
// void putU2(char c)
//   Sends one character out UART2 to the PC (waits while the transmit buffer is full).
// ---------------------------------------------------------------------------------------------
void putU2(char c)
{
    while (U2STAbits.UTXBF);    // wait while the send buffer is full
    U2TXREG=c;
}

// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Sends a text line to the PC over UART2, followed by a new line.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s)            // send a line
{
    while (*s){
        putU2(*s++);
    }
    putU2('\r');
    putU2('\n');
}

// ---------- ESCs, LEDs, buttons ----------
// ---------------------------------------------------------------------------------------------
// void set_all(int ticks)
//   Sets the same pulse (in timer ticks) on all 4 ESCs, limited to 1.0-2.0 ms.
// ---------------------------------------------------------------------------------------------
void set_all(int ticks)         // same pulse to all 4 ESCs
{
    OC1RS=ticks;                // ESC 1, pin 11, front-left
    OC2RS=ticks;                // ESC 2, pin 12, front-right
    OC3RS=ticks;                // ESC 3, pin 21, back-left
    OC4RS=ticks;                // ESC 4, pin 19, back-right
}

// ---------------------------------------------------------------------------------------------
// void show_step(int n)
//   Shows the calibration step number on LED1-LED3.
// ---------------------------------------------------------------------------------------------
void show_step(int n)           // LED1, LED2 or LED3 = step 1, 2 or 3 (0 = all off)
{
    LATHbits.LATH0=(n==1);
    LATHbits.LATH1=(n==2);
    LATHbits.LATH2=(n==3);
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
// int sw2_pressed(void)
//   1 while SW2 is held (SW2 pulls RB13 low).
// ---------------------------------------------------------------------------------------------
int sw2_pressed(void)
{
    return PORTBbits.RB13==0;   // SW2 pulls RB13 low
}

// ---------------------------------------------------------------------------------------------
// void stop_forever(void)
//   SW1: all 4 ESCs to 1.0 ms, relay off (motor power cut), then waits until RESET.
// ---------------------------------------------------------------------------------------------
void stop_forever(void)         // SW1: motors to 1.0 ms, relay off, wait for RESET
{
    set_all(MIN_TICKS);
    LATKbits.LATK4=0;           // RK4 LOW = relay off = motor power CUT
    show_step(0);
    putsU2("STOPPED (SW1). All 4 at 1.0 ms, relay OFF. Press RESET to start over.");
    while(1){
        delay_ms(100);          // nothing more until RESET
    }
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
// void wait_sw2(void)
//   Waits for a full SW2 press and release (debounced); SW1 still stops everything meanwhile.
// ---------------------------------------------------------------------------------------------
void wait_sw2(void)             // wait for one SW2 press (and its release)
{
    while (!sw2_pressed()){
        if (sw1_pressed()) stop_forever();
    }
    delay_ms(30);               // let the button stop bouncing
    while (sw2_pressed()){
        if (sw1_pressed()) stop_forever();
    }
    delay_ms(30);
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: relay off, then SW2 steps: 2.0 ms, relay on (ESCs power up seeing full), 1.0
//   ms, test spin.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    int ticks;

    T3CON=0x8070;               // Timer3 ON, 1:256 (delays)

    // LED1-3 = step, SW1 = stop, SW2 = next
    TRISHbits.TRISH0=0;
    TRISHbits.TRISH1=0;
    TRISHbits.TRISH2=0;
    show_step(0);
    ANSELBbits.ANSB12=0;        // SW1 (RB12) and SW2 (RB13): digital inputs,
    ANSELBbits.ANSB13=0;        // pressed = 0, so they need the pull-ups
    TRISBbits.TRISB12=1;
    TRISBbits.TRISB13=1;
    CNPUBbits.CNPUB12=1;
    CNPUBbits.CNPUB13=1;

    initU2();

    // Motor-power relay on RK4 (J12 pin 33): latch LOW first, THEN make the pin an output,
    // so it never goes high for a moment (same order as Drone.X and the relay test).
    LATKbits.LATK4=0;
    TRISKbits.TRISK4=0;

    // ESC outputs (peripheral pin select, same as the spin test and Drone.X)
    TRISDbits.TRISD0=0;
    RPD0R=12;                   // RD0 = OC1 -> J12 pin 11 (ESC 1)
    TRISFbits.TRISF2=0;
    RPF2R=11;                   // RF2 = OC2 -> J12 pin 12 (ESC 2)
    TRISFbits.TRISF4=0;
    RPF4R=11;                   // RF4 = OC3 -> J12 pin 21 (ESC 3)
    TRISFbits.TRISF5=0;
    RPF5R=11;                   // RF5 = OC4 -> J12 pin 19 (ESC 4)

    // Timer2 + 4 output compares in PWM mode, starting with NO pulse
    T2CON=0x0000;
    TMR2=0;
    PR2=18749;                  // 20 ms
    OC1R=NO_PULSE;
    OC1RS=NO_PULSE;
    OC2R=NO_PULSE;
    OC2RS=NO_PULSE;
    OC3R=NO_PULSE;
    OC3RS=NO_PULSE;
    OC4R=NO_PULSE;
    OC4RS=NO_PULSE;
    OC1CON=0x8006;              // ON, PWM mode, uses Timer2
    OC2CON=0x8006;
    OC3CON=0x8006;
    OC4CON=0x8006;
    T2CON=0x8060;               // Timer2 ON, 1:64

    // ---- step 1: nothing yet ----
    show_step(1);
    putsU2("");
    putsU2("===== ESC CALIBRATION (all 4, through the relay) =====");
    putsU2("PROPS OFF.  Battery may stay plugged in: the relay is OFF until step 2.");
    putsU2("SW2 = next step    SW1 = STOP    never press SW3");
    putsU2("");
    putsU2("Step 1 (LED1): no pulses yet. Press SW2 to start sending 2.0 ms.");
    wait_sw2();

    // ---- step 2: full throttle signal, THEN the relay gives the ESCs battery power ----
    set_all(MAX_TICKS);
    show_step(2);
    putsU2("");
    putsU2("Step 2 (LED2): sending 2.0 ms (FULL) to all 4.");
    wait_check(500);            // the ESCs must see 2.0 ms before they get power
    LATKbits.LATK4=1;           // RK4 HIGH = relay on = battery to the ESCs
    putsU2("  Relay ON: the ESCs are powering up now.");
    putsU2("  Wait for the power-up tune, then 2 short beeps.");
    putsU2("  Right after the 2 beeps, press SW2 (within 2 s).");
    wait_sw2();

    // ---- step 3: stop signal, ESCs save the range ----
    set_all(MIN_TICKS);
    show_step(3);
    putsU2("");
    putsU2("Step 3 (LED3): sending 1.0 ms (stop).");
    putsU2("  Listen: a few beeps (3 for a 3S battery), then 1 long beep = range saved.");
    putsU2("  All 4 ESCs must do it. A long tune instead = programming mode:");
    putsU2("  press SW1 (relay off), then RESET, and start over.");
    putsU2("");
    putsU2("Press SW2 for a test spin (all 4 together, slow, 3 s).");

    // ---- step 4: test spin, as many times as you want ----
    while (1){
        wait_sw2();
        putsU2("Test: all 4 ramp to 1.10 ms ...");
        for (ticks=MIN_TICKS;ticks<=TEST_TICKS;ticks=ticks+3){     // gentle start, about 1 s
            set_all(ticks);
            wait_check(30);
        }
        wait_check(3000);                                           // spin 3 s
        set_all(MIN_TICKS);
        putsU2("  Stopped. All 4 should have started together at about the same speed.");
        putsU2("  SW2 = test again    SW1 = stop");
    }
    return 0;
}
