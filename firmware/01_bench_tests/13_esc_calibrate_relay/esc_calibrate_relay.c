/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/13_esc_calibrate_relay/esc_calibrate_relay.c
 *  What    : ESC throttle calibration from PuTTY, through the motor-power relay - all 4 ESCs at
 *            once, no kit buttons
 *  Version : bench test   |   Date: 2026-11-25
 *  Status  : New Nov 25: 07_esc_calibrate redone for the assembled drone: PuTTY commands
 *            instead of SW1/SW2, and the relay is closed by the program (2.0 ms first, then
 *            RK4). Checked in a PC simulation (26 checks). Not yet run on the board. Props OFF.
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
 *    putsU2(), printU2(), getU2(), us_to_ticks(), initMotors(), write_pulses(), set_all_now(),
 *    all_stop(), set_relay(), show_step(), stop_all_relay_off(), is_stop_key(), service(),
 *    wait_or_stop(), relay_on_and_arm(), read_line(), read_int(), calibrate(), test_spin(),
 *    help(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz (same file as every other program)

// =====================================================================
// ESC THROTTLE CALIBRATION FROM PUTTY - all 4 ESCs at once, through the motor-power relay
// Ethan DiMeo, Nov 25 2026
//
// For the drone fully assembled: no kit buttons needed. Everything is typed in PuTTY.
// Calibration teaches every ESC the same range: 1.0 ms = stop, 2.0 ms = full.
// After that the same pulse asks every ESC for the same throttle. The ESCs save it,
// so this only has to be done once. (Bench test 11 then matches the motors' thrust.)
//
// The battery reaches the ESCs only through the relay (J12 pin 33 = RK4, HIGH = power on).
// So the battery stays plugged in: the board sends 2.0 ms FIRST, then closes the relay,
// and the ESCs power up already seeing full throttle - that is what starts calibration.
//
// BEFORE YOU START
//   - PROPS OFF. 2.0 ms is full throttle if an ESC doesn't go into calibration.
//   - Battery plugged in (the 7805 powers the kit, the relay keeps the ESCs off).
//   - USB from the PC to the kit for PuTTY: 115200 baud, 8N1.
//
// Commands (type, then Enter):
//   CAL         calibrate all 4 (asks Y to confirm props are off):
//                 2.0 ms -> relay ON -> power-up tune + 2 short beeps
//                 -> YOU press Enter right after the 2 beeps -> 1.0 ms
//                 -> a few beeps + 1 long beep = range saved
//   T           slow test spin: all 4 ramp to 1100 us, 3 s, stop   (T 1150 = other pulse, 1050-1200)
//   S           stop all (1000 us; Enter alone also stops)
//   OFF / ON    relay off / on (ON arms the ESCs at 1000 us for 4 s)
//   ?           help
//   Esc, Space or X while a step is waiting or a test spin is running = stop all + relay OFF.
//   SW1 on the kit (if you can reach it) = stop all + relay OFF, any time.
//
// Wiring (J12 pinout, drone v2.0 and later):
//   ESC 1 FL front-left : signal pin 11 (RD0 = OC1)   ground pin 14
//   ESC 2 FR front-right: signal pin 12 (RF2 = OC2)   ground pin 20
//   ESC 3 RL rear-left  : signal pin 21 (RF4 = OC3)   ground pin 25
//   ESC 4 RR rear-right : signal pin 19 (RF5 = OC4)   ground pin 30
//   Motor-power relay   : pin 33 (RK4), HIGH = motor power on
//   PuTTY: the kit's USB serial port (J11), 115200 baud, 8N1. Never wire pins 8 and 10.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms = 50 Hz
//   ticks = us x 0.9375:  1000 us = 938 ticks (stop)   1100 us = 1031   2000 us = 1875 (full)
// =====================================================================

#define STOP_US     1000.0f     // 1.00 ms = motor off
#define FULL_US     2000.0f     // 2.00 ms = full throttle (only during CAL step 2)
#define TEST_US     1100        // test spin pulse
#define TEST_MIN    1050        // T accepts 1050-1200 us (props off)
#define TEST_MAX    1200
#define RAMP_STEP   0.8f        // +0.8 us every 4 ms = 200 us per second (slow start)
#define ARM_MS      4000        // after the relay closes: 1 ms pulses for 4 s so the ESCs arm
#define PRE_MS      500         // 2.0 ms goes out this long BEFORE the relay closes
#define DEAF_MS     1000        // after the relay closes, Enter is ignored this long (too early)
#define CAL_WAIT_MS 20000       // no Enter within 20 s -> abort (never hold 2.0 ms for long)
#define POWERDOWN_MS 3000       // relay off before a new CAL: let the ESCs power down fully
#define SPIN_MS     3000        // test spin: time at the test pulse

float now_us[5]={0, STOP_US, STOP_US, STOP_US, STOP_US};    // pulse each ESC gets now (index 1-4)
float want_us[5]={0, STOP_US, STOP_US, STOP_US, STOP_US};   // pulse it is ramping to
int relay_on=0;
int stopped=0;                  // 1 = Esc/Space/X or SW1 stopped everything

// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
// ---------------- delays: Timer3, 60 MHz / 256 = 234,375 counts per second -> 234 counts = 1 ms
void delay_ms(int n)
{
    for (int i=0;i<n;i++){
        TMR3=0;
        while(TMR3<234);
    }
}

// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
// ---------------- serial (UART2 to PuTTY through the kit USB)
void initU2(void)
{
    ANSELGbits.ANSG6=0;         // RG6 = U2RX: digital input
    TRISGbits.TRISG6=1;
    U2RXR=1;                    // U2RX <- RG6
    TRISBbits.TRISB14=0;        // RB14 = U2TX
    RPB14R=2;                   // U2TX -> RB14
    U2BRG=129;                  // 60 MHz / (4 x 130) = 115,384 baud (BRGH = 1)
    U2MODE=0x8008;              // UART ON, BRGH = 1, 8 bits, no parity, 1 stop bit
    U2STA=0x1400;               // receiver + transmitter ON
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

// ---------------------------------------------------------------------------------------------
// void printU2(char *s)
//   Sends text to PuTTY without a new line (for questions).
// ---------------------------------------------------------------------------------------------
void printU2(char *s)           // send text, no new line (prompts)
{
    while (*s){
        putU2(*s++);
    }
}

// ---------------------------------------------------------------------------------------------
// int getU2(void)
//   Returns a key typed in PuTTY, or -1 if none (polled, never waits).
// ---------------------------------------------------------------------------------------------
int getU2(void)                 // a key from PuTTY, or -1 if none (never waits)
{
    if (U2STAbits.OERR) U2STAbits.OERR=0;
    if (!U2STAbits.URXDA) return -1;
    return U2RXREG;
}

// ---------------------------------------------------------------------------------------------
// int us_to_ticks(float us)
//   Converts a pulse in microseconds to Timer2 ticks (0.9375 ticks per us, rounded).
// ---------------------------------------------------------------------------------------------
// ---------------- ESCs
int us_to_ticks(float us)
{
    return (int)(us*0.9375f+0.5f);      // 937,500 ticks per second = 0.9375 ticks per us
}

// ---------------------------------------------------------------------------------------------
// void initMotors(void)
//   Sets up Timer2 (1:64, 50 Hz) and OC1-OC4 on pins 11/12/21/19, every ESC at 1000 us.
// ---------------------------------------------------------------------------------------------
void initMotors(void)
{
    TRISDbits.TRISD0=0;
    RPD0R=12;                   // RD0 = OC1 -> J12 pin 11 (ESC 1 FL)
    TRISFbits.TRISF2=0;
    RPF2R=11;                   // RF2 = OC2 -> J12 pin 12 (ESC 2 FR)
    TRISFbits.TRISF4=0;
    RPF4R=11;                   // RF4 = OC3 -> J12 pin 21 (ESC 3 RL)
    TRISFbits.TRISF5=0;
    RPF5R=11;                   // RF5 = OC4 -> J12 pin 19 (ESC 4 RR)

    T2CON=0x0000;
    TMR2=0;
    PR2=18749;                  // 20 ms
    OC1R=938;                   // every ESC starts at 1000 us = stop
    OC1RS=938;
    OC2R=938;
    OC2RS=938;
    OC3R=938;
    OC3RS=938;
    OC4R=938;
    OC4RS=938;
    OC1CON=0x8006;              // ON, PWM mode, uses Timer2
    OC2CON=0x8006;
    OC3CON=0x8006;
    OC4CON=0x8006;
    T2CON=0x8060;               // Timer2 ON, 1:64
}

// ---------------------------------------------------------------------------------------------
// void write_pulses(void)
//   Copies the 4 current pulses into OC1RS-OC4RS (ESC 1-4).
// ---------------------------------------------------------------------------------------------
void write_pulses(void)         // copy now_us[] into the 4 output compares
{
    OC1RS=us_to_ticks(now_us[1]);
    OC2RS=us_to_ticks(now_us[2]);
    OC3RS=us_to_ticks(now_us[3]);
    OC4RS=us_to_ticks(now_us[4]);
}

// ---------------------------------------------------------------------------------------------
// void set_all_now(float us)
//   Sets all 4 ESCs to the same pulse at once, no ramp (2.0 ms or 1.0 ms during CAL).
// ---------------------------------------------------------------------------------------------
void set_all_now(float us)      // all 4 ESCs to this pulse right now (no ramp)
{
    int n;
    for (n=1;n<=4;n++){
        now_us[n]=us;
        want_us[n]=us;
    }
    write_pulses();
}

// ---------------------------------------------------------------------------------------------
// void all_stop(void)
//   Every motor to 1000 us right now (the relay stays as it is).
// ---------------------------------------------------------------------------------------------
void all_stop(void)             // every motor to 1000 us right now
{
    set_all_now(STOP_US);
}

// ---------------------------------------------------------------------------------------------
// void set_relay(int on)
//   Opens (0) or closes (1) the motor-power relay on RK4, J12 pin 33.
// ---------------------------------------------------------------------------------------------
// ---------------- relay (J12 pin 33 = RK4) and the kit's LEDs / SW1
void set_relay(int on)
{
    relay_on=on;
    LATKbits.LATK4=on;          // HIGH = relay closed = battery to the ESCs
}

// ---------------------------------------------------------------------------------------------
// void show_step(int n)
//   Shows the calibration step number on LED1-LED3.
// ---------------------------------------------------------------------------------------------
void show_step(int n)           // LED1, LED2 or LED3 = CAL step 1, 2 or 3 (0 = all off)
{
    LATHbits.LATH0=(n==1);
    LATHbits.LATH1=(n==2);
    LATHbits.LATH2=(n==3);
}

// ---------------------------------------------------------------------------------------------
// void stop_all_relay_off(char *why)
//   The emergency stop (Esc/Space/X or SW1): all 4 at 1000 us, relay open, prints why.
// ---------------------------------------------------------------------------------------------
void stop_all_relay_off(char *why)  // the emergency stop: 1000 us, relay open
{
    all_stop();
    set_relay(0);
    show_step(0);
    stopped=1;
    putsU2("");
    printU2(why);
    putsU2(": all 4 at 1000 us, relay OFF (motor power cut).");
}

// ---------------------------------------------------------------------------------------------
// int is_stop_key(int c)
//   1 if the key is Esc, Space or X: the keys that stop everything.
// ---------------------------------------------------------------------------------------------
int is_stop_key(int c)          // Esc, Space or X = stop everything
{
    return c==27 || c==' ' || c=='X' || c=='x';
}

// ---------------------------------------------------------------------------------------------
// void service(void)
//   One 4 ms step: ramps each motor toward its target (up 200 us/s, down at once) and checks
//   SW1 (stop all + relay off).
// ---------------------------------------------------------------------------------------------
// One 4 ms step: ramp every motor toward its target, check SW1.
void service(void)
{
    int n;
    delay_ms(4);
    for (n=1;n<=4;n++){
        if (want_us[n]<now_us[n]) now_us[n]=want_us[n];            // slowing down: right away
        else if (now_us[n]<want_us[n]){                             // speeding up: slowly
            now_us[n]=now_us[n]+RAMP_STEP;
            if (now_us[n]>want_us[n]) now_us[n]=want_us[n];
        }
    }
    write_pulses();
    if (PORTBbits.RB12==0 && (relay_on || now_us[1]>STOP_US || now_us[2]>STOP_US ||
                              now_us[3]>STOP_US || now_us[4]>STOP_US)){   // SW1 = STOP
        stop_all_relay_off("SW1");
    }
}

// ---------------------------------------------------------------------------------------------
// int wait_or_stop(int ms)
//   Waits ms milliseconds; Esc/Space/X or SW1 stops everything (0 = stopped). Other keys are
//   dropped.
// ---------------------------------------------------------------------------------------------
// Wait ms milliseconds. Esc/Space/X (or SW1) stops everything. 1 = waited, 0 = stopped.
// Other keys are thrown away (nothing should be typed during these waits).
int wait_or_stop(int ms)
{
    int i, c;
    for (i=0;i<ms;i=i+4){
        service();
        if (stopped) return 0;
        c=getU2();
        if (c>=0 && is_stop_key(c)){
            stop_all_relay_off("STOP key");
            return 0;
        }
    }
    return 1;
}

// ---------------------------------------------------------------------------------------------
// void relay_on_and_arm(void)
//   Closes the relay with every ESC at 1000 us, then waits 4 s so the ESCs arm before anything
//   spins.
// ---------------------------------------------------------------------------------------------
void relay_on_and_arm(void)     // relay on with 1000 us so the ESCs arm normally (no calibration)
{
    if (relay_on) return;
    all_stop();                 // the ESCs must see 1000 us when their power comes on
    set_relay(1);
    putsU2("Relay ON: ESCs powering up at 1000 us - wait for the beeps (4 s)...");
    if (wait_or_stop(ARM_MS)) putsU2("ESCs armed.");
}

// ---------------------------------------------------------------------------------------------
// int read_line(char *line, int max)
//   Waits for one typed line (echoed, uppercase, backspace works) while SW1 keeps working. 0 =
//   SW1.
// ---------------------------------------------------------------------------------------------
// Wait for one typed line (echoed), uppercase. The ramps and SW1 keep working meanwhile.
// 1 = got a line, 0 = SW1 was pressed.
int read_line(char *line, int max)
{
    int len=0, c;
    while (1){
        service();
        if (stopped) return 0;
        while ((c=getU2())>=0){
            if (c=='\r' || c=='\n'){
                if (c=='\n' && len==0) continue;    // the LF after a CR
                line[len]=0;
                putsU2("");
                return 1;
            }
            if ((c==8 || c==127) && len>0){     // backspace
                len--;
                putU2(8);
                putU2(' ');
                putU2(8);
            }
            else if (c>=' ' && len<max-1){
                if (c>='a' && c<='z') c=c-32;   // uppercase
                line[len++]=c;
                putU2(c);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// int read_int(char *s, int *v)
//   Reads the first whole number in a line. 1 = found one.
// ---------------------------------------------------------------------------------------------
int read_int(char *s, int *v)   // first whole number in s -> *v. 1 = found one
{
    int n=0, found=0;
    while (*s && (*s<'0' || *s>'9')) s++;
    while (*s>='0' && *s<='9'){
        n=n*10+(*s-'0');
        found=1;
        s++;
    }
    *v=n;
    return found;
}

// ---------------------------------------------------------------------------------------------
// void calibrate(void)
//   CAL: asks Y (props off), sends 2.0 ms, closes the relay, waits for Enter after the 2 beeps,
//   then 1.0 ms.
// ---------------------------------------------------------------------------------------------
// ---------------- the calibration
void calibrate(void)
{
    char line[8];
    int i, c, got_enter;

    putsU2("");
    putsU2("CAL sends 2.0 ms = FULL THROTTLE. PROPS MUST BE OFF.");
    printU2("Props off? Type Y then Enter to start (anything else cancels): ");
    if (!read_line(line, 8)) return;
    if (line[0]!='Y'){
        putsU2("Cancelled. Nothing changed.");
        return;
    }

    if (relay_on){              // the ESCs must power up fresh, so turn them off first
        all_stop();
        set_relay(0);
        putsU2("Relay OFF: letting the ESCs power down (3 s)...");
        if (!wait_or_stop(POWERDOWN_MS)) return;
    }

    // ---- step 2: full throttle signal, THEN the relay gives the ESCs battery power ----
    show_step(2);
    set_all_now(FULL_US);
    putsU2("Step 2: sending 2.0 ms (FULL) to all 4.  Esc / Space / X = abort.");
    if (!wait_or_stop(PRE_MS)) return;          // the ESCs must see 2.0 ms before they get power
    set_relay(1);
    putsU2("  Relay ON: the ESCs are powering up now.");
    putsU2("  Listen: power-up tune, then 2 short beeps.");
    putsU2("  Press Enter RIGHT AFTER the 2 beeps (within 2 s).");

    got_enter=0;
    for (i=0;i<CAL_WAIT_MS && !got_enter;i=i+4){
        service();
        if (stopped) return;
        c=getU2();
        if (c<0) continue;
        if (is_stop_key(c)){
            stop_all_relay_off("Aborted");
            return;
        }
        if ((c=='\r' || c=='\n') && i>=DEAF_MS) got_enter=1;   // earlier = a leftover key, ignored
    }
    if (!got_enter){
        stop_all_relay_off("No Enter within 20 s, aborted");
        putsU2("Type CAL to try again.");
        return;
    }

    // ---- step 3: stop signal, the ESCs save the range ----
    all_stop();
    show_step(3);
    putsU2("Step 3: sending 1.0 ms (stop).");
    putsU2("  Listen: a few beeps (3 for a 3S battery), then 1 long beep = range saved.");
    putsU2("  All 4 ESCs must do it. A long tune instead = programming mode:");
    putsU2("  type OFF, then CAL to start over.");
    putsU2("Then type T for a slow test spin (all 4 together at 1100 us).");
}

// ---------------------------------------------------------------------------------------------
// void test_spin(int us)
//   T: arms if needed, ramps all 4 to the test pulse, holds 3 s, stops. Any key stops it.
// ---------------------------------------------------------------------------------------------
// ---------------- test spin: all 4 together, slow, props off
void test_spin(int us)
{
    char s[80];
    int i, c;

    relay_on_and_arm();
    if (!relay_on) return;                      // stopped while arming
    show_step(0);
    sprintf(s, "Test: all 4 ramp to %d us (%d ticks), %d s, then stop.  Any key = stop.",
            us, us_to_ticks(us), SPIN_MS/1000);
    putsU2(s);
    for (i=1;i<=4;i++) want_us[i]=us;
    for (i=0;i<1000+SPIN_MS;i=i+4){             // ramp (up to 1 s), then spin
        service();
        if (stopped) return;
        c=getU2();
        if (c>=0){
            all_stop();
            if (is_stop_key(c)) stop_all_relay_off("STOP key");
            else putsU2("  Stopped (key pressed). Relay still on.");
            return;
        }
    }
    all_stop();
    putsU2("  Stopped. All 4 should have started together at about the same speed.");
}

// ---------------------------------------------------------------------------------------------
// void help(void)
//   Prints the commands (CAL, T, S, OFF/ON, ?) and the stop keys.
// ---------------------------------------------------------------------------------------------
void help(void)
{
    putsU2("");
    putsU2("=== ESC CALIBRATION FROM PUTTY (all 4, through the relay) - PROPS OFF ===");
    putsU2("  CAL       calibrate: 2.0 ms, relay on, Enter after the 2 beeps, 1.0 ms");
    putsU2("  T         test spin all 4 at 1100 us for 3 s   (T 1150 = other pulse, 1050-1200)");
    putsU2("  S         stop all (Enter alone too)    OFF / ON = relay off / on    ? = help");
    putsU2("  Esc / Space / X during a step or spin = stop all + relay OFF");
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: relay off, ESCs at 1000 us, UART up, then runs PuTTY lines: CAL, T, S,
//   OFF/ON, ?.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char line[40], s[80];
    int us;

    T3CON=0x8070;               // Timer3 ON, 1:256 (delays)

    // Motor-power relay on RK4 (J12 pin 33): latch LOW first, THEN make the pin an output,
    // so it never goes high for a moment (same order as Drone.X and the relay test).
    LATKbits.LATK4=0;
    TRISKbits.TRISK4=0;
    relay_on=0;

    initMotors();               // all 4 ESCs at 1000 us (no power yet: relay off)

    ANSELBbits.ANSB12=0;        // SW1 = stop (backup, if you can reach it)
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;        // pull-up: SW1 reads 1 until pressed
    TRISHbits.TRISH0=0;         // LED1-3 = CAL step
    TRISHbits.TRISH1=0;
    TRISHbits.TRISH2=0;
    show_step(1);

    initU2();
    delay_ms(100);
    help();
    putsU2("Relay is OFF. Type CAL to start.");

    while (1){
        printU2(">");
        stopped=0;
        if (!read_line(line, 40)) continue;     // SW1 while waiting: already stopped
        if (line[0]=='C') calibrate();
        else if (line[0]=='T'){
            if (!read_int(line+1, &us)) us=TEST_US;     // T alone = 1100 us
            if (us<TEST_MIN || us>TEST_MAX){
                sprintf(s, "%d us is outside %d-%d us. Nothing changed.", us, TEST_MIN, TEST_MAX);
                putsU2(s);
            }
            else test_spin(us);
        }
        else if (line[0]==0 || line[0]=='S' || line[0]=='X'){
            all_stop();
            if (relay_on) putsU2("All motors stopped (1000 us). Relay still on - type OFF to open it.");
            else putsU2("All motors stopped (1000 us). Relay is off.");
        }
        else if (line[0]=='O' && line[1]=='F'){
            all_stop();
            set_relay(0);
            show_step(0);
            putsU2("All motors stopped, relay OFF.");
        }
        else if (line[0]=='O' && line[1]=='N') relay_on_and_arm();
        else if (line[0]=='?' || line[0]=='H') help();
        else putsU2("Not a command (? = help).");
    }
    return 0;
}
