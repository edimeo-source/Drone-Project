/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/11_motor_calibration_test/motor_calibration_test.c
 *  What    : Motor calibration test - kitchen-scale thrust sweep that prints per-motor pulse
 *            trims
 *  Version : bench test   |   Date: 2026-11-25
 *  Status  : New Nov 25. Checked in a PC simulation with 4 deliberately mismatched motors: the
 *            printed trims bring them within 0.4% of each other at every level; R/X/SW1
 *            checked. Not yet run on the board.
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
 *    putsU2(), printU2(), getU2(), fstr(), read_float(), us_to_ticks(), initMotors(),
 *    write_pulses(), all_stop(), set_relay(), show_leds(), service(), wait_service(),
 *    read_line(), ask_reading(), relay_on_and_arm(), measure(), fit_lines(), make_trims(),
 *    trimmed(), print_results(), sweep(), check(), help(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <math.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz (same file as every other program)

// =====================================================================
// MOTOR CALIBRATION TEST - measure each motor's thrust on a kitchen scale and
// work out the pulse trims that make all 4 motors push the same
// Ethan DiMeo
//
// The PIC can't measure speed or thrust by itself, so YOU read a kitchen scale
// and type the number into PuTTY. The program does the rest:
//   1. GO starts the sweep. For each pulse level (1150, 1200 ... 1350 us):
//        - all motors stopped: you type the scale reading (the baseline)
//        - FL, FR, RL, RR spin one at a time at that pulse; you type the reading
//      thrust = baseline - reading  (the motor lifts the drone, so the scale reads less)
//      All 4 motors are measured at one level before the next level, so the
//      battery voltage dropping during the test affects every motor the same way.
//   2. Thrust goes with speed squared, so it fits a straight line to the SQUARE ROOT
//      of each motor's thrust:   sqrt(thrust) = a x (pulse - zero)
//        a    = how strong the motor is (bigger a = more thrust for the same pulse)
//        zero = the pulse where the motor starts to push
//   3. It prints each motor's gain and offset so that
//          trimmed pulse = 1000 + gain x (pulse - 1000) + offset
//      gives every motor the thrust of the AVERAGE motor at the same pulse.
//   4. V 1250 checks it: each motor runs at its trimmed pulse for 1250 us,
//      you type the readings, it prints how far apart the 4 thrusts are.
//
// Commands (type, then Enter):
//   GO          start (or restart) the sweep
//   V 1250      check the trims at 1250 us (after a sweep)
//   T           print the results table again
//   X           stop everything (also aborts a sweep or a check)
//   OFF / ON    relay off / on      ? = help
//   While it asks for a reading: type the number, R = spin this motor again, X = abort.
// SW1 on the kit = stop all + relay off, any time.
//
// SET UP (props ON, the props must push air down):
//   - Run 07_esc_calibrate first, so all 4 ESCs have the same 1.0-2.0 ms range.
//   - FULL battery. Drone centered on the scale (a board on the scale if the legs
//     don't fit), legs fully on it, nothing under the props, nothing touching the drone.
//   - Any unit works (grams, ounces) - use the same one for every reading.
//   - Taring the scale with the drone on it is fine: the readings go negative.
//   - USB cable to the PC must hang loose so it doesn't pull on the drone.
//   - Stand back from the props. Max pulse in this test is 1400 us.
//
// Wiring (J12 pinout, drone v2.0):
//   ESC 1 FL front-left : signal pin 11 (RD0 = OC1)   ground pin 14
//   ESC 2 FR front-right: signal pin 12 (RF2 = OC2)   ground pin 20
//   ESC 3 RL rear-left  : signal pin 21 (RF4 = OC3)   ground pin 25
//   ESC 4 RR rear-right : signal pin 19 (RF5 = OC4)   ground pin 30
//   Motor-power relay   : pin 33 (RK4), HIGH = motor power on
//   PuTTY: the kit's USB serial port (J11), 115200 baud, 8N1. Never wire pins 8 and 10.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second, PR2 = 18749 -> 20 ms (50 Hz)
//   ticks = us x 0.9375:  1000 us = 938 ticks (stop)   1250 us = 1172   1400 us = 1313
// =====================================================================

#define STOP_US     1000.0f     // 1.00 ms = motor off
#define MAX_US      1400.0f     // highest pulse this test will send
#define RAMP_STEP   0.8f        // +0.8 us every 4 ms = 200 us per second (slow start)
#define ARM_MS      4000        // after the relay closes: 1 ms pulses for 4 s so the ESCs arm
#define SETTLE_MS   3000        // after a motor reaches its pulse: let it and the scale settle
#define REST_MS     2000        // after a motor stops: let the scale come back
#define NLEVELS     5
#define MIN_PART    0.03f       // readings under 3% of a motor's biggest one are left out of its line
                                // (the motor wasn't really spinning yet)

float levels[NLEVELS]={1150, 1200, 1250, 1300, 1350};   // pulses measured, us

char *names[5]={"", "FL", "FR", "RL", "RR"};

float now_us[5]={0, STOP_US, STOP_US, STOP_US, STOP_US};    // pulse each ESC gets now (index 1-4)
float want_us[5]={0, STOP_US, STOP_US, STOP_US, STOP_US};   // pulse it is ramping to
int relay_on=0;
int aborted=0;                  // 1 = X or SW1: stop what we're doing

float thrust[5][NLEVELS];       // thrust[motor][level], measured
int have_data=0;                // 1 = a full sweep was done
float slope[5], zero[5];        // sqrt(thrust) = slope x (pulse - zero), per motor
float gain[5], offset[5];       // trimmed = 1000 + gain x (pulse - 1000) + offset
float worst_fit[5];             // biggest distance between a reading and the fitted curve (scale units)
int have_trims=0;

// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
// ---------------- delays: Timer3, 60 MHz / 256 = 234,375 counts per second -> 234 counts = 1 ms
void delay_ms(int n)
{
    int i;
    for (i=0;i<n;i++){
        TMR3=0;
        while(TMR3<234);
    }
}

// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
// ---------------- PuTTY: UART2 at 115200 baud
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
//   Sends one character to PuTTY (waits while the UART's send buffer is full).
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
void putsU2(char *s)            // text + new line
{
    while (*s) putU2(*s++);
    putU2('\r');
    putU2('\n');
}

// ---------------------------------------------------------------------------------------------
// void printU2(char *s)
//   Sends text to PuTTY without a new line (for questions).
// ---------------------------------------------------------------------------------------------
void printU2(char *s)           // text, no new line
{
    while (*s) putU2(*s++);
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
// char *fstr(char *buf, float v, int decimals)
//   Turns a float into text with a fixed number of decimals (XC32's printf may leave out %f).
// ---------------------------------------------------------------------------------------------
// Float to text without %f (XC32 can leave float printing out of printf).
// fstr(buf, -12.345, 2) -> "-12.35"   fstr(buf, 1.0091, 4) -> "1.0091"   0-4 decimals, buf 16 chars.
char *fstr(char *buf, float v, int decimals)
{
    long scale=1, n;
    int i;
    char *sign="";
    for (i=0;i<decimals;i++) scale=scale*10;
    if (v<0){
        sign="-";
        v=-v;
    }
    n=(long)(v*scale+0.5f);
    if (n==0) sign="";
    if (decimals==0) sprintf(buf, "%s%ld", sign, n);
    else if (decimals==1) sprintf(buf, "%s%ld.%01ld", sign, n/scale, n%scale);
    else if (decimals==2) sprintf(buf, "%s%ld.%02ld", sign, n/scale, n%scale);
    else if (decimals==3) sprintf(buf, "%s%ld.%03ld", sign, n/scale, n%scale);
    else sprintf(buf, "%s%ld.%04ld", sign, n/scale, n%scale);   // 4 decimals (the most used here)
    return buf;
}

// ---------------------------------------------------------------------------------------------
// int read_float(char *s, float *v)
//   Reads a typed number such as 1234, -56 or 7.5. 1 = found one.
// ---------------------------------------------------------------------------------------------
// Text -> float: "123", "-45.6", " 7.25 g". 1 = found a number.
int read_float(char *s, float *v)
{
    float x=0, place=0.1f;
    int neg=0, found=0;
    while (*s==' ') s++;
    if (*s=='-'){
        neg=1;
        s++;
    }
    else if (*s=='+') s++;
    while (*s>='0' && *s<='9'){
        x=x*10+(*s-'0');
        found=1;
        s++;
    }
    if (*s=='.'){
        s++;
        while (*s>='0' && *s<='9'){
            x=x+(*s-'0')*place;
            place=place*0.1f;
            found=1;
            s++;
        }
    }
    if (neg) x=-x;
    *v=x;
    return found;
}

// ---------------------------------------------------------------------------------------------
// int us_to_ticks(float us)
//   Converts a pulse in microseconds to Timer2 ticks (0.9375 ticks per us, rounded).
// ---------------------------------------------------------------------------------------------
// ---------------- ESC pulses: Timer2 + OC1-OC4 at 50 Hz
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
// void all_stop(void)
//   Every motor to 1000 us right now (the relay stays as it is).
// ---------------------------------------------------------------------------------------------
void all_stop(void)             // every motor to 1000 us right now
{
    int n;
    for (n=1;n<=4;n++){
        now_us[n]=STOP_US;
        want_us[n]=STOP_US;
    }
    write_pulses();
}

// ---------------------------------------------------------------------------------------------
// void set_relay(int on)
//   Opens (0) or closes (1) the motor-power relay on RK4, J12 pin 33.
// ---------------------------------------------------------------------------------------------
void set_relay(int on)          // J12 pin 33 = RK4, HIGH = battery to the ESCs
{
    relay_on=on;
    LATKbits.LATK4=on;
}

// ---------------------------------------------------------------------------------------------
// void show_leds(void)
//   Shows which motor runs: LED1 FL, LED2 FR, LED1+LED2 RL, LED3 RR.
// ---------------------------------------------------------------------------------------------
void show_leds(void)            // LED1 FL, LED2 FR, LED1+LED2 RL, LED3 RR
{
    int a=(now_us[1]>STOP_US), b=(now_us[2]>STOP_US), c=(now_us[3]>STOP_US), d=(now_us[4]>STOP_US);
    LATHbits.LATH0=(a || c);
    LATHbits.LATH1=(b || c);
    LATHbits.LATH2=d;
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
    show_leds();
    if (PORTBbits.RB12==0 && relay_on){         // SW1 = STOP
        all_stop();
        set_relay(0);
        aborted=1;
        putsU2("");
        putsU2("SW1: all motors stopped, relay OFF.");
    }
}

// ---------------------------------------------------------------------------------------------
// int wait_service(int ms)
//   Waits ms milliseconds while the ramps and the SW1 check keep running (0 = SW1 aborted it).
// ---------------------------------------------------------------------------------------------
int wait_service(int ms)        // wait while the ramps and SW1 keep working. 0 = aborted
{
    int i;
    for (i=0;i<ms;i=i+4){
        service();
        if (aborted) return 0;
    }
    return 1;
}

// ---------------------------------------------------------------------------------------------
// int read_line(char *line, int max)
//   Waits for one typed line (echoed, backspace works) while the ramps and SW1 keep running. 0
//   = SW1.
// ---------------------------------------------------------------------------------------------
// Wait for one typed line (echoed). The ramps and SW1 keep working meanwhile.
// 1 = got a line, 0 = SW1 was pressed.
int read_line(char *line, int max)
{
    int len=0, c;
    while (1){
        service();
        if (aborted) return 0;
        while ((c=getU2())>=0){                 // every key that has arrived (a pasted line too)
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
// int ask_reading(char *question, float *v)
//   Asks for a scale reading until a number is typed. 1 = number, 2 = R (spin again), 0 = X or
//   SW1.
// ---------------------------------------------------------------------------------------------
// Ask for a scale reading. 1 = got one in *v, 2 = R (redo), 0 = X or SW1.
int ask_reading(char *question, float *v)
{
    char line[24];
    while (1){
        printU2(question);
        if (!read_line(line, 24)) return 0;
        if (line[0]=='X'){
            aborted=1;
            return 0;
        }
        if (line[0]=='R') return 2;
        if (read_float(line, v)) return 1;
        putsU2("  Type the number on the scale (R = spin again, X = abort).");
    }
}

// ---------------------------------------------------------------------------------------------
// int relay_on_and_arm(void)
//   Closes the relay with every ESC at 1000 us, then waits 4 s so the ESCs arm before anything
//   spins.
// ---------------------------------------------------------------------------------------------
int relay_on_and_arm(void)      // 1 = relay on and ESCs armed, 0 = SW1 pressed
{
    if (relay_on) return 1;
    all_stop();                 // the ESCs must see 1000 us when their power comes on
    set_relay(1);
    putsU2("Relay ON: ESCs powering up at 1000 us - wait for the beeps (4 s)...");
    return wait_service(ARM_MS);
}

// ---------------------------------------------------------------------------------------------
// int measure(int n, float us, float *r)
//   Ramps one motor to a pulse, lets it settle 3 s, asks for the scale reading, stops it, waits
//   2 s.
// ---------------------------------------------------------------------------------------------
// Spin motor n at pulse us (slow ramp), wait for the scale, ask for the reading,
// stop the motor, let the scale come back. 1 = reading in *r, 0 = aborted.
int measure(int n, float us, float *r)
{
    char q[80], t[16];
    int got;
    while (1){
        want_us[n]=us;
        while (now_us[n]<us){                   // ramp up at 200 us/s
            service();
            if (aborted) return 0;
        }
        if (!wait_service(SETTLE_MS)) return 0;
        sprintf(q, "  %s at %s us - scale reading: ", names[n], fstr(t, us, 0));
        got=ask_reading(q, r);
        all_stop();
        if (got==0) return 0;
        if (!wait_service(REST_MS)) return 0;
        if (got==1) return 1;
        // got==2: R = spin this motor again
    }
}

// ---------------------------------------------------------------------------------------------
// void fit_lines(void)
//   Least-squares fit per motor: sqrt(thrust) = slope x (pulse - zero), skipping readings where
//   it wasn't pushing.
// ---------------------------------------------------------------------------------------------
// ---------------- the math
// A propeller's thrust goes with its speed squared, and the speed goes up about
// in a straight line with the pulse. So sqrt(thrust) is a straight line in the pulse:
//     sqrt(thrust) = slope x (pulse - zero)
// Least-squares line through each motor's (pulse, sqrt(thrust)) points, leaving out
// readings where the motor wasn't really pushing yet.
void fit_lines(void)
{
    int n, k, cnt;
    float big, sx, sy, sxx, sxy, mx, my, x, y, d;
    for (n=1;n<=4;n++){
        big=0;
        for (k=0;k<NLEVELS;k++)
            if (thrust[n][k]>big) big=thrust[n][k];
        cnt=0;
        sx=0; sy=0;
        for (k=0;k<NLEVELS;k++){
            if (thrust[n][k]<=0 || thrust[n][k]<MIN_PART*big) continue;
            sx=sx+levels[k];
            sy=sy+sqrtf(thrust[n][k]);
            cnt++;
        }
        slope[n]=0;
        zero[n]=STOP_US;
        worst_fit[n]=0;
        if (cnt<2) continue;                    // not enough points for a line
        mx=sx/cnt;
        my=sy/cnt;
        sxx=0; sxy=0;
        for (k=0;k<NLEVELS;k++){
            if (thrust[n][k]<=0 || thrust[n][k]<MIN_PART*big) continue;
            x=levels[k]-mx;
            sxx=sxx+x*x;
            sxy=sxy+x*(sqrtf(thrust[n][k])-my);
        }
        slope[n]=sxy/sxx;                       // sqrt(thrust) per us
        if (slope[n]<=0) continue;
        zero[n]=mx-my/slope[n];                 // pulse where the motor starts to push
        for (k=0;k<NLEVELS;k++){                // how far each reading is from the curve
            if (thrust[n][k]<=0 || thrust[n][k]<MIN_PART*big) continue;
            y=slope[n]*(levels[k]-zero[n]);
            d=thrust[n][k]-y*y;
            if (d<0) d=-d;
            if (d>worst_fit[n]) worst_fit[n]=d;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// void make_trims(void)
//   Works out each motor's gain and offset so it makes the average motor's thrust at every
//   pulse.
// ---------------------------------------------------------------------------------------------
// Trims that make every motor match the AVERAGE motor:
//   average motor:  sqrt(thrust) = s_ref x (pulse - z_ref)
//   motor n gives the same thrust at  p_n = z_n + (s_ref / s_n) x (pulse - z_ref)
//   written as      p_n = 1000 + gain x (pulse - 1000) + offset
void make_trims(void)
{
    int n;
    float s_ref=0, z_ref=0;
    have_trims=0;
    for (n=1;n<=4;n++){
        if (slope[n]<=0) return;                // a motor without a good line: no trims
        s_ref=s_ref+slope[n];
        z_ref=z_ref+zero[n];
    }
    s_ref=s_ref/4;
    z_ref=z_ref/4;
    for (n=1;n<=4;n++){
        gain[n]=s_ref/slope[n];
        offset[n]=zero[n]-STOP_US-gain[n]*(z_ref-STOP_US);
    }
    have_trims=1;
}

// ---------------------------------------------------------------------------------------------
// float trimmed(int n, float us)
//   The pulse motor n needs to match the average motor at pulse us.
// ---------------------------------------------------------------------------------------------
float trimmed(int n, float us)  // the pulse motor n needs to match the average motor at us
{
    return STOP_US+gain[n]*(us-STOP_US)+offset[n];
}

// ---------------------------------------------------------------------------------------------
// void print_results(void)
//   Prints the thrust table with the spread per level, the fit per motor, the trims and the
//   #define lines.
// ---------------------------------------------------------------------------------------------
void print_results(void)
{
    char s[140], t1[16], t2[16], t3[16], t4[16], t5[16];
    int n, k;
    float lo, hi, avg;

    if (!have_data){
        putsU2("No sweep yet - type GO.");
        return;
    }
    putsU2("");
    putsU2("THRUST MEASURED (scale units)");
    printU2("  pulse us ");
    for (n=1;n<=4;n++){
        sprintf(s, "%9s", names[n]);
        printU2(s);
    }
    putsU2("   spread");
    for (k=0;k<NLEVELS;k++){
        sprintf(s, "  %8s ", fstr(t1, levels[k], 0));
        printU2(s);
        lo=thrust[1][k];
        hi=thrust[1][k];
        avg=0;
        for (n=1;n<=4;n++){
            sprintf(s, "%9s", fstr(t1, thrust[n][k], 1));
            printU2(s);
            if (thrust[n][k]<lo) lo=thrust[n][k];
            if (thrust[n][k]>hi) hi=thrust[n][k];
            avg=avg+thrust[n][k]/4;
        }
        if (avg>0) sprintf(s, "   %s%%", fstr(t1, (hi-lo)/avg*100, 1));
        else sprintf(s, "   (too low)");
        putsU2(s);
    }
    putsU2("  spread = (biggest - smallest) / average. Under about 5% is good.");

    putsU2("");
    putsU2("FIT PER MOTOR: sqrt(thrust) = a x (pulse - zero)");
    avg=0;
    for (n=1;n<=4;n++) avg=avg+slope[n]/4;
    for (n=1;n<=4;n++){
        if (slope[n]<=0){
            sprintf(s, "  %s: fewer than 2 readings where it pushed - check this motor, then GO again.", names[n]);
            putsU2(s);
            continue;
        }
        sprintf(s, "  %s: starts pushing at %s us   strength %s%% vs average   worst reading off the curve by %s",
                names[n], fstr(t2, zero[n], 1), fstr(t1, (slope[n]/avg-1)*100, 1), fstr(t3, worst_fit[n], 1));
        putsU2(s);
    }
    putsU2("  A worst-reading number much bigger than the others = a typo or a bad reading: GO again.");

    if (!have_trims){
        putsU2("No trims: every motor needs at least 2 readings above the minimum.");
        return;
    }
    putsU2("");
    putsU2("TRIMS (match the average motor): trimmed = 1000 + gain x (pulse - 1000) + offset");
    for (n=1;n<=4;n++){
        sprintf(s, "  %s: gain %s   offset %s us   ->  1200 us becomes %s, 1300 us becomes %s",
                names[n], fstr(t1, gain[n], 4), fstr(t2, offset[n], 1),
                fstr(t3, trimmed(n, 1200), 1), fstr(t4, trimmed(n, 1300), 1));
        putsU2(s);
    }
    putsU2("");
    putsU2("For the flight code (copy these lines):");
    for (n=1;n<=4;n++){
        sprintf(s, "#define TRIM_GAIN_%s   %sf", names[n], fstr(t1, gain[n], 4));
        putsU2(s);
        sprintf(s, "#define TRIM_OFFSET_%s %sf", names[n], fstr(t5, offset[n], 1));
        putsU2(s);
    }
    putsU2("Only trim pulses above 1000 us: a stop (1000 us) must stay exactly 1000 us.");
    putsU2("Check them with  V 1250  (each motor at its trimmed pulse).");
}

// ---------------------------------------------------------------------------------------------
// void sweep(void)
//   The calibration sweep: 5 pulse levels; at each, a baseline reading, then FL, FR, RL, RR one
//   at a time.
// ---------------------------------------------------------------------------------------------
// ---------------- the sweep and the check
void sweep(void)
{
    char q[80], t[16];
    float base, r;
    int k, n, got;

    aborted=0;
    have_data=0;
    have_trims=0;
    if (!relay_on_and_arm()) return;
    putsU2("");
    putsU2("SWEEP: 5 pulse levels x 4 motors. Type each reading when it asks (R = again, X = abort).");
    for (k=0;k<NLEVELS;k++){
        all_stop();
        sprintf(q, "Level %s us. All motors stopped - scale reading: ", fstr(t, levels[k], 0));
        do got=ask_reading(q, &base);
        while (got==2);                         // R here just asks again
        if (got!=1){
            all_stop();
            putsU2("Sweep aborted.");
            return;
        }
        for (n=1;n<=4;n++){
            if (!measure(n, levels[k], &r)){
                all_stop();
                putsU2("Sweep aborted.");
                return;
            }
            thrust[n][k]=base-r;
        }
    }
    all_stop();
    have_data=1;
    fit_lines();
    make_trims();
    print_results();
}

// ---------------------------------------------------------------------------------------------
// void check(float us)
//   V command: runs each motor at its trimmed pulse, asks for the readings and prints the
//   spread.
// ---------------------------------------------------------------------------------------------
void check(float us)
{
    char s[120], t1[16], t2[16], t3[16];
    float base, r, th[5], lo, hi, avg;
    int n, got;

    if (!have_trims){
        putsU2("No trims yet - run GO first.");
        return;
    }
    if (us<1100 || us>MAX_US){
        putsU2("Check between 1100 and 1400 us. Example: V 1250");
        return;
    }
    for (n=1;n<=4;n++){
        if (trimmed(n, us)>MAX_US || trimmed(n, us)<STOP_US){
            putsU2("A trimmed pulse would be outside 1000-1400 us - pick a lower check pulse.");
            return;
        }
    }
    aborted=0;
    if (!relay_on_and_arm()) return;
    all_stop();
    do got=ask_reading("CHECK. All motors stopped - scale reading: ", &base);
    while (got==2);
    if (got!=1){
        all_stop();
        return;
    }
    for (n=1;n<=4;n++){
        if (!measure(n, trimmed(n, us), &r)){
            all_stop();
            putsU2("Check aborted.");
            return;
        }
        th[n]=base-r;
    }
    all_stop();
    lo=th[1];
    hi=th[1];
    avg=0;
    for (n=1;n<=4;n++){
        if (th[n]<lo) lo=th[n];
        if (th[n]>hi) hi=th[n];
        avg=avg+th[n]/4;
    }
    sprintf(s, "CHECK at %s us (trimmed):", fstr(t1, us, 0));
    putsU2(s);
    for (n=1;n<=4;n++){
        sprintf(s, "  %s: pulse %s us   thrust %s", names[n], fstr(t1, trimmed(n, us), 1), fstr(t2, th[n], 1));
        putsU2(s);
    }
    if (avg>0){
        sprintf(s, "  spread %s%% (was this level untrimmed: see T). Under about 5%% is good.",
                fstr(t3, (hi-lo)/avg*100, 1));
        putsU2(s);
    }
}

// ---------------------------------------------------------------------------------------------
// void help(void)
//   Prints the commands and the setup reminders.
// ---------------------------------------------------------------------------------------------
void help(void)
{
    putsU2("");
    putsU2("=== MOTOR CALIBRATION TEST (props ON, drone on a kitchen scale) ===");
    putsU2("  GO        sweep: each motor at 1150-1350 us, you type the scale readings");
    putsU2("  V 1250    check the trims at 1250 us        T = results again");
    putsU2("  X = stop  OFF / ON = relay off / on   ? = help   SW1 = stop all + relay off");
    putsU2("Run 07_esc_calibrate first. Full battery. Stand back from the props.");
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: relay off, ESCs at 1000 us, UART up, then runs GO (sweep), V (check), T, X,
//   ON/OFF.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char line[40];
    float us;

    T3CON=0x8070;               // Timer3 ON, 1:256 (delays)

    LATKbits.LATK4=0;           // relay pin LOW first, then make it an output: power stays off
    TRISKbits.TRISK4=0;
    relay_on=0;

    initMotors();               // all 4 ESCs at 1000 us

    ANSELBbits.ANSB12=0;        // SW1 = stop
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;        // pull-up: SW1 reads 1 until pressed
    TRISHbits.TRISH0=0;         // LED1-3
    TRISHbits.TRISH1=0;
    TRISHbits.TRISH2=0;
    LATHbits.LATH0=0;
    LATHbits.LATH1=0;
    LATHbits.LATH2=0;

    initU2();
    delay_ms(100);
    help();

    while (1){
        printU2(">");
        aborted=0;
        if (!read_line(line, 40)) continue;     // SW1 while waiting: already stopped
        if (line[0]=='G') sweep();
        else if (line[0]=='V'){
            if (read_float(line+1, &us)) check(us);
            else putsU2("Example: V 1250");
        }
        else if (line[0]=='T') print_results();
        else if (line[0]=='X' || line[0]=='S' || line[0]==0){
            all_stop();
            putsU2("All motors stopped. Relay still on - type OFF to open it.");
        }
        else if (line[0]=='O' && line[1]=='F'){
            all_stop();
            set_relay(0);
            putsU2("All motors stopped, relay OFF.");
        }
        else if (line[0]=='O' && line[1]=='N'){
            if (relay_on_and_arm()) putsU2("ESCs armed.");
        }
        else if (line[0]=='?' || line[0]=='H') help();
        else putsU2("Not a command (? = help).");
    }
    return 0;
}
