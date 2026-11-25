/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/10_motor_id_test/motor_id_test.c
 *  What    : Motor ID test - spin the motor you name from PuTTY (RR 1150) to check wiring and
 *            direction
 *  Version : bench test   |   Date: 2026-11-25
 *  Status  : New Nov 25. Checked in a PC simulation (17 checks: each name drives the right pin
 *            and tick count, slow ramp, relay arming, SW1, bad input). Not yet run on the
 *            board.
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
 *    putsU2(), getU2(), us_to_ticks(), initMotors(), write_pulses(), all_stop(), set_relay(),
 *    show_leds(), service(), wait_service(), relay_on_and_arm(), help(), upper(), read_int(),
 *    run_command(), main()
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
// MOTOR ID TEST - spin ONE motor you pick from PuTTY, to check the wires and
// the spin direction of each motor
// Ethan DiMeo
//
// Type a motor name, a space and a pulse in microseconds, then Enter:
//     RR 1150      rear-right motor at 1150 us, the other 3 stopped
//     FL 1100      front-left motor at 1100 us (the rear-right one stops)
//     ALL 1100     all 4 motors at 1100 us
//     S            stop all (an empty Enter, X or STOP also stops all)
//     OFF          stop all and open the motor-power relay
//     ?            help
// Motor names: FL, FR, RL, RR (BL and BR also work for the back motors).
// The motor ramps up slowly (200 us per second). Going down is immediate.
// SW1 on the kit = stop all + relay off, any time.
//
// Wiring (J12 pinout, drone v2.0):
//   ESC 1 FL front-left : signal pin 11 (RD0 = OC1)   ground pin 14
//   ESC 2 FR front-right: signal pin 12 (RF2 = OC2)   ground pin 20
//   ESC 3 RL rear-left  : signal pin 21 (RF4 = OC3)   ground pin 25
//   ESC 4 RR rear-right : signal pin 19 (RF5 = OC4)   ground pin 30
//   Motor-power relay   : pin 33 (RK4), HIGH = motor power on
//   PuTTY: the kit's USB serial port (J11), 115200 baud, 8N1. Never wire pins 8 and 10.
//
// Spin directions seen from ABOVE (checked Nov 18):
//   FL = CCW (silver nut)   FR = CW (black nut)   RL = CW (black nut)   RR = CCW (silver nut)
//   Wrong direction: swap any 2 of that motor's 3 wires at the ESC.
//   Wrong motor moves: that ESC's signal wire is on the wrong J12 pin.
//
// PROPS OFF for this test. Pulses are limited to 1000-1500 us.
//
// ESC pulse: Timer2 = 60 MHz / 64 = 937,500 ticks per second
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms = 50 Hz
//   ticks = us x 0.9375:  1000 us = 938 ticks (stop)   1150 us = 1078   1500 us = 1406
// =====================================================================

#define STOP_US     1000.0f     // 1.00 ms = motor off
#define MAX_US      1500.0f     // highest pulse this test will send (props off)
#define RAMP_STEP   0.8f        // +0.8 us every 4 ms = 200 us per second (slow start)
#define ARM_MS      4000        // after the relay closes: 1 ms pulses for 4 s so the ESCs arm (beeps)

char *names[5]={"", "FL", "FR", "RL", "RR"};
char *where[5]={"", "ESC 1 front-left,  J12 pin 11 (OC1)", "ESC 2 front-right, J12 pin 12 (OC2)",
                    "ESC 3 rear-left,   J12 pin 21 (OC3)", "ESC 4 rear-right,  J12 pin 19 (OC4)"};
char *spin[5]={"", "CCW", "CW", "CW", "CCW"};

float now_us[5]={0, STOP_US, STOP_US, STOP_US, STOP_US};    // pulse each ESC gets now (index 1-4)
float want_us[5]={0, STOP_US, STOP_US, STOP_US, STOP_US};   // pulse it is ramping to
int relay_on=0;

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
// ---------------- relay (J12 pin 33 = RK4) and the kit's LEDs / SW1
void set_relay(int on)
{
    relay_on=on;
    LATKbits.LATK4=on;          // HIGH = relay closed = battery to the ESCs
}

// ---------------------------------------------------------------------------------------------
// void show_leds(void)
//   Shows which motor runs: LED1 FL, LED2 FR, LED1+LED2 RL, LED3 RR.
// ---------------------------------------------------------------------------------------------
void show_leds(void)            // LED1 FL, LED2 FR, LED1+LED2 RL, LED3 RR, all 3 = ALL
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
    if (PORTBbits.RB12==0 && (relay_on || now_us[1]>STOP_US || now_us[2]>STOP_US ||
                              now_us[3]>STOP_US || now_us[4]>STOP_US)){   // SW1 = STOP
        all_stop();
        set_relay(0);
        putsU2("");
        putsU2("SW1: all motors stopped, relay OFF.");
    }
}

// ---------------------------------------------------------------------------------------------
// void wait_service(int ms)
//   Waits ms milliseconds while the ramps and the SW1 check keep running (0 = SW1 aborted it).
// ---------------------------------------------------------------------------------------------
void wait_service(int ms)       // wait ms milliseconds while the ramps and SW1 keep working
{
    int i;
    for (i=0;i<ms;i=i+4) service();
}

// ---------------------------------------------------------------------------------------------
// void relay_on_and_arm(void)
//   Closes the relay with every ESC at 1000 us, then waits 4 s so the ESCs arm before anything
//   spins.
// ---------------------------------------------------------------------------------------------
void relay_on_and_arm(void)
{
    if (relay_on) return;
    all_stop();                 // the ESCs must see 1000 us when their power comes on
    set_relay(1);
    putsU2("Relay ON: ESCs powering up at 1000 us - wait for the beeps (4 s)...");
    wait_service(ARM_MS);
    putsU2("ESCs armed.");
}

// ---------------------------------------------------------------------------------------------
// void help(void)
//   Prints the commands and the spin direction each motor should have.
// ---------------------------------------------------------------------------------------------
// ---------------- commands
void help(void)
{
    putsU2("");
    putsU2("=== MOTOR ID TEST (props OFF) ===");
    putsU2("  RR 1150   rear-right at 1150 us (only that motor spins)");
    putsU2("  FL / FR / RL / RR / ALL  then a pulse from 1000 to 1500 us");
    putsU2("  S, X, STOP or just Enter = stop all    OFF = stop all + relay off");
    putsU2("  ON = relay on    ? = this help    SW1 on the kit = stop all + relay off");
    putsU2("Seen from above: FL CCW, FR CW, RL CW, RR CCW.");
    putsU2("Wrong direction: swap 2 of that motor's 3 wires. Wrong motor: ESC signal on the wrong pin.");
}

// ---------------------------------------------------------------------------------------------
// void upper(char *s)
//   Changes a typed line to uppercase, so rr and RR both work.
// ---------------------------------------------------------------------------------------------
void upper(char *s)             // lowercase -> uppercase, in place
{
    while (*s){
        if (*s>='a' && *s<='z') *s=*s-32;
        s++;
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
// void run_command(char *line)
//   Runs one PuTTY line: FL/FR/RL/RR/ALL plus a pulse, S/X/Enter = stop, ON/OFF = relay, ? =
//   help.
// ---------------------------------------------------------------------------------------------
void run_command(char *line)
{
    char s[120];
    int motor=-1, us, n;

    upper(line);
    while (*line==' ') line++;                  // skip leading spaces

    if (line[0]==0 || line[0]=='S' || line[0]=='X'){   // Enter, S, STOP, X
        all_stop();
        putsU2("All motors stopped (1000 us). Relay still on - type OFF to open it.");
        return;
    }
    if (line[0]=='?' || line[0]=='H'){
        help();
        return;
    }
    if (line[0]=='O' && line[1]=='F'){          // OFF
        all_stop();
        set_relay(0);
        putsU2("All motors stopped, relay OFF.");
        return;
    }
    if (line[0]=='O' && line[1]=='N'){          // ON
        relay_on_and_arm();
        return;
    }

    if (line[0]=='A' && line[1]=='L' && line[2]=='L') motor=0;
    else if (line[0]=='F' && line[1]=='L') motor=1;
    else if (line[0]=='F' && line[1]=='R') motor=2;
    else if ((line[0]=='R' || line[0]=='B') && line[1]=='L') motor=3;
    else if ((line[0]=='R' || line[0]=='B') && line[1]=='R') motor=4;

    if (motor<0){
        putsU2("Not a command. Example: RR 1150   (? = help)");
        return;
    }
    if (!read_int(line+2, &us)){
        putsU2("Give a pulse too. Example: RR 1150");
        return;
    }
    if (us<STOP_US || us>MAX_US){
        sprintf(s, "%d us is outside 1000-1500 us. Nothing changed.", us);
        putsU2(s);
        return;
    }

    relay_on_and_arm();
    if (!relay_on) return;                      // SW1 was pressed while arming

    for (n=1;n<=4;n++){                         // only the chosen motor (or all of them)
        if (motor==0 || n==motor) want_us[n]=us;
        else{
            want_us[n]=STOP_US;
            now_us[n]=STOP_US;
        }
    }
    if (motor==0){
        sprintf(s, "ALL 4 -> %d us (%d ticks), ramping at 200 us/s.", us, us_to_ticks(us));
        putsU2(s);
    }
    else{
        sprintf(s, "%s = %s -> %d us (%d ticks). Should spin %s seen from above.",
                names[motor], where[motor], us, us_to_ticks(us), spin[motor]);
        putsU2(s);
    }
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: relay off, ESCs at 1000 us, UART up, then runs each PuTTY line (RR 1150, ALL
//   1120, S, OFF, ...).
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char line[40];
    int len=0, c;

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
    putsU2("Relay is OFF. The first motor command turns it on (or type ON).");
    putU2('>');

    while (1){
        service();              // ramps + SW1, every 4 ms
        while ((c=getU2())>=0){                 // every key that has arrived (a pasted line too)
            if (c=='\r' || c=='\n'){            // Enter: run the line
                if (c=='\n' && len==0) continue;    // the LF after a CR
                putsU2("");
                line[len]=0;
                run_command(line);
                len=0;
                putU2('>');
            }
            else if ((c==8 || c==127) && len>0){    // backspace
                len--;
                putU2(8);
                putU2(' ');
                putU2(8);
            }
            else if (c>=' ' && len<39){
                line[len++]=c;
                putU2(c);                       // echo, so you see what you type
            }
        }
    }
    return 0;
}
