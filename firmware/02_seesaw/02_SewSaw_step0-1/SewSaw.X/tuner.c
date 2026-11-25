/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/02_SewSaw_step0-1/SewSaw.X/tuner.c
 *  What    : PID-tuner protocol (PC or phone): gains, setpoint, recorded step tests, live data.
 *  Version : SewSaw v1   |   Date: 2026-11-16
 *  Status  : WORKED: step 0 (IMU check) passed and step 1 ran on the board Nov 16.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC pins 11/12 are reserved for the seesaw motors but not driven yet (motors.c arrives
 *      in step 2-3)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): read_numbers(), tuner_command(),
 *    tuner_send_live(), tuner_send_run(), tuner_setpoint_step(), tuner_record()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include "tuner.h"
# include "control.h"
# include "imu.h"
# include "uart.h"

// Link to the PID tuner on the PC (tools/pid-tuner/pid_tuner.py from the capstone zip).
// It uses the same COM port as PuTTY: close PuTTY before connecting the tuner.
//
// Tuner -> PIC (one line each):
//   K,kp,ki,kd       new gains                         (PIC answers A,kp,ki,kd)
//   S,deg            new setpoint
//   R,from,to,ms     step test: hold "from" 1 s, record 0.2 s, jump to "to", record ms, send it all
//   L,1  /  L,0      live data on / off
//   X                KILL: motor output off (a step test turns it back on)
// PIC -> tuner:
//   D,t_ms,angle,setpoint,motor_us,P,I,D     one sample (motor_us = motor A, the left end)
//   RUN_BEGIN ... D lines ... RUN_END        a recorded step test (t_ms counted from the step)
//   anything else shows up in the tuner's Console tab

#define PRE_STEP   50       // samples kept from before the step (0.2 s)
#define RUN_MAX    1300     // longest recording: 0.2 s + 5 s = 1300 samples
#define HOLD_LOOPS 250      // hold the start angle 1 s before recording

volatile int display_on=1;
volatile int live_on=0;
volatile int run_state=0;
volatile int run_count=0;
volatile int run_total=0;
volatile float run_from=0, run_to=0;

float rec_angle[RUN_MAX], rec_sp[RUN_MAX], rec_motor[RUN_MAX];
float rec_p[RUN_MAX], rec_i[RUN_MAX], rec_d[RUN_MAX];

// ---------------------------------------------------------------------------------------------
// int read_numbers(char *line, float *n1, float *n2, float *n3)
//   Reads up to 3 numbers after the command letter: 'K,2.5,0,0.4' -> 2.5, 0, 0.4. Returns how
//   many.
// ---------------------------------------------------------------------------------------------
// read up to 3 numbers after the letter: "K,2.5,0,0.4" -> 2.5, 0, 0.4. Returns how many it found.
int read_numbers(char *line, float *n1, float *n2, float *n3)
{
    char *p;

    p=line+2;                   // skip "K,"
    *n1=atof(p);
    p=strchr(p, ',');           // find the next comma
    if (p==0) return 1;
    p++;
    *n2=atof(p);
    p=strchr(p, ',');
    if (p==0) return 2;
    p++;
    *n3=atof(p);
    return 3;
}

// ---------------------------------------------------------------------------------------------
// void tuner_command(char *line)
//   Runs one command line from the PID tuner (K gains, S setpoint, R step test, G, M, B, X, Z,
//   ...).
// ---------------------------------------------------------------------------------------------
void tuner_command(char *line)
{
    char s[100], t1[16], t2[16], t3[16];
    float n1=0, n2=0, n3=0;
    int found;

    display_on=0;                           // the tuner is talking: stop the PuTTY lines

    if (line[0]=='K' && line[1]==','){      // K,kp,ki,kd
        found=read_numbers(line, &n1, &n2, &n3);
        if (found==3){
            Kp=n1;
            Ki=n2;
            Kd=n3;
            sprintf(s, "A,%s,%s,%s", fstr(t1, Kp, 4), fstr(t2, Ki, 4), fstr(t3, Kd, 4));
            putsU2(s);
        }
    }
    else if (line[0]=='S' && line[1]==','){ // S,deg
        if (run_state==0) setpoint=atof(line+2);
    }
    else if (line[0]=='R' && line[1]==','){ // R,from,to,ms
        found=read_numbers(line, &n1, &n2, &n3);
        if (found==3 && run_state==0){
            if (n3>5000) n3=5000;           // the recording holds 5 s at most
            if (n3<4) n3=4;
            run_from=n1;
            run_to=n2;
            run_total=PRE_STEP+(int)(n3/4); // one sample every 4 ms
            run_count=HOLD_LOOPS;
            pid_on=1;
            run_state=1;                    // set last: the interrupt takes it from here
        }
    }
    else if (line[0]=='L' && line[1]==','){ // L,1 or L,0
        if (line[2]=='1') live_on=1;
        else live_on=0;
    }
    else if (line[0]=='X'){                 // KILL
        pid_on=0;
        if (run_state!=0){
            run_state=0;
            putsU2("RUN_END");
        }
        putsU2("KILL - motor output off. Start a step test to turn it back on.");
    }
    else{
        sprintf(s, "Unknown command: %.40s", line);
        putsU2(s);
    }
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_live(void)
//   Sends one live data line (D,...) to the tuner graph.
// ---------------------------------------------------------------------------------------------
void tuner_send_live(void)
{
    char s[100], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16];

    sprintf(s, "D,%lu,%s,%s,%s,%s,%s,%s", loop_count*4,
            fstr(t1, roll, 2), fstr(t2, setpoint, 2), fstr(t3, motor_a, 1),
            fstr(t4, p_term, 1), fstr(t5, i_term, 1), fstr(t6, d_term, 1));
    putsU2(s);
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_run(void)
//   Sends the recorded step test to the PC, one line per call, so commands still work
//   meanwhile.
// ---------------------------------------------------------------------------------------------
void tuner_send_run(void)
{
    char s[100], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16];
    int k;

    putsU2("RUN_BEGIN");
    for (k=0;k<run_total;k++){
        sprintf(s, "D,%d,%s,%s,%s,%s,%s,%s", (k-PRE_STEP)*4,
                fstr(t1, rec_angle[k], 2), fstr(t2, rec_sp[k], 2), fstr(t3, rec_motor[k], 1),
                fstr(t4, rec_p[k], 1), fstr(t5, rec_i[k], 1), fstr(t6, rec_d[k], 1));
        putsU2(s);
    }
    putsU2("RUN_END");
    run_state=0;
}

// ---- called from the 4 ms interrupt ----

// ---------------------------------------------------------------------------------------------
// void tuner_setpoint_step(void)
//   Runs every 4 ms: holds the start angle, then makes the setpoint step for a step test.
// ---------------------------------------------------------------------------------------------
void tuner_setpoint_step(void)
{
    if (run_state==1){                      // holding the start angle for 1 s
        setpoint=run_from;
        run_count--;
        if (run_count<=0) run_state=2;      // run_count is 0 now: recording starts at sample 0
    }
    else if (run_state==2){
        if (run_count==PRE_STEP) setpoint=run_to;   // THE STEP (t = 0)
    }
}

// ---------------------------------------------------------------------------------------------
// void tuner_record(void)
//   Runs every 4 ms during a step test: stores angle, setpoint, motor pulses and P/I/D terms.
// ---------------------------------------------------------------------------------------------
void tuner_record(void)
{
    if (run_state!=2) return;

    rec_angle[run_count]=roll;
    rec_sp[run_count]=setpoint;
    rec_motor[run_count]=motor_a;
    rec_p[run_count]=p_term;
    rec_i[run_count]=i_term;
    rec_d[run_count]=d_term;
    run_count++;
    if (run_count>=run_total) run_state=3;  // full: main() sends it
}
