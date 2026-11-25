/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/tuner.c
 *  What    : PID-tuner protocol (PC or phone): gains, setpoint, recorded step tests, live data.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): read_numbers(), clamp_us(),
 *    tuner_help(), tuner_command(), tuner_send_live(), tuner_send_run(), tuner_setpoint_step(),
 *    tuner_record()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include "tuner.h"
# include "control.h"
# include "motors.h"
# include "imu.h"
# include "uart.h"

// Link to the PID tuner on the PC (tools/pid-tuner/pid_tuner.py, the version with the
// "Motors A & B" panel). It uses the same COM port as PuTTY: close PuTTY first.
//
// Tuner -> PIC (one line each):
//   K,kp,ki,kd       new gains                         (PIC answers A,kp,ki,kd)
//   S,deg            new setpoint
//   R,from,to,ms     step test: PID on, soft start, hold "from" 1 s, step to "to", record ms
//   L,1  /  L,0      live data on / off
//   X                KILL: motors off
//   G                PID on, hold the setpoint (no step test)
//   M,a,b            manual: PID off, motor A = a us, motor B = b us
//   B,a,b            PID base pulse of motor A and B (the GUI sends it when it connects)
//   Z                zero the angle: hold the seesaw level, then send Z
//   ?                list the commands
// PIC -> tuner:
//   D,t_ms,angle,setpoint,motorA_us,P,I,D,motorB_us   one live sample (every 8 ms)
//   RUN_BEGIN ... D lines ... RUN_END                  a recorded step test (t_ms from the step)
//   anything else shows up in the tuner's Console tab

#define PRE_STEP   50       // samples kept from before the step (0.2 s)
#define RUN_MAX    5050     // longest recording: 0.2 s + 20 s = 5050 samples
#define HOLD_LOOPS 250      // hold the start angle 1 s (after the soft start) before the step

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

    if (line[1]!=',') return 0; // no numbers at all
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
// float clamp_us(float us)
//   Clamps a pulse to 1000-2000 us.
// ---------------------------------------------------------------------------------------------
float clamp_us(float us)
{
    if (us<PULSE_OFF) us=PULSE_OFF;
    if (us>PULSE_MAX) us=PULSE_MAX;
    return us;
}

// ---------------------------------------------------------------------------------------------
// void tuner_help(void)
//   Sends the list of tuner commands.
// ---------------------------------------------------------------------------------------------
void tuner_help(void)
{
    putsU2("Commands: K,kp,ki,kd  S,deg  R,from,to,ms  L,1/L,0  X=KILL");
    putsU2("          G=PID on  M,a,b=manual us  B,a,b=base us  Z=zero angle  ?=help");
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
    found=read_numbers(line, &n1, &n2, &n3);

    if (line[0]=='K'){                      // K,kp,ki,kd
        if (found==3){
            Kp=n1;
            Ki=n2;
            Kd=n3;
        }
        sprintf(s, "A,%s,%s,%s", fstr(t1, Kp, 4), fstr(t2, Ki, 4), fstr(t3, Kd, 4));
        putsU2(s);
    }
    else if (line[0]=='S'){                 // S,deg
        if (found==1 && run_state==0) setpoint=n1;
    }
    else if (line[0]=='R'){                 // R,from,to,ms
        if (found==3 && run_state==0){
            if (n3>20000) n3=20000;         // the recording holds 20 s at most
            if (n3<100) n3=100;
            run_from=n1;
            run_to=n2;
            run_total=PRE_STEP+(int)(n3/4); // one sample every 4 ms
            run_count=HOLD_LOOPS;
            setpoint=n1;
            motor_mode=1;                   // PID on (soft start if the motors were off)
            run_state=1;                    // set last: the interrupt takes it from here
            putsU2("Step test started");
        }
    }
    else if (line[0]=='L'){                 // L,1 or L,0
        if (line[2]=='1') live_on=1;
        else live_on=0;
    }
    else if (line[0]=='X'){                 // KILL
        motors_off();
        if (run_state==1 || run_state==2){
            run_state=0;
            putsU2("RUN_END");
        }
        putsU2("KILL - motors off");
    }
    else if (line[0]=='G'){                 // PID on, hold the setpoint
        motor_mode=1;
        putsU2("PID on");
    }
    else if (line[0]=='M'){                 // M,a,b manual
        if (found==2){
            man_a=clamp_us(n1);
            man_b=clamp_us(n2);
            motor_mode=2;
            sprintf(s, "Manual: A %s us  B %s us", fstr(t1, man_a, 0), fstr(t2, man_b, 0));
            putsU2(s);
        }
    }
    else if (line[0]=='B'){                 // B,a,b PID base
        if (found==2){
            base_a=clamp_us(n1);
            base_b=clamp_us(n2);
        }
        sprintf(s, "Base: A %s us  B %s us", fstr(t1, base_a, 0), fstr(t2, base_b, 0));
        putsU2(s);
    }
    else if (line[0]=='Z'){                 // zero the angle where the seesaw is now
        angle_offset=angle_offset+angle;
        sprintf(s, "Angle zeroed (offset %s deg)", fstr(t1, angle_offset, 2));
        putsU2(s);
    }
    else if (line[0]=='?'){
        tuner_help();
    }
    else{
        sprintf(s, "Unknown command: %.40s  (? = help)", line);
        putsU2(s);
    }
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_live(void)
//   Sends one live data line (D,...) to the tuner graph.
// ---------------------------------------------------------------------------------------------
void tuner_send_live(void)
{
    char s[100], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16], t7[16];

    sprintf(s, "D,%lu,%s,%s,%s,%s,%s,%s,%s", loop_count*4,
            fstr(t1, angle, 2), fstr(t2, setpoint, 2), fstr(t3, motor_a, 0),
            fstr(t4, p_term, 1), fstr(t5, i_term, 1), fstr(t6, d_term, 1),
            fstr(t7, motor_b, 0));
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
                fstr(t1, rec_angle[k], 2), fstr(t2, rec_sp[k], 2), fstr(t3, rec_motor[k], 0),
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
    if ((run_state==1 || run_state==2) && motor_mode!=1){
        run_state=4;                        // the motors were turned off: main() ends the test
        return;
    }
    if (run_state==1){                      // holding the start angle for 1 s
        setpoint=run_from;
        if (ramp_us<PULSE_MAX) return;      // wait until the soft start is done
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

    rec_angle[run_count]=angle;
    rec_sp[run_count]=setpoint;
    rec_motor[run_count]=motor_a;
    rec_p[run_count]=p_term;
    rec_i[run_count]=i_term;
    rec_d[run_count]=d_term;
    run_count++;
    if (run_count>=run_total) run_state=3;  // full: main() sends it
}
