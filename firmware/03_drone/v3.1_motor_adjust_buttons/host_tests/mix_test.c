/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/host_tests/mix_test.c
 *  What    : Host test: the two PIDs, the 4-motor mixer signs, one axis at a time, every v3.x
 *            command including O (live +/- per motor), and the step-test dumps.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : Built in MPLAB on the laptop (Nov 25). v3.0 plus the O command. Host tests pass
 *            (87 checks). Replaced by v3.2 (same code, Bluefruit CS on pin 40).
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): save_dump(), putsU2(), tick(),
 *    run_for(), arm_and_wait(), near(), us_of(), main()
 * ============================================================================================ */

/* v3.0/v3.1 host test: the two PIDs, the 4-motor mixer, one axis at a time, the soft start,
   the per-motor trims, the live per-motor offsets (O, v3.1) and the new commands
   (A, KR/KP, SR/SP, B,T, M with 1/2/4 numbers, U, O).
   The stand-in IMU reads 0, so one filter update gives 0.98 x the previous angle: the test
   sets angle / 0.98 so the loop sees exactly the angle it wants (rates are 0, so D = 0). */
#include "p32xxxx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "control.h"
#include "motors.h"
#include "imu.h"
#include "tuner.h"
void Controller(void);
float trim(int n, float us);
extern int check_count;
extern volatile int data_complete; extern volatile int run_total;
extern float data_sp_roll[], data_sp_pitch[], data_m1[], data_m2[], data_m3[], data_m4[];
static char last[220]; static int nl=0; static char dump[400][220];
// ---------------------------------------------------------------------------------------------
// static void save_dump(const char *name)
//   Writes the captured PC lines to a file in TUNER_DUMP (read by the PC tuner v7 and phone app
//   3.0 tests).
// ---------------------------------------------------------------------------------------------
static void save_dump(const char *name){         /* for the PC tuner test (tuner_tests/test_v7.py) */
    if (!getenv("TUNER_DUMP")) return;
    char fn[300]; snprintf(fn,300,"%s/%s",getenv("TUNER_DUMP"),name);
    FILE *f=fopen(fn,"w"); if (!f) return;
    for (int i=0;i<nl;i++) fprintf(f,"%s\n",dump[i]); fclose(f); }
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Test stand-in for the UART: keeps the last line and the first 400 lines (the step-test
//   dumps).
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ strncpy(last,s,219); if(nl<400) strncpy(dump[nl++],s,219); }
static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s\n",msg); fails++;} }while(0)
static float hold_pitch=0, hold_roll=0;
// ---------------------------------------------------------------------------------------------
// static void tick(void)
//   Test helper: moves the simulated clock 4 ms forward and runs the control interrupt once.
// ---------------------------------------------------------------------------------------------
static void tick(void){ cp0+=4u*60000u; check_count=0; pitch=hold_pitch/0.98f; roll=hold_roll/0.98f; Controller(); }
// ---------------------------------------------------------------------------------------------
// static void run_for(int loops)
//   Test helper: runs the 4 ms control loop 'loops' times.
// ---------------------------------------------------------------------------------------------
static void run_for(int loops){ for(int i=0;i<loops;i++){ if(i%25==0) control_heartbeat(2); tick(); } }
// ---------------------------------------------------------------------------------------------
// static void arm_and_wait(void)
//   Test helper: heartbeat, ARM from the PC, then wait past the 2 s ESC start-up.
// ---------------------------------------------------------------------------------------------
static void arm_and_wait(void){ tick(); control_heartbeat(2); control_arm(2); run_for(520); }
// ---------------------------------------------------------------------------------------------
// static int near(float a, float b)
//   Test helper: 1 if two floats agree within 0.01.
// ---------------------------------------------------------------------------------------------
static int near(float a, float b){ return a>b-0.01f && a<b+0.01f; }
// ---------------------------------------------------------------------------------------------
// static int us_of(unsigned int t)
//   Test helper: converts OCxRS ticks back to microseconds (0.9375 ticks per us).
// ---------------------------------------------------------------------------------------------
static int us_of(unsigned int t){ return (int)(t/0.9375f+0.5f); }

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void){
    SPI1STATbits.SPIRBF=1; PORTBbits.RB12=1; IEC0bits.T4IE=1; control_ready=1; cp0=1000;
    arm_and_wait();

    /* --- protocol --- */
    tuner_command("A,1");  CHECK(axis_mode==1 && strcmp(last,"AXIS,1,ROLL")==0, "A,1 selects ROLL (reply AXIS,1,ROLL)");
    tuner_command("K,10,0,0");  CHECK(Kp_roll==10 && Kp_pitch==1, "K in ROLL mode sets only the roll gains");
    tuner_command("KP,10,0.5,0.25"); CHECK(Kp_pitch==10 && Ki_pitch==0.5f && Kd_pitch==0.25f && strcmp(last,"AP,+10.0000,+0.5000,+0.2500")==0, "KP sets the pitch gains any time (reply AP,...)");
    tuner_command("KP,10,0,0");
    tuner_command("B,1500");     CHECK(throttle==1500 && strcmp(last,"BASE,1500,1500")==0, "B,T sets the throttle (reply BASE,T,T)");
    tuner_command("B,1400,1600"); CHECK(throttle==1500, "old B,front,back = the average");
    tuner_command("U,250"); CHECK(ramp_rate==250, "U,250 sets the soft start to 250 us/s");
    tuner_command("U,100");
    tuner_command("SR,3"); tuner_command("SP,-2"); CHECK(sp_roll==3 && sp_pitch==-2, "SR / SP set each setpoint");
    tuner_command("A,3"); tuner_command("S,4,6"); CHECK(sp_roll==4 && sp_pitch==6, "BOTH: S,roll,pitch");
    tuner_command("S,0,0"); tuner_command("A,2");

    /* --- PITCH only: RC must be 0, front pair +PC, back pair -PC --- */
    tuner_command("G");
    run_for(250);
    CHECK(near(thr_now,1100), "soft start: 1 s after G the throttle is at 1100 us (100 us/s)");
    run_for(1100);                                                 /* 5.4 s: ramp done */
    hold_pitch=-5; hold_roll=-4; run_for(2);                       /* nose LOW and left side low */
    CHECK(near(pitch_comp,50) && near(roll_comp,0), "PITCH mode, nose 5 deg low: PC = +50 us, RC = 0 (roll ignored)");
    CHECK(us_of(OC1RS)==1550 && us_of(OC2RS)==1550 && us_of(OC3RS)==1450 && us_of(OC4RS)==1450,
          "PITCH mode: FL = FR = 1550 (front faster -> nose comes up), RL = RR = 1450");
    tuner_command("A,1"); CHECK(axis_mode==2, "axis can't change while the motors run");
    tuner_command("X"); hold_pitch=0; hold_roll=0; off_reason=0; arm_and_wait();

    /* --- ROLL only: PC must be 0, left pair +RC, right pair -RC --- */
    tuner_command("A,1"); tuner_command("G"); run_for(1300);
    hold_roll=-4; hold_pitch=-5; run_for(2);                       /* left side 4 deg low, nose low too */
    CHECK(near(roll_comp,40) && near(pitch_comp,0), "ROLL mode, left side 4 deg low: RC = +40 us, PC = 0 (pitch ignored)");
    CHECK(us_of(OC1RS)==1540 && us_of(OC3RS)==1540 && us_of(OC2RS)==1460 && us_of(OC4RS)==1460,
          "ROLL mode: FL = RL = 1540 (left faster -> left side comes up), FR = RR = 1460");
    tuner_command("X"); hold_pitch=0; hold_roll=0; off_reason=0; arm_and_wait();

    /* --- BOTH: all four different, = T +/- PC +/- RC --- */
    tuner_command("A,3"); tuner_command("G"); run_for(1300);
    hold_roll=-4; hold_pitch=-3; run_for(2);                       /* RC = +40, PC = +30 */
    CHECK(us_of(OC1RS)==1570 && us_of(OC2RS)==1490 && us_of(OC3RS)==1510 && us_of(OC4RS)==1430,
          "BOTH: FL 1570 = T+PC+RC, FR 1490 = T+PC-RC, RL 1510 = T-PC+RC, RR 1430 = T-PC-RC (all different)");
    hold_roll=8; hold_pitch=3; run_for(2);                         /* left side HIGH, nose HIGH */
    CHECK(motor_us[1]<motor_us[4] && motor_us[2]<motor_us[4] && motor_us[3]<motor_us[4] && motor_us[2]>motor_us[3],
          "BOTH, left side and nose high: FL slowest... RR fastest (both corrections push it back)");
    hold_roll=-55; hold_pitch=0; run_for(2);                       /* RC = 550 -> clamped to 500 */
    CHECK(motor_us[1]==2000 && motor_us[4]==1100, "big error: FL capped at 2000 us, RR held at the 1100 us idle floor");
    hold_roll=0; run_for(2);
    tuner_command("X"); off_reason=0; arm_and_wait();

    /* --- integrals only while really flying (after the soft start) --- */
    tuner_command("A,2"); tuner_command("KP,10,5,0"); tuner_command("G"); hold_pitch=-2;
    run_for(500);
    CHECK(eint_pitch==0, "pitch integral held at 0 during the soft start");
    run_for(1000);
    CHECK(eint_pitch>0 && eint_roll==0, "pitch integral grows once the throttle is reached; roll integral stays 0 in PITCH mode");
    hold_pitch=0; tuner_command("X"); off_reason=0; tuner_command("KP,10,0,0");

    /* --- manual: M,t (all four) and M,fl,fr,rl,rr --- */
    arm_and_wait(); tuner_command("M,1300"); run_for(800);
    CHECK(us_of(OC1RS)==1300 && us_of(OC2RS)==1300 && us_of(OC3RS)==1300 && us_of(OC4RS)==1300, "M,1300: all four at 1300 us (finding liftoff)");
    tuner_command("M,1200,1250,1300,1350"); run_for(10);
    CHECK(us_of(OC1RS)==1296 && us_of(OC2RS)==1296, "manual changes are gradual both ways (FL 1300 -> 1296 after 40 ms)");
    run_for(300);
    CHECK(us_of(OC1RS)==1200 && us_of(OC2RS)==1250 && us_of(OC3RS)==1300 && us_of(OC4RS)==1350, "M,fl,fr,rl,rr: each motor its own pulse");
    tuner_command("X"); off_reason=0;

    /* --- step test on ROLL only: only the roll setpoint steps, D lines graph roll, pairs = left/right --- */
    arm_and_wait(); tuner_command("A,1"); tuner_command("S,0"); tuner_command("SP,0");
    nl=0; tuner_command("R,0,5,100");
    int loops=0; while(!data_complete && loops<4000){ run_for(1); loops++; }
    CHECK(data_complete==1 && data_sp_roll[60]==5 && data_sp_pitch[60]==0 && data_sp_roll[10]==0, "ROLL step test: roll setpoint 0 -> 5 at the step, pitch setpoint untouched");
    nl=0; while(data_complete) tuner_send_run();
    CHECK(strcmp(dump[0],"RUN_BEGIN")==0 && strcmp(dump[nl-1],"RUN_END")==0 && nl==run_total+2, "PC dump: RUN_BEGIN + one D line per sample + RUN_END");
    { int commas=0; for(char *c=dump[60];*c;c++) if(*c==',') commas++;
      CHECK(commas==10 && strstr(dump[60],",+5.00,")!=0, "ROLL dump line: 11 fields, setpoint 5.00 is the roll setpoint"); }
    save_dump("pc_roll.txt");
    tuner_command("X"); off_reason=0;

    /* --- BOTH step test: both setpoints step, D lines carry all 4 motors + RC + PC --- */
    arm_and_wait(); tuner_command("A,3"); tuner_command("R,0,4,100");
    loops=0; while(!data_complete && loops<4000){ run_for(1); loops++; }
    CHECK(data_sp_roll[60]==4 && data_sp_pitch[60]==4, "BOTH step test: both setpoints step 0 -> 4");
    nl=0; while(data_complete) tuner_send_run();
    { int commas=0; for(char *c=dump[60];*c;c++) if(*c==',') commas++;
      CHECK(commas==18, "BOTH dump line: 19 fields (adds sp_roll, sp_pitch, ESC 1-4, RC, PC)"); }
    save_dump("pc_both.txt");
    tuner_command("X"); off_reason=0;

    /* --- v3.1: live +/- per motor (O command, the app's motor buttons) --- */
    tuner_command("O"); CHECK(strcmp(last,"OFFS,0,0,0,0")==0, "O alone reports the offsets (OFFS,0,0,0,0 at power-up)");
    tuner_command("O,300,-300,0.4,-7.6"); CHECK(strcmp(last,"OFFS,200,-200,0,-8")==0, "O: held inside +/-200 us, rounded to whole us");
    tuner_command("O,50,50,50,50"); run_for(5);
    CHECK(OC1RS==938 && OC4RS==938, "motors off: an offset never starts a motor (1000 us = 938 ticks)");
    arm_and_wait(); tuner_command("B,1500"); tuner_command("A,3"); tuner_command("S,0,0"); tuner_command("O,0,0,0,0");
    tuner_command("G"); run_for(1300);
    CHECK(us_of(OC1RS)==1500 && us_of(OC2RS)==1500 && us_of(OC3RS)==1500 && us_of(OC4RS)==1500, "level PID hold, no offsets: all four at T = 1500");
    tuner_command("O,10,-10,-10,10"); run_for(1);                   /* while flying: the yaw pair */
    CHECK(us_of(OC1RS)==1510 && us_of(OC2RS)==1490 && us_of(OC3RS)==1490 && us_of(OC4RS)==1510,
          "O,10,-10,-10,10 while flying: FL 1510 FR 1490 RL 1490 RR 1510 on the next loop");
    CHECK(near(motor_us[1],1500) && near(roll_comp,0) && near(pitch_comp,0),
          "the offsets come after the mixer: RC = PC = 0, left/right and front/back sums unchanged");
    tuner_command("X"); off_reason=0; tuner_command("O");
    CHECK(strcmp(last,"OFFS,10,-10,-10,10")==0 && OC1RS==938 && OC4RS==938, "KILL stops the motors (938 ticks = 1000 us) but keeps the offsets");
    arm_and_wait(); tuner_command("B,1150"); tuner_command("O,-200,0,0,0"); tuner_command("G"); run_for(1300);
    CHECK(us_of(OC1RS)==1100, "PID hold: a big minus can't take a motor under the 1100 us idle floor");
    tuner_command("X"); off_reason=0;
    arm_and_wait(); tuner_command("O,-20,0,0,15"); tuner_command("M,1300"); run_for(800);
    CHECK(us_of(OC1RS)==1280 && us_of(OC2RS)==1300 && us_of(OC4RS)==1315, "manual M,1300 with O,-20,0,0,15: FL 1280, FR 1300, RR 1315");
    tuner_command("X"); off_reason=0; tuner_command("O,0,0,0,0"); tuner_command("B,1500");

    /* --- trims: a stop stays 1000 us; the formula is 1000 + g (p - 1000) + o --- */
    CHECK(trim(1,1000)==1000 && trim(3,1500)==1500, "default trims: no change, stop stays 1000 us");

    printf(fails? "\n%d FAILED\n":"\nall passed\n", fails);
    return fails;
}
