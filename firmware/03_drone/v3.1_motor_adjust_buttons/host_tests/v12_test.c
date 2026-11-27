/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/host_tests/v12_test.c
 *  What    : Host test: the 60 deg tip cutoff and the end of a recorded run.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : Built in MPLAB on the laptop (Nov 25). v3.0 plus the O command. Host tests pass
 *            (87 checks). Replaced by v3.2 (same code, Bluefruit CS on pin 40).
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): putsU2(), tick(), run_for(),
 *    arm_and_wait(), main()
 * ============================================================================================ */

#include "p32xxxx.h"
#include <stdio.h>
#include <string.h>
#include "control.h"
#include "motors.h"
#include "imu.h"
#include "tuner.h"
void Controller(void);
extern volatile int data_complete;
extern int check_count;
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Test stand-in for the UART: throws the text away.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ (void)s; }
static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s\n",msg); fails++;} }while(0)
/* The stand-in IMU reads 0, so one filter update gives 0.98 x the previous angle.
   The test sets angle / 0.98 so the value the tip check sees is exactly the target. */
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
static void arm_and_wait(void){ tick(); control_heartbeat(2); control_arm(2); run_for(520); }   /* tick first: angles take the new values */
// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void){
    SPI1STATbits.SPIRBF=1; PORTBbits.RB12=1; IEC0bits.T4IE=1; control_ready=1; cp0=1000;
    /* imu_update overwrites angle/roll each loop; it reads 0 here, so these checks
       drive the tip test through the limit values directly */
    arm_and_wait(); tuner_command("G");
    hold_pitch=59.9f; run_for(5);
    CHECK(kill_latched==0 && motor_mode==1, "pitch +59.9 deg: still running");
    hold_pitch=-59.9f; run_for(5);
    CHECK(kill_latched==0, "pitch -59.9 deg: still running");
    hold_roll=59.9f; run_for(5);
    CHECK(kill_latched==0, "roll +59.9 deg: still running");
    hold_roll=0;
    hold_pitch=60.1f; run_for(1);
    CHECK(kill_latched==1 && off_reason==OFF_TIP && LATKbits.LATK4==0, "pitch +60.1 deg: motors off, relay open, reason = tip");
    hold_pitch=0; off_reason=0; arm_and_wait(); tuner_command("G");
    hold_roll=-60.1f; run_for(1);
    CHECK(kill_latched==1 && off_reason==OFF_TIP, "roll -60.1 deg: motors off");
    hold_roll=0; off_reason=0; arm_and_wait(); tuner_command("G");
    hold_pitch=-60.1f; run_for(1);
    CHECK(kill_latched==1 && off_reason==OFF_TIP, "pitch -60.1 deg: motors off");
    hold_pitch=0; off_reason=0; hold_roll=55.0f; tick(); control_heartbeat(2);
    CHECK(control_arm(2)==1, "ARM accepted at roll 55 deg (inside 60)");
    hold_roll=0; tuner_command("X"); run_for(1); off_reason=0;
    hold_roll=0; off_reason=0;

    /* professor: record the whole run into the arrays, stop the motors, then send */
    arm_and_wait();
    tuner_command("R,0,10,100");            /* 0.1 s after the step -> 50 + 25 samples (after the 5 s soft start + 1 s hold) */
    CHECK(motor_mode==1, "step test starts the PID");
    int loops=0; while(!data_complete && loops<4000){ run_for(1); loops++; }
    CHECK(data_complete==1, "recording finished and flagged complete");
    run_for(1);
    CHECK(motor_mode==0 && OC1RS==938 && OC3RS==938, "motors at 1000 us once the run ends");
    CHECK(kill_latched==0 && LATKbits.LATK4==1, "still armed, relay still closed (no re-ARM needed)");
    tuner_command("R,0,10,100");
    CHECK(motor_mode==0, "next run refused until the data has been sent");
    printf(fails? "\n%d FAILED\n":"\nall passed\n", fails);
    return fails;
}
