/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/host_tests/fw_test.c
 *  What    : Host test: arming, relay, heartbeat, motor output and PC data (runs on a PC with
 *            gcc).
 *  Version : v2.0   |   Date: 2026-11-23
 *  Status  : CURRENT. Host tests pass (simulated Bluefruit, arming, relay, heartbeat, cutoff).
 *            Not yet compiled with XC32 or flown.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): putsU2(), tick(), main()
 * ============================================================================================ */

#include "p32xxxx.h"
#include <stdio.h>
#include <string.h>
#include "control.h"
#include "motors.h"
#include "imu.h"
#include "tuner.h"
void Controller(void);
extern volatile int data_complete; extern volatile int run_total;
extern float data_angle[];
static int nlines=0; static char lastline[160];
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Sends a text line to the PC over UART2, followed by a new line.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ nlines++; strncpy(lastline,s,159); }
static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s\n",msg); fails++;} }while(0)
extern int check_count;
// ---------------------------------------------------------------------------------------------
// static void tick(void)
//   Test helper: moves the simulated clock 4 ms forward and runs the control interrupt once.
// ---------------------------------------------------------------------------------------------
/* the stand-in SPI returns 0 for WHO_AM_I, so skip the once-a-second IMU check */
static void tick(void){ cp0+=4u*60000u; check_count=0; Controller(); }
// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void){
    SPI1STATbits.SPIRBF=1; PORTBbits.RB12=1; IEC0bits.T4IE=1;
    control_ready=1; imu_error=0; cp0=1000;   /* no Bluetooth at all: a PC (USB) session */

    /* 1. USB session, no Bluetooth central: motors must follow the PID/manual pulse */
    CHECK(control_arm(2)==1 && LATKbits.LATK4==1, "USB ARM closes relay (RK4 high)");
    CHECK(control_can_start()==0, "start refused during the 2 s ESC startup");
    for(int i=0;i<520;i++){ if(i%25==0) control_heartbeat(2); tick(); }   /* 2.08 s, H every 100 ms */
    CHECK(control_can_start()==1, "start allowed after 2 s with heartbeats");
    tuner_command("M,1400,1300");
    for(int i=0;i<300;i++){ if(i%25==0) control_heartbeat(2); tick(); }   /* soft start done */
    CHECK(OC1RS==1313 && OC3RS==1219, "manual 1400/1300 us reach OC1/OC3 with no Bluetooth (1313/1219 ticks)");

    /* 2. moving the manual sliders sends M again while already in manual mode */
    tuner_command("M,1500,1500");
    for(int i=0;i<10;i++){ if(i%25==0) control_heartbeat(2); tick(); }
    CHECK(motor_mode==2 && OC1RS==1406, "second M accepted while running (1500 us = 1406 ticks)");

    /* 3. heartbeat timeout: stop sending H -> motors off and relay open within 350 ms + 1 loop */
    control_heartbeat(2); int t=0;
    while(!kill_latched && t<200){ tick(); t++; }
    CHECK(kill_latched==1 && LATKbits.LATK4==0 && OC1RS==938, "no H: motors 1000 us + relay open");
    printf("      tripped after %d ms (limit 500 ms)\n", t*4);
    CHECK(off_reason==OFF_BLE, "off_reason = heartbeat/link lost");

    /* 4. step-test dump: one line per call, so main keeps reading commands */
    run_total=5; data_complete=1; nlines=0; int calls=0;
    while(data_complete && calls<50){ int before=nlines; tuner_send_run(); calls++; if(nlines-before!=1){printf("call %d sent %d lines\n",calls,nlines-before); fails++;} }
    CHECK(calls==7 && strcmp(lastline,"RUN_END")==0, "dump of 5 samples = 7 calls of 1 line (RUN_BEGIN, 5 D, RUN_END)");

    printf(fails? "\n%d FAILED\n":"\nall passed\n", fails);
    return fails;
}
