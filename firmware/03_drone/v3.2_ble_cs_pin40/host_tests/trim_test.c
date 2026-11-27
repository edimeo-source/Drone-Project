/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.2_ble_cs_pin40/host_tests/trim_test.c
 *  What    : Host test: the per-motor trims (built with test trim values).
 *  Version : v3.2   |   Date: 2026-11-26
 *  Status  : CURRENT. v3.1 with the Bluefruit CS moved from pin 13 (RJ13) to pin 40 (RJ3);
 *            nothing else changed. Host tests pass (87 checks; the Bluetooth simulation checks
 *            CS on RJ3 for every byte). Not yet built in MPLAB.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): putsU2(), main()
 * ============================================================================================ */

/* v3.0 host test: per-motor trims (compiled with test trim values) */
#include "p32xxxx.h"
#include <stdio.h>
#include "control.h"
#include "motors.h"
float trim(int n, float us);
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Test stand-in for the UART: throws the text away.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ (void)s; }
static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s\n",msg); fails++;} }while(0)
// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void){
    CHECK(trim(1,1500)>1521.99f && trim(1,1500)<1522.01f, "FL gain 1.05, offset -3: 1500 us -> 1522 us = 1000 + 1.05 x 500 - 3");
    CHECK(trim(1,1000)==1000, "a stop stays exactly 1000 us");
    CHECK(trim(4,1200)>1193.99f && trim(4,1200)<1194.01f, "RR gain 0.97, offset 0: 1200 us -> 1194 us");
    motor_mode=1; kill_latched=0; control_ready=1;
    set_motor(1, 1500);
    CHECK(OC1RS==1427, "set_motor applies the trim: 1522 us = 1427 ticks on OC1");
    set_motor(1, 1990);
    CHECK(OC1RS==1875, "a trimmed pulse is still capped at 2000 us (1875 ticks)");
    printf(fails? "\n%d FAILED\n":"\nall passed\n", fails);
    return fails;
}
