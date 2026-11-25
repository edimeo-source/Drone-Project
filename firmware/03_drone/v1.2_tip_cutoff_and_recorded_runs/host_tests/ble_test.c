/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs/host_tests/ble_test.c
 *  What    : Host test: the Bluetooth send path (queue, FIFO room, replies).
 *  Version : v1.2   |   Date: 2026-11-22
 *  Status  : Host tests pass. Not built with XC32.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): putsU2(), idle(), step(),
 *    wait20(), answer(), main()
 * ============================================================================================ */

#include "p32xxxx.h"
#include <stdio.h>
#include <string.h>
#include "bluefruit.h"
#include "control.h"
/* internals of bluefruit.c (compiled with -Dstatic= for this test) */
extern int started, phase, command_kind, fifo_free, fifo_supported, txq_count;
extern unsigned int next_service, last_query, last_rx, reply_len, tx_pos;
extern char command[], reply[];
void service_step(void); void finish_command(void);
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Test stand-in for the UART: throws the text away.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ (void)s; }
static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s   [command=%s]\n",msg,command); fails++;} }while(0)
// ---------------------------------------------------------------------------------------------
// static void idle(void)
//   Test helper: pretend the link and receive checks were just done.
// ---------------------------------------------------------------------------------------------
static void idle(void){ phase=3; last_query=cp0; last_rx=cp0; }        /* link + RX checks just done */
// ---------------------------------------------------------------------------------------------
// static void step(void)
//   Test helper: moves the clock 2 ms forward and runs one Bluetooth service step.
// ---------------------------------------------------------------------------------------------
static void step(void){ cp0+=2u*60000u; idle(); service_step(); }
// ---------------------------------------------------------------------------------------------
// static void wait20(void)
//   Test helper: moves the clock 21 ms forward (the FIFO is asked at most every 20 ms).
// ---------------------------------------------------------------------------------------------
static void wait20(void){ cp0+=21u*60000u; }   /* FIFO is asked at most every 20 ms */
// ---------------------------------------------------------------------------------------------
// static void answer(const char *text)
//   Test helper: gives the code under test a fake Bluefruit reply.
// ---------------------------------------------------------------------------------------------
static void answer(const char *text){ strcpy(reply,text); reply_len=strlen(text); finish_command(); }
// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void){
    initBluetooth(); ble_ready=1; ble_connected=1; cp0=5000000;
    CHECK(bluetooth_send_live("D,0,1.00,0.00,1500,1.0,0.0,0.0,1500")==1, "live line queued when queue empty");
    CHECK(bluetooth_send_live("D,4,1.00,0.00,1500,1.0,0.0,0.0,1500")==0, "second live line refused while one waits");
    step(); CHECK(strcmp(command,"AT+BLEUARTFIFO=TX")==0, "asks the module for FIFO room first");
    answer("1024\r\nOK\r\n"); CHECK(fifo_free==1024 && phase==3, "FIFO reply 1024 parsed");
    step(); CHECK(strncmp(command,"AT+BLEUARTTX=D,0,",17)==0 && strstr(command,"\\r\\n"), "then sends the line with \\r\\n escapes");
    CHECK(fifo_free==1024-37, "room estimate reduced by the line + CR LF");
    answer("ERROR\r\n"); CHECK(ble_ready==1 && phase==3 && fifo_free==0, "TX answered ERROR: module NOT treated as dead");
    bluetooth_send_text("DISARMED,KILL");
    wait20(); step(); CHECK(strcmp(command,"AT+BLEUARTFIFO=TX")==0, "after an ERROR it asks for room again");
    answer("12\r\nOK\r\n"); step();
    CHECK(txq_count==1 && strstr(command,"BLEUARTTX")==0, "no room: status line waits (not dropped)");
    wait20(); step(); answer("1000\r\nOK\r\n");
    step(); CHECK(strcmp(command,"AT+BLEUARTTX=DISARMED,KILL\\r\\n")==0, "room again: status line sent");
    answer("OK\r\n");
    bluetooth_send_live("D,8,1.00,0.00,1500,1.0,0.0,0.0,1500"); fifo_free=5;
    wait20(); step(); answer("5\r\nOK\r\n"); step();
    CHECK(txq_count==0, "no room: live line dropped instead of blocking");
    bluetooth_send_text("A,1.0000,0.0000,0.0000"); fifo_free=-1;
    wait20(); step(); answer("ERROR\r\n");
    CHECK(fifo_supported==0 && ble_ready==1, "old firmware (FIFO command ERROR): stops asking, link kept");
    step(); CHECK(strcmp(command,"AT+BLEUARTTX=A,1.0000,0.0000,0.0000\\r\\n")==0, "then sends without asking");
    for(int i=0;i<9;i++) bluetooth_send_text("STATUS: armed, motors running");
    CHECK(txq_count==8, "queue holds 8 lines, extra refused");
    printf(fails? "\n%d FAILED\n":"\nall passed\n", fails);
    return fails;
}
