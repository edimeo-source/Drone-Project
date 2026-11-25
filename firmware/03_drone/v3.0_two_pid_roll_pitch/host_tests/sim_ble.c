/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/host_tests/sim_ble.c
 *  What    : Host test: the Bluetooth driver against a simulated Bluefruit module.
 *  Version : v3.0   |   Date: 2026-11-25
 *  Status  : CURRENT. Built from v2.0 (which builds and flew on the tether). Host tests pass
 *            (78 checks, including a simulated flight with the real 4 ms loop). Not yet built
 *            in MPLAB or flown.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): sim_cp0(), answer(),
 *    command_done(), sim_stat(), putsU2(), run_ms(), main()
 * ============================================================================================ */

// Host test: firmware v2.0 Bluetooth driver against a SIMULATED Bluefruit (SDEP over SPI).
// The fake module answers byte by byte like the real one: 0xFE while busy, 0x20 response packets
// of up to 16 data bytes with a "more" bit, AT replies ending in OK / ERROR.
#include "p32xxxx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ---- simulated module, hooked into the SPI registers bluefruit.c uses ----
static volatile __SPI1STATbits_t sim_stat_v;
volatile unsigned int sim_buf;
volatile __SPI1STATbits_t *sim_stat(void);
// ---------------------------------------------------------------------------------------------
// unsigned int sim_cp0(void)
//   Simulated core timer for the host tests.
// ---------------------------------------------------------------------------------------------
unsigned int sim_cp0(void) { cp0 += 60; return cp0; }     // every read of the timer = 1 us later
#undef _CP0_GET_COUNT
#define _CP0_GET_COUNT() sim_cp0()
#define SPI1BUF sim_buf
#define SPI1STATbits (*sim_stat())
#include "../Drone.X/bluefruit.c"
#undef SPI1BUF
#undef SPI1STATbits

int connected=1, fifo_free=1024, fifo_supported=1, silent=0, resp_delay=0;
char phone_to_pic[1024]; int p2p=0;
char pic_to_phone[40000]; int pp=0;
int violations=0, packets=0;
static int state=0, idx=0, cmd_id, cmd_len_byte, cmd_n;
static char cmd[600]; static int cmdl=0;
static char resp[600]; static int resp_len=0, resp_pos=0, resp_id=0, resp_ready=0, resp_wait=0;
static unsigned char outp[20]; static int out_n=0;

// ---------------------------------------------------------------------------------------------
// static void answer(const char *t)
//   Test helper: gives the code under test a fake Bluefruit reply.
// ---------------------------------------------------------------------------------------------
static void answer(const char *t) { resp_len=strlen(t); memcpy(resp, t, resp_len); }
// ---------------------------------------------------------------------------------------------
// static void command_done(void)
//   Simulated Bluefruit: a whole command has arrived, so prepare the reply a real module would
//   give.
// ---------------------------------------------------------------------------------------------
static void command_done(void)
{
    char t[64];
    cmd[cmdl]=0; resp_len=0; resp_pos=0; resp_id=cmd_id; resp_ready=1; resp_wait=resp_delay;
    if (cmd_id==0x0A00){
        if (!strcmp(cmd,"ATE=0") || !strcmp(cmd,"AT") || !strncmp(cmd,"AT+GAPINTERVALS=",16)) answer("OK\r\n");
        else if (!strcmp(cmd,"ATI")) answer("BLESPIFRIEND\r\nnRF51822 QFACA10\r\n0.8.1\r\nOK\r\n");
        else if (!strcmp(cmd,"AT+GAPGETCONN")) answer(connected ? "1\r\nOK\r\n" : "0\r\nOK\r\n");
        else if (!strcmp(cmd,"AT+BLEUARTFIFO=TX")){ if (fifo_supported){ sprintf(t,"%d\r\nOK\r\n",fifo_free); answer(t);} else answer("ERROR\r\n"); }
        else answer("ERROR\r\n");
    }
    else if (cmd_id==0x0A01){ memcpy(pic_to_phone+pp, cmd, cmdl); pp+=cmdl; fifo_free-=cmdl; resp_len=0; }
    else if (cmd_id==0x0A02){ memcpy(resp, phone_to_pic, p2p); resp_len=p2p; p2p=0; }
    cmdl=0;
}
// ---------------------------------------------------------------------------------------------
// volatile __SPI1STATbits_t *sim_stat(void)
//   Simulated SPI1 status register: every byte the driver sends goes through the simulated
//   Bluefruit.
// ---------------------------------------------------------------------------------------------
volatile __SPI1STATbits_t *sim_stat(void)
{
    int b=sim_buf&0xFF, r=0;
    if (LATJbits.LATJ13!=0 || IEC0bits.T4IE!=0 || SPI1CON!=0x8120 || SPI1BRG!=29 || TMR4>2800) violations++;
    if (silent) r=0xFE;
    else if (state==0){
        packets++;
        if (b==0x10){ r=0x00; state=1; idx=1; }
        else if (b==0xFF){
            if (resp_ready && resp_wait>0){ resp_wait--; r=0xFE; }
            else if (resp_ready){
                int n=resp_len-resp_pos; if (n>16) n=16;
                out_n=0; outp[out_n++]=resp_id&0xFF; outp[out_n++]=resp_id>>8;
                outp[out_n++]=n | ((resp_len-resp_pos>16)?0x80:0);
                memcpy(outp+out_n, resp+resp_pos, n); out_n+=n; resp_pos+=n;
                if (resp_pos>=resp_len) resp_ready=0;
                r=0x20; state=3; idx=0;
            }
            else r=0xFE;
        }
    }
    else if (state==1){                 // command header
        if (idx==1) cmd_id=b;
        else if (idx==2) cmd_id|=b<<8;
        else if (idx==3){
            cmd_len_byte=b; cmd_n=b&0x7F;
            if (cmd_n==0){ state=0; if (!(b&0x80)) command_done(); }
            else state=2;
        }
        idx++;
    }
    else if (state==2){                 // command data
        cmd[cmdl++]=b; cmd_n--;
        if (cmd_n==0){ state=0; if (!(cmd_len_byte&0x80)) command_done(); }
    }
    else if (state==3){                 // response bytes after the type byte
        r=outp[idx++]; if (idx>=out_n) state=0;
    }
    sim_buf=r; sim_stat_v.SPIRBF=1; return &sim_stat_v;
}

// ---- capture what the firmware prints to PuTTY ----
char usb[8000]; int usbl=0;
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Test stand-in for the UART: collects everything printed to the PC in usb[].
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ int n=strlen(s); if (usbl+n+2<8000){ memcpy(usb+usbl,s,n); usbl+=n; usb[usbl++]='\n'; usb[usbl]=0; } }

extern float data_pitch[], data_pc[], data_roll[], data_rc[]; extern int run_axis;
extern int run_phone, dump_k;
extern volatile int run_total, data_complete;
static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s\n",msg); fails++;} }while(0)
// ---------------------------------------------------------------------------------------------
// static void run_ms(int ms)
//   Test helper: runs the firmware's main-loop work for ms simulated milliseconds.
// ---------------------------------------------------------------------------------------------
static void run_ms(int ms){ unsigned int t=cp0; while ((cp0-t)/60000 < (unsigned)ms) ble_service(); }

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char line[48]; int n=0;
    T4CONbits.ON=1; IEC0bits.T4IE=1; TMR4=100; SPI1CON=0x8060; SPI1BRG=14; LATJbits.LATJ13=1;

    run_ms(50);
    CHECK(ble_ok==1 && strstr(usb,"Bluefruit OK") && strstr(usb,"0.8.1"), "start-up: ATE=0, AT, ATI answered (firmware version printed)");
    CHECK(SPI1CON==0x8060 && SPI1BRG==14 && IEC0bits.T4IE==1, "after Bluetooth traffic: IMU SPI settings and the 4 ms interrupt are back");
    run_ms(250);
    CHECK(ble_connected==1 && strstr(usb,"Phone connected"), "link check every 100 ms: phone connected");

    strcpy(phone_to_pic, "ARM;H;K,1.5,0,0.2\nstop;"); p2p=strlen(phone_to_pic);
    run_ms(60);
    char got[4][48]; while (n<4 && ble_get_line(line)) strcpy(got[n++], line);
    CHECK(n==4 && !strcmp(got[0],"ARM") && !strcmp(got[1],"H") && !strcmp(got[2],"K,1.5,0,0.2") && !strcmp(got[3],"STOP"),
          "phone text split into commands at ; and newline (lowercase stop -> STOP)");

    ble_print("A,+1.5000,+0.0000,+0.2000"); run_ms(30);
    CHECK(strstr(pic_to_phone,"A,+1.5000,+0.0000,+0.2000\n")!=0, "a reply reaches the phone with its newline");

    fifo_free=20; pp=0; memset(pic_to_phone,0,100);
    ble_print("THIS LINE IS LONGER THAN THE ROOM LEFT"); run_ms(100);
    CHECK(pp==0, "module FIFO nearly full: the PIC waits instead of sending");
    fifo_free=1024; run_ms(50);
    CHECK(pp>0 && strstr(pic_to_phone,"THIS LINE"), "room again: the line is sent");

    fifo_supported=0; pp=0; ble_print("OLD FIRMWARE TEST"); run_ms(50);
    CHECK(fifo_check==0 && strstr(pic_to_phone,"OLD FIRMWARE TEST"), "no FIFO command (ERROR): falls back to a fixed pace and still sends");
    fifo_supported=1;

    // step-test data round trip: 58 samples (0.2 s before + 32 after), real values
    run_total=58; run_phone=1; dump_k=-1;
    for (int k=0;k<58;k++){ data_pitch[k]=(k<50?0:10.0f*(1-expf(-(k-50)/8.0f)))+0.037f*sinf(k); data_pc[k]=-3.21f*k+40.6f; }
    data_complete=1; pp=0;
    unsigned int t0=cp0;
    while (data_complete || tx_count>0){ if (data_complete) tuner_send_run_phone(); ble_service(); if ((cp0-t0)/60000>20000) break; }
    pic_to_phone[pp]=0;
    int ok=1, count=0; char *p=strstr(pic_to_phone,"RUN_BEGIN,58,50,");
    if (!p || !strstr(pic_to_phone,"RUN_END")) ok=0;
    for (char *q=strstr(pic_to_phone,"\nRD,"); q; q=strstr(q+1,"\nRD,")){
        int k=atoi(q+4); char *c=strchr(q+4,',');
        while (c && *c==',' && k<58){
            int a=atoi(c+1); c=strchr(c+1,','); int u=atoi(c+1); c=strpbrk(c+1,",\n");
            if (a!=(int)lroundf(data_pitch[k]*10) || u!=(int)lroundf(data_pc[k])) ok=0;
            k++; count++;
            if (!c || *c=='\n') break;
        }
    }
    CHECK(ok && count==58, "step-test arrays: RUN_BEGIN + RD lines + RUN_END, all 58 samples back exactly (0.1 deg, 1 us)");
    printf("      one axis: %d bytes for 58 samples = %.1f bytes per sample, %u ms at the old-firmware pace\n", pp, pp/58.0, (cp0-t0)/60000);

    // BOTH mode (v3.0): 4 values a sample (roll, pitch, RC, PC), 4 samples a line
    run_axis=3; run_total=58; dump_k=-1;
    for (int k=0;k<58;k++){ data_roll[k]=3.3f*sinf(k*0.3f); data_pitch[k]=-2.1f*cosf(k*0.2f); data_rc[k]=12.4f-0.5f*k; data_pc[k]=0.7f*k-9.3f; }
    data_complete=1; pp=0; memset(pic_to_phone,0,sizeof(pic_to_phone)); t0=cp0;
    while (data_complete || tx_count>0){ if (data_complete) tuner_send_run_phone(); ble_service(); if ((cp0-t0)/60000>40000) break; }
    pic_to_phone[pp]=0;
    ok=1; count=0; p=strstr(pic_to_phone,"RUN_BEGIN,58,50,");
    if (!p || !strstr(pic_to_phone,"RUN_END")) ok=0;
    { int commas=0; for (char *c=p; c && *c && *c!='\n'; c++) if (*c==',') commas++; if (commas!=13) ok=0; }   // 14 fields, axis = 3
    for (char *q=strstr(pic_to_phone,"\nRD,"); q; q=strstr(q+1,"\nRD,")){
        int k=atoi(q+4); char *c=strchr(q+4,',');
        while (c && *c==',' && k<58){
            int r=atoi(c+1); c=strchr(c+1,','); int pi=atoi(c+1); c=strchr(c+1,',');
            int rc=atoi(c+1); c=strchr(c+1,','); int pc=atoi(c+1); c=strpbrk(c+1,",\n");
            if (r!=(int)lroundf(data_roll[k]*10) || pi!=(int)lroundf(data_pitch[k]*10) ||
                rc!=(int)lroundf(data_rc[k]) || pc!=(int)lroundf(data_pc[k])) ok=0;
            k++; count++;
            if (!c || *c=='\n') break;
        }
    }
    CHECK(ok && count==58, "BOTH-mode step test to the phone: roll, pitch, RC, PC for all 58 samples back exactly");
    if (getenv("TUNER_DUMP")){ char fn[300]; snprintf(fn,300,"%s/phone_both.txt",getenv("TUNER_DUMP"));   // for the PC tuner test
        FILE *f=fopen(fn,"w"); if (f){ fputs(pic_to_phone,f); fclose(f);} }
    printf("      BOTH mode: %d bytes for 58 samples = %.1f bytes per sample\n", pp, pp/58.0);
    run_axis=2;

    // disconnect while the phone has the drone armed
    kill_latched=0; control_link_source=1; LATKbits.LATK4=1; motor_mode=1;
    connected=0; run_ms(150);
    CHECK(ble_connected==0 && kill_latched==1 && LATKbits.LATK4==0 && motor_mode==0, "phone disconnects while armed: motors off, relay open");

    // module stops answering
    connected=1; run_ms(150); kill_latched=0; control_link_source=1; LATKbits.LATK4=1;
    silent=1; run_ms(800);
    CHECK(ble_ok==0 && kill_latched==1 && LATKbits.LATK4==0, "Bluefruit stops answering while armed: motors off, relay open");
    silent=0; run_ms(1200);
    CHECK(ble_ok==1, "module answers again: driver restarts by itself");

    resp_delay=3; run_ms(300); pp=0; ble_print("SLOW MODULE"); run_ms(200);
    CHECK(strstr(pic_to_phone,"SLOW MODULE")!=0, "module slow to answer (0xFE several times): still works");

    CHECK(violations==0, "EVERY Bluefruit byte: its CS low, 4 ms interrupt held off, mode 0 / 1 MHz, started in the safe window");
    printf("      %d SPI packets checked\n", packets);
    printf(fails? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails;
}
