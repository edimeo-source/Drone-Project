/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/Drone.X/bluefruit.c
 *  What    : Adafruit Bluefruit LE SPI Friend driver (SDEP over SPI1, shared with the IMU): the
 *            phone link.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : Built in MPLAB on the laptop (Nov 25). v3.0 plus the O command. Host tests pass
 *            (87 checks). Replaced by v3.2 (same code, Bluefruit CS on pin 40).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Bluefruit SPI Friend (shares SPI1 with the IMU through splitter wires)
 *      VIN pin 2 (+5 V, the module has its own 3.3 V regulator)   GND any J12 GND pin (e.g.
 *      9)
 *      SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 13 (RJ13)   IRQ and RST not connected
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): ms_since(), wait_us(),
 *    packet_start(), packet_end(), spi(), send_packet(), read_packet(), ble_cmd(), ble_at(),
 *    lost(), failed(), ble_start(), phone_char(), ble_get_line(), ble_room(), ble_print(),
 *    send_some(), ble_service()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include "bluefruit.h"
# include "control.h"
# include "uart.h"

// Bluefruit LE SPI Friend (nRF51822) on SPI1, shared with the IMU.
//
// HOW THE PIC TALKS TO IT: Adafruit's "SDEP" packets over SPI (same as Labs 5 and 6).
// One packet is at most 20 bytes:
//   byte 0      type: 0x10 = command (PIC -> module), 0x20 = response (module -> PIC)
//   bytes 1-2   command ID, low byte first:
//                 0x0A00 = AT command      (link check "AT+GAPGETCONN", FIFO check)
//                 0x0A01 = UART TX         (text to the phone)
//                 0x0A02 = UART RX         (text the phone sent)
//   byte 3      data length 0..16, bit 7 = 1 means "more packets follow"
//   bytes 4..   the data (16 bytes at most)
// While the module is busy it answers 0xFE (or 0xFF) instead of a packet: wait and try again.
//
// SPI SETTINGS: the two chips want different settings, so each Bluefruit packet sets
// its own and puts the IMU's back afterwards:
//   Bluefruit: SPI mode 0, 1 MHz   (SPI1CON = 0x8120, SPI1BRG = 29, the Lab 5/6 setting)
//   IMU:       SPI mode 3, 2 MHz   (SPI1CON = 0x8060, SPI1BRG = 14, imu.c)
//   1 MHz: 60 MHz / (2 x (29 + 1)) = 1 MHz, so one byte takes 8 us.
//
// IMU FIRST (the IMU is read inside the 4 ms interrupt):
//   - a Bluefruit packet only STARTS while TMR4 <= 2800, i.e. in the first 2.99 ms of the
//     4 ms period (TMR4 counts 937,500 per second: 2800 / 937,500 = 2.99 ms);
//   - the 4 ms interrupt is held off during that ONE packet (~0.3 ms: 110 us wait after CS
//     goes low + 20 bytes x 8 us = 270 us), so the packet always ends before the next
//     interrupt is due (2.99 + 0.3 = 3.3 ms < 4 ms). The IMU read is never late or skipped.
//
// SPEED: the PIC <-> module wire runs at 1 MHz, but the radio link to the phone is the slow
// part. People who measured this module got about 100-300 bytes per second (the BLE UART
// sends 20-byte packets). So the phone only gets short lines: commands, a 4 Hz pitch/roll
// line while the motors are off, and the step-test arrays after a run.
//
// main() calls ble_service() every pass. It is simple and "blocking": each exchange waits
// for the module's answer (a few ms, 500 ms at most). Only main() waits; the 4 ms control
// loop keeps running in its interrupt the whole time.

#define CMD_AT      0x0A00
#define CMD_UART_TX 0x0A01
#define CMD_UART_RX 0x0A02

#define LINK_EVERY_MS 100       // ask "is a phone connected?" 10 times a second
#define RX_EVERY_MS   20        // read the phone's text 50 times a second
#define RETRY_MS      1000      // module not answering: try again every 1 s
#define TX_CHUNK      64        // most bytes sent to the phone in one go (4 packets)
#define TX_SIZE       1024      // send queue (ring buffer)
#define LINES         8         // finished phone commands waiting for main()
#define LINE_LEN      48

volatile int ble_ok=0;
volatile int ble_connected=0;

static char reply[300];                 // the module's answer to the last command
static int reply_len=0;
static char tx_buf[TX_SIZE];            // text waiting to go to the phone
static int tx_head=0, tx_count=0;
static char line_now[LINE_LEN];         // phone command being received
static int line_len=0;
static char lines[LINES][LINE_LEN];     // finished phone commands
static int lines_head=0, lines_count=0;
static int fifo_check=1;                // 1 = the module has AT+BLEUARTFIFO (firmware 0.6.7 or newer)
static unsigned int t_link, t_rx, t_tx, t_retry;
static unsigned int tx_wait=0;          // ms to wait before the next send
static int t4_was_on;
static int tried=0;                     // 1 = ble_start() has run at least once

// ---------- time (core timer: 120 MHz / 2 = 60,000 counts per ms) ----------

// ---------------------------------------------------------------------------------------------
// static unsigned int ms_since(unsigned int t)
//   Returns how many milliseconds have passed since core-timer count t (safe across the timer
//   wrap).
// ---------------------------------------------------------------------------------------------
static unsigned int ms_since(unsigned int t)
{
    return (_CP0_GET_COUNT()-t)/60000;  // unsigned subtraction works across the timer wrap
}

// ---------------------------------------------------------------------------------------------
// static void wait_us(unsigned int us)
//   Waits us microseconds on the core timer (60 counts per microsecond).
// ---------------------------------------------------------------------------------------------
static void wait_us(unsigned int us)
{
    unsigned int t=_CP0_GET_COUNT();
    while (_CP0_GET_COUNT()-t < us*60);
}

// ---------- one SPI packet, with the IMU-first rule ----------

// ---------------------------------------------------------------------------------------------
// static void packet_start(void)
//   Starts one SPI packet the IMU-first way: wait until the 4 ms loop has just run, hold it
//   off, Bluefruit SPI settings, CS low.
// ---------------------------------------------------------------------------------------------
static void packet_start(void)
{
    if (T4CONbits.ON && IEC0bits.T4IE)
        while (TMR4>2800);              // wait (1 ms at most) until the 4 ms loop has just run
    t4_was_on=IEC0bits.T4IE;
    IEC0bits.T4IE=0;                    // hold the loop off for this one packet (~0.3 ms)
    SPI1CON=0;                          // Bluefruit settings: mode 0, 1 MHz
    SPI1BRG=29;
    SPI1CON=0x8120;
    LATJbits.LATJ13=0;                  // CS low (the IMU's CS stays high)
    wait_us(110);                       // the module needs 100 us after CS goes low
}

// ---------------------------------------------------------------------------------------------
// static void packet_end(void)
//   Ends the packet: CS high, IMU SPI settings back (mode 3, 2 MHz), 4 ms loop back on.
// ---------------------------------------------------------------------------------------------
static void packet_end(void)
{
    LATJbits.LATJ13=1;                  // CS high
    SPI1CON=0;                          // IMU settings back: mode 3, 2 MHz
    SPI1BRG=14;
    SPI1CON=0x8060;
    IEC0bits.T4IE=t4_was_on;            // the loop runs again
}

// ---------------------------------------------------------------------------------------------
// static int spi(int b)
//   One SPI1 byte out, one byte in (used while the Bluefruit is selected).
// ---------------------------------------------------------------------------------------------
static int spi(int b)
{
    SPI1BUF=b;
    while (!SPI1STATbits.SPIRBF);       // 8 clocks at 1 MHz = 8 us
    return SPI1BUF;
}

// ---------------------------------------------------------------------------------------------
// static int send_packet(int id, const char *data, int n, int more)
//   Sends one SDEP command packet (the next 16-byte piece of the command) to the Bluefruit over
//   SPI.
// ---------------------------------------------------------------------------------------------
// Send one command packet. 1 = sent, 0 = the module stayed busy for 50 ms.
static int send_packet(int id, const char *data, int n, int more)
{
    int tries, first, i;

    for (tries=0; tries<50; tries++){
        packet_start();
        first=spi(0x10);                // command packet; the module answers 0xFE if busy
        if (first!=0xFE){
            spi(id&0xFF);               // command ID, low byte first
            spi(id>>8);
            spi(n | (more ? 0x80 : 0)); // length + "more packets follow" bit
            for (i=0; i<n; i++) spi(data[i]);
            packet_end();
            return 1;
        }
        packet_end();
        wait_us(1000);                  // busy: try again in 1 ms
    }
    return 0;
}

// ---------------------------------------------------------------------------------------------
// static int read_packet(int id, int *more)
//   Reads one answer packet into reply[]. 1 = got one, 0 = not ready yet, -1 = error.
// ---------------------------------------------------------------------------------------------
// Read one answer packet into reply[]. 1 = got one, 0 = not ready yet, -1 = error.
static int read_packet(int id, int *more)
{
    int type, lo, hi, len, n, i, c;

    packet_start();
    type=spi(0xFF);
    if (type==0xFE || type==0xFF){      // nothing ready yet
        packet_end();
        return 0;
    }
    lo=spi(0xFF);
    hi=spi(0xFF);
    len=spi(0xFF);
    n=len&0x7F;
    if (n>16) n=16;
    for (i=0; i<n; i++){
        c=spi(0xFF);
        if (type==0x20 && reply_len<(int)sizeof(reply)-1) reply[reply_len++]=c;
    }
    packet_end();
    *more=len&0x80;
    if (type==0x40) return 0;           // an "alert" packet from the module: ignore it
    if (type!=0x20 || (lo|(hi<<8))!=id) return -1;  // error packet (0x80) or the wrong answer
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static int ble_cmd(int id, const char *data, int n)
//   Sends a whole command (split into 16-byte packets) and waits up to 500 ms for the full
//   answer.
// ---------------------------------------------------------------------------------------------
// Send a whole command (split into 16-byte packets) and wait up to 500 ms for the answer.
// The answer ends up in reply[] / reply_len. 1 = OK, 0 = failed.
static int ble_cmd(int id, const char *data, int n)
{
    int k, more=0, r;
    unsigned int t;

    do {
        k=(n>16) ? 16 : n;
        if (!send_packet(id, data, k, n>k)) return 0;
        data=data+k;
        n=n-k;
    } while (n>0);

    reply_len=0;
    t=_CP0_GET_COUNT();
    while (ms_since(t)<500){
        r=read_packet(id, &more);
        if (r<0) return 0;
        if (r==0){
            wait_us(1000);              // not ready: look again in 1 ms
            continue;
        }
        if (!more){
            reply[reply_len]=0;
            return 1;
        }
    }
    return 0;                           // no answer in 500 ms
}

// ---------------------------------------------------------------------------------------------
// static int ble_at(const char *cmd)
//   Sends an AT command. 1 = answered OK (reply[] holds the text before OK), 0 = ERROR, -1 = no
//   answer.
// ---------------------------------------------------------------------------------------------
// AT command. 1 = answered OK (reply[] = the text before "OK"), 0 = ERROR, -1 = no answer.
static int ble_at(const char *cmd)
{
    char *p;

    if (!ble_cmd(CMD_AT, cmd, strlen(cmd))) return -1;
    while (reply_len>0 && (reply[reply_len-1]=='\r' || reply[reply_len-1]=='\n')) reply_len--;
    reply[reply_len]=0;
    if (reply_len<2 || strcmp(reply+reply_len-2, "OK")!=0) return 0;
    reply_len=reply_len-2;              // cut "OK" and the line break before it
    while (reply_len>0 && (reply[reply_len-1]=='\r' || reply[reply_len-1]=='\n')) reply_len--;
    reply[reply_len]=0;
    p=reply;
    while (*p=='\r' || *p=='\n') p++;   // and any line break in front
    if (p!=reply) memmove(reply, p, strlen(p)+1);
    return 1;
}

// ---------- lost module / lost phone ----------

// ---------------------------------------------------------------------------------------------
// static void lost(char *why)
//   Phone link lost: motors off and relay open if the phone had armed the drone; prints why.
// ---------------------------------------------------------------------------------------------
static void lost(char *why)
{
    if (control_link_source==1){        // the phone had armed the drone: motors off, relay open
        motors_off();
        off_reason=OFF_BLE;
    }
    putsU2(why);
    ble_connected=0;
    tx_count=0;
    line_len=0;
}

// ---------------------------------------------------------------------------------------------
// static void failed(void)
//   The Bluefruit stopped answering: marks it down and retries every 1 s.
// ---------------------------------------------------------------------------------------------
static void failed(void)
{
    if (ble_ok) lost(" Bluefruit stopped answering. Motors off if the phone had armed them. Retrying every 1 s.");
    ble_ok=0;
    t_retry=_CP0_GET_COUNT();
}

// ---------- start-up ----------

// ---------------------------------------------------------------------------------------------
// static void ble_start(void)
//   Start-up: throws away old replies, then ATE=0, AT and ATI (prints the module's firmware
//   version).
// ---------------------------------------------------------------------------------------------
static void ble_start(void)
{
    int i, more;

    TRISJbits.TRISJ13=0;                // CS output, high = not selected
    LATJbits.LATJ13=1;
    for (i=0; i<20; i++){               // throw away any old answer still in the module
        reply_len=0;
        if (read_packet(CMD_AT, &more)==0) break;
    }
    if (ble_at("ATE=0")<0 || ble_at("AT")!=1){
        failed();
        return;
    }
    ble_ok=1;
    // Faster radio timing: connection interval 20-40 ms (Adafruit's default allows up to
    // 100 ms). A measured test with this setting tripled the module's data rate.
    ble_at("AT+GAPINTERVALS=20,40,25,30");
    fifo_check=1;
    if (ble_at("ATI")==1){              // module name and firmware version, for PuTTY
        putsU2(" Bluefruit OK:");
        putsU2(reply);
    }
    putsU2(" Bluefruit ready. Open the phone app and press Connect.");
    t_link=t_rx=t_tx=_CP0_GET_COUNT();
}

// ---------- phone text ----------

// ---------------------------------------------------------------------------------------------
// static void phone_char(int c)
//   Adds one character from the phone to the command being received (; CR LF end it, lowercase
//   accepted).
// ---------------------------------------------------------------------------------------------
static void phone_char(int c)
{
    if (c>='a' && c<='z') c=c-'a'+'A';  // accept "stop" as well as "STOP"
    if (c==';' || c=='\n' || c=='\r'){  // end of a command
        if (line_len>0 && lines_count<LINES){
            line_now[line_len]=0;
            strcpy(lines[(lines_head+lines_count)%LINES], line_now);
            lines_count++;
        }
        line_len=0;
    }
    else if (c!=' ' && line_len<LINE_LEN-1){
        line_now[line_len]=c;
        line_len++;
    }
}

// ---------------------------------------------------------------------------------------------
// int ble_get_line(char *line)
//   Copies the next complete command the phone sent into line. 1 = got one.
// ---------------------------------------------------------------------------------------------
int ble_get_line(char *line)
{
    if (lines_count==0) return 0;
    strcpy(line, lines[lines_head]);
    lines_head=(lines_head+1)%LINES;
    lines_count--;
    return 1;
}

// ---------------------------------------------------------------------------------------------
// int ble_room(void)
//   Returns how many characters still fit in the phone send buffer.
// ---------------------------------------------------------------------------------------------
int ble_room(void)
{
    return TX_SIZE-tx_count;
}

// ---------------------------------------------------------------------------------------------
// int ble_print(const char *text)
//   Queues a text line to send to the phone (does nothing if no phone is connected).
// ---------------------------------------------------------------------------------------------
int ble_print(const char *text)
{
    int n=strlen(text), i;

    if (!ble_connected || n+1>ble_room()) return 0;
    for (i=0; i<=n; i++){
        tx_buf[(tx_head+tx_count)%TX_SIZE]=(i<n) ? text[i] : '\n';
        tx_count++;
    }
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static void send_some(void)
//   Sends up to 64 queued bytes to the phone, but only if the module's TX FIFO has room for
//   them.
// ---------------------------------------------------------------------------------------------
// Send up to 64 queued bytes, but only if the module has room for them.
// AT+BLEUARTFIFO=TX answers with the free bytes in the module's send buffer. Without this
// check, a full buffer makes the module stall for 200 ms and then drop the text.
static void send_some(void)
{
    char buf[TX_CHUNK];
    int n, i, r;

    n=(tx_count>TX_CHUNK) ? TX_CHUNK : tx_count;
    if (fifo_check){
        r=ble_at("AT+BLEUARTFIFO=TX");
        if (r<0){ failed(); return; }
        if (r==0) fifo_check=0;         // older firmware: no FIFO command, send at a safe pace
        else if (strlen(reply)>0 && (reply[0]<'0' || reply[0]>'9')) fifo_check=0;
        else if (atoi(reply)<n+8){      // not enough room yet: try again in 20 ms
            tx_wait=20;
            return;
        }
    }
    for (i=0; i<n; i++){
        buf[i]=tx_buf[tx_head];
        tx_head=(tx_head+1)%TX_SIZE;
    }
    tx_count=tx_count-n;
    if (!ble_cmd(CMD_UART_TX, buf, n)){ failed(); return; }
    // with the FIFO check, send again right away; without it, 64 bytes every 400 ms (160 B/s)
    tx_wait=fifo_check ? 0 : 400;
}

// ---------- main loop ----------

// ---------------------------------------------------------------------------------------------
// void ble_service(void)
//   Called from main() every pass: checks the link, reads phone text, sends queued text
//   (retries every 1 s if the module failed).
// ---------------------------------------------------------------------------------------------
void ble_service(void)
{
    int was, i;

    if (!ble_ok){
        if (!tried || ms_since(t_retry)>=RETRY_MS){     // first try right away, then every 1 s
            tried=1;
            t_retry=_CP0_GET_COUNT();
            ble_start();
        }
        return;
    }

    if (ms_since(t_link)>=LINK_EVERY_MS){           // is a phone connected?
        t_link=_CP0_GET_COUNT();
        if (ble_at("AT+GAPGETCONN")<0){ failed(); return; }
        was=ble_connected;
        ble_connected=(reply[0]=='1');
        if (was && !ble_connected) lost(" Phone disconnected. Motors off if the phone had armed them.");
        if (!was && ble_connected){
            putsU2(" Phone connected.");
            ble_cmd(CMD_UART_RX, "", 0);            // throw away anything old in the module
            line_len=0;
            lines_count=0;
        }
    }
    if (!ble_connected) return;

    if (ms_since(t_rx)>=RX_EVERY_MS){               // what did the phone send?
        t_rx=_CP0_GET_COUNT();
        if (!ble_cmd(CMD_UART_RX, "", 0)){ failed(); return; }
        for (i=0; i<reply_len; i++) phone_char(reply[i]);
    }

    if (tx_count>0 && ms_since(t_tx)>=tx_wait){     // text waiting for the phone
        t_tx=_CP0_GET_COUNT();
        send_some();
    }
}
