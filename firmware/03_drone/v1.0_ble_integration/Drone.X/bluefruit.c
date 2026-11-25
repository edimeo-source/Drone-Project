/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v1.0_ble_integration/Drone.X/bluefruit.c
 *  What    : Adafruit Bluefruit LE SPI Friend driver (SDEP over SPI1, shared with the IMU): the
 *            phone link.
 *  Version : v1.0   |   Date: 2026-11-20
 *  Status  : Partner's merge of v0.1 + v0.2. Not built with XC32. Replaced by v1.1 the same
 *            day.
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
 *    Bluefruit SPI Friend (shares SPI1 with the IMU)
 *      VIN pin 1 (3.3 V, shared with the IMU)   GND pin 6   SCK pin 23   MOSI pin 37   MISO
 *      pin 7
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): clock_now(), wait_us(),
 *    bus_take(), bus_release(), bus_byte(), send_packet(), receive_packet(), begin_command(),
 *    communication_failed(), bluetooth_link_ok(), queue_status(), bluetooth_send_text(),
 *    phone_command(), phone_text(), unpack_reply(), finish_command(), initBluetooth(),
 *    bluetooth_service()
 * ============================================================================================ */

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "bluefruit.h"
#include "control.h"
#include "tuner.h"
#include "motors.h"

// Bluefruit LE SPI Friend. No changes to the universal pinout:
// Current harness: VIN=J12-1 (3.3 V), GND=J12-6, SCK=23, MOSI=37, MISO=7, CS=13; IRQ/RST NC.
// SDEP: 4-byte header, at most 16 data bytes, command ID 0x0A00.
// Each packet borrows SPI1, then restores the IMU's mode before the ISR can run.
// No AT response wait holds off the 4 ms interrupt.
// Sources: Adafruit's SDEP, BLE Services and BLE GAP command guides.

#define TICKS_MS       60000u      // core timer = SYSCLK/2 = 60 MHz
#define AT_TIMEOUT     (500u*TICKS_MS)
#define LINK_TIMEOUT   (750u*TICKS_MS)
#define COMMAND_ECHO   1
#define COMMAND_AT     2
#define COMMAND_LINK   3
#define COMMAND_RX     4
#define COMMAND_TX     5

volatile int ble_ready=0;
volatile int ble_connected=0;
volatile int ble_arm_ready=0;
volatile unsigned int ble_last_link=0;
volatile int ble_notice=0;          // 1 ready, 2 disconnected, 3 communication failed

static int started=0, phase=0, command_kind=0;
static unsigned int next_service=0, retry_at=0, command_started=0, last_query=0, last_rx=0;
static char command[192], reply[300], phone_word[96], outgoing[144];
static unsigned int tx_pos=0, reply_len=0, word_len=0;
static int bad_word=0;
static unsigned int saved_con, saved_brg;
static int saved_t4, spi_failed;

// ---------------------------------------------------------------------------------------------
// static unsigned int clock_now(void)
//   Returns the core-timer count (60 MHz) used for every timeout in the Bluetooth driver.
// ---------------------------------------------------------------------------------------------
static unsigned int clock_now(void)
{
    return (unsigned int)_CP0_GET_COUNT();
}

// ---------------------------------------------------------------------------------------------
// static void wait_us(unsigned int us)
//   Waits us microseconds on the core timer (60 counts per microsecond).
// ---------------------------------------------------------------------------------------------
static void wait_us(unsigned int us)
{
    unsigned int start=clock_now();
    while ((unsigned int)(clock_now()-start)<us*60u) ;
}

// ---------------------------------------------------------------------------------------------
// static void bus_take(void)
//   Borrows SPI1 from the IMU: 4 ms interrupt held off, IMU CS high, Bluefruit SPI settings.
// ---------------------------------------------------------------------------------------------
static void bus_take(void)
{
    saved_t4=IEC0bits.T4IE;
    IEC0bits.T4IE=0;                 // no IMU transaction can interrupt this packet
    saved_con=SPI1CON;
    saved_brg=SPI1BRG;
    LATKbits.LATK1=1;               // IMU CS high
    LATJbits.LATJ13=1;              // Bluefruit CS high while changing clock mode
    SPI1CON=0;
    SPI1STATCLR=0x40;               // clear receive overflow
    SPI1BRG=29;                     // PBCLK 60 MHz -> SPI 1 MHz
    SPI1CON=0x8120;                 // master, 8 bits, mode 0
    spi_failed=0;
    LATJbits.LATJ13=0;
    wait_us(110);                   // Bluefruit needs at least 100 us after CS falls
}

// ---------------------------------------------------------------------------------------------
// static void bus_release(void)
//   Gives SPI1 back to the IMU (its mode and speed restored before the interrupt is
//   re-enabled).
// ---------------------------------------------------------------------------------------------
static void bus_release(void)
{
    LATJbits.LATJ13=1;
    SPI1CON=0;
    SPI1STATCLR=0x40;
    SPI1BRG=saved_brg;
    SPI1CON=saved_con;              // IMU mode 3 restored BEFORE its interrupt is enabled
    IEC0bits.T4IE=saved_t4;         // leave a pending timer flag pending
}

// ---------------------------------------------------------------------------------------------
// static int bus_byte(int value)
//   One SPI byte to/from the Bluefruit with a 50 us timeout.
// ---------------------------------------------------------------------------------------------
static int bus_byte(int value)
{
    unsigned int start=clock_now();
    if (spi_failed) return 0;
    SPI1BUF=value;
    while (!SPI1STATbits.SPIRBF){
        if ((unsigned int)(clock_now()-start)>3000u){ // 50 us, never hang on broken SPI
            spi_failed=1;
            return 0;
        }
    }
    return SPI1BUF & 255;
}

// ---------------------------------------------------------------------------------------------
// static int send_packet(void)
//   Sends one SDEP command packet (the next 16-byte piece of the command) to the Bluefruit over
//   SPI.
// ---------------------------------------------------------------------------------------------
// 1 = sent, 0 = try again on a later service call, -1 = broken SPI.
static int send_packet(void)
{
    int ready, count, i, failed;
    unsigned int remaining=(unsigned int)strlen(command)-tx_pos;
    count=remaining>16u ? 16 : (int)remaining;
    bus_take();
    ready=bus_byte(0x10);
    if (!spi_failed && (ready==0xFE || ready==0xFF)){
        bus_release();
        return 0;
    }
    bus_byte(0x00);
    bus_byte(0x0A);
    bus_byte(count | (remaining>(unsigned int)count ? 0x80 : 0));
    for (i=0;i<count;i++) bus_byte((unsigned char)command[tx_pos+i]);
    failed=spi_failed;
    bus_release();
    if (failed) return -1;
    tx_pos+=(unsigned int)count;
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static int receive_packet(int discard)
//   Reads one SDEP reply packet (IRQ is not wired, so it polls). -1 error, 0 nothing/partial, 1
//   complete.
// ---------------------------------------------------------------------------------------------
// -1 = error, 0 = empty/alert/another fragment, 1 = complete reply.
// IRQ is not wired, so read the SDEP first byte and retry FE/FF later.
static int receive_packet(int discard)
{
    int type, low, high, size, length, value, is_reply, failed;
    int i, bad=0;
    bus_take();
    type=bus_byte(0xFF);
    if (!spi_failed && (type==0xFE || type==0xFF)){
        bus_release();
        return 0;
    }
    low=bus_byte(0xFF);
    high=bus_byte(0xFF);
    size=bus_byte(0xFF);
    length=size & 0x7F;
    if (length>16 || (size & 0x60) ||
        (type!=0x20 && type!=0x40 && type!=0x80)) bad=1;
    is_reply=type==0x20 && low==0 && high==0x0A;
    if (!bad){
        for (i=0;i<length;i++){
            value=bus_byte(0xFF);
            if (is_reply && !discard){
                if (reply_len<sizeof(reply)-1u) reply[reply_len++]=(char)value;
                else bad=1;
            }
        }
    }
    failed=spi_failed;
    bus_release();
    if (bad || failed || type==0x80) return -1;
    if (discard) return 2;          // a packet was drained during startup/recovery
    if (!is_reply || (size & 0x80)) return 0;
    reply[reply_len]=0;
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static void begin_command(int kind, const char *text)
//   Starts sending an AT command to the Bluefruit and remembers what kind of reply to expect.
// ---------------------------------------------------------------------------------------------
static void begin_command(int kind, const char *text)
{
    strcpy(command, text);          // all commands are fixed strings or bounded snprintf
    command_kind=kind;
    tx_pos=reply_len=0;
    reply[0]=0;
    command_started=clock_now();
    phase=1;
}

// ---------------------------------------------------------------------------------------------
// static void communication_failed(void)
//   Marks the Bluefruit as failed; motors off only if the phone was the link that armed them.
// ---------------------------------------------------------------------------------------------
static void communication_failed(void)
{
    ble_ready=0;
    ble_connected=0;
    /* A failed BLE module must not disable the independent USB tether path. */
    if (control_link_source==1){
        motors_off();
        off_reason=OFF_BLE;
    }
    ble_notice=3;
    word_len=bad_word=0;
    outgoing[0]=0;
    phase=0;
    retry_at=clock_now()+1000u*TICKS_MS;
}

// ---------------------------------------------------------------------------------------------
// int bluetooth_link_ok(void)
//   1 if the Bluefruit is working, a phone is connected, and the link was checked recently.
// ---------------------------------------------------------------------------------------------
int bluetooth_link_ok(void)
{
    return ble_ready && ble_connected &&
           (unsigned int)(clock_now()-ble_last_link)<LINK_TIMEOUT;
}


// ---------------------------------------------------------------------------------------------
// static void queue_status(const char *text)
//   Queues a status line for the phone.
// ---------------------------------------------------------------------------------------------
static void queue_status(const char *text)
{
    snprintf(outgoing, sizeof(outgoing), "%s", text);
}

// ---------------------------------------------------------------------------------------------
// int bluetooth_send_text(const char *text)
//   Queues a status line or reply for the phone (waits in the 8-line queue until there is
//   room).
// ---------------------------------------------------------------------------------------------
int bluetooth_send_text(const char *text)
{
    if (!text || !ble_ready || !ble_connected) return 0;
    if (outgoing[0]) return 0;          /* never overwrite an in-flight/pending line */
    if (strlen(text)>=sizeof(outgoing)) return 0;
    strcpy(outgoing,text);
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static void phone_command(const char *text)
//   Runs one phone command: STOP/KILL/X, ARM, STATUS, or a tuner command.
// ---------------------------------------------------------------------------------------------
static void phone_command(const char *text)
{
    if (strcmp(text,"STOP")==0 || strcmp(text,"KILL")==0 || strcmp(text,"X")==0){
        motors_off();
        off_reason=OFF_REMOTE;
        queue_status("STOPPED - REARM REQUIRED");
    }
    else if (strcmp(text,"ARM")==0){
        if (control_arm_ble()) queue_status("ARMED - ESC STARTUP; WAIT 2 SECONDS");
        else queue_status("ARM REFUSED - CHECK LINK, CALIBRATION AND SW1");
    }
    else if (strcmp(text,"STATUS")==0){
        if (kill_latched) queue_status("STOPPED - REARM REQUIRED");
        else if (motor_mode==0) queue_status("ARMED - ESC POWER ON; WAIT OR START");
        else queue_status("MOTOR OUTPUT ACTIVE");
    }
    else if (strcmp(text,"H")==0){
        control_heartbeat_ble();
    }
    else if (text[0]=='R' && (text[1]==',' || text[1]=='\0')){
        queue_status("STEP TEST RECORDINGS USE USB; BLE LIVE DATA ONLY");
    }
    else if (*text){
        tuner_set_ble_output(1);
        tuner_command((char *)text);
        tuner_set_ble_output(0);
    }
}

// ---------------------------------------------------------------------------------------------
// static void phone_text(const char *text)
//   Splits text from the phone into commands at ;, CR or LF.
// ---------------------------------------------------------------------------------------------
static void phone_text(const char *text)
{
    char c;
    for (;*text;text++){
        c=*text;
        if (c=='\r' || c=='\n' || c==';'){
            if (word_len && !bad_word){
                phone_word[word_len]=0;
                phone_command(phone_word);
            }
            word_len=0; bad_word=0;
            continue;
        }
        if (bad_word) continue;
        if (word_len>=sizeof(phone_word)-1u){
            motors_off();
            word_len=0; bad_word=1;
            queue_status("STOPPED - INVALID INPUT");
            continue;
        }
        if (c>='a' && c<='z') c=(char)(c-'a'+'A');
        phone_word[word_len++]=c;
        phone_word[word_len]=0;
        /* Immediate kill even before a line terminator arrives. */
        if (strcmp(phone_word,"STOP")==0 || strcmp(phone_word,"KILL")==0 || strcmp(phone_word,"X")==0){
            phone_command(phone_word);
            word_len=0; bad_word=0;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// static int unpack_reply(void)
//   Removes the AT wrapper's final OK from a reply, keeping the phone's line breaks. 0 = not a
//   good reply.
// ---------------------------------------------------------------------------------------------
// Remove the AT wrapper's final OK; preserve phone data's newline delimiters.
static int unpack_reply(void)
{
    unsigned int n=reply_len;
    while (n && (reply[n-1]=='\r' || reply[n-1]=='\n')) --n;
    if (n<2u || reply[n-2]!='O' || reply[n-1]!='K') return 0;
    if (n>2u && reply[n-3]!='\r' && reply[n-3]!='\n') return 0;
    n-=2;
    if (n>=2u && reply[n-2]=='\r' && reply[n-1]=='\n') n-=2;
    else if (n && reply[n-1]=='\n') --n;
    reply[n]=0;                    // strip one AT separator, not the phone's own delimiter
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static void finish_command(void)
//   Handles the Bluefruit's reply to the command in progress (link state, FIFO room, phone
//   text).
// ---------------------------------------------------------------------------------------------
static void finish_command(void)
{
    unsigned int now=clock_now();
    char *p;
    int connected;
    if (!unpack_reply()){
        communication_failed();
        return;
    }
    phase=3;                       // idle: schedule the next command on a later call
    if (command_kind==COMMAND_ECHO) begin_command(COMMAND_AT,"AT");
    else if (command_kind==COMMAND_AT){
        ble_ready=1;
        ble_notice=1;
        last_query=now-100u*TICKS_MS;
        last_rx=now-20u*TICKS_MS;
    }
    else if (command_kind==COMMAND_LINK){
        p=reply;
        while (*p=='\r' || *p=='\n') p++;
        if (*p!='0' && *p!='1') { communication_failed(); return; }
        connected=(*p++=='1');
        while (*p=='\r' || *p=='\n') p++;
        if (*p) { communication_failed(); return; }
        if (connected){
            if (!ble_connected) ble_arm_ready=0;
            ble_last_link=now;
            ble_connected=1;
        }
        else{
            int was_ble_owner=(control_link_source==1);
            if (ble_connected || was_ble_owner) ble_notice=2;
            ble_connected=0;
            /* Do not let an absent BLE module kill a separately armed USB session. */
            if (was_ble_owner) motors_off();
            word_len=bad_word=0;
            outgoing[0]=0;
        }
    }
    else if (command_kind==COMMAND_RX){
        if (ble_connected){
            if (reply[0]) phone_text(reply);
            else if (!ble_arm_ready){
                word_len=bad_word=0;
                ble_arm_ready=1;    /* stale RX queue drained; fresh ARM can now be accepted */
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// void initBluetooth(void)
//   Starts the Bluefruit driver: CS pin high, queues cleared, link marked down, drone disarmed.
// ---------------------------------------------------------------------------------------------
void initBluetooth(void)
{
    LATJbits.LATJ13=1;
    TRISJbits.TRISJ13=0;
    kill_latched=1;
    ble_ready=ble_connected=ble_arm_ready=0;
    ble_last_link=0;
    phase=0;
    outgoing[0]=0;
    word_len=bad_word=0;
    phone_word[0]=0;
    retry_at=clock_now()+1000u*TICKS_MS;
    next_service=clock_now();
    started=1;
}

// ---------------------------------------------------------------------------------------------
// void bluetooth_service(void)
//   Called from main() every pass: does one small step of Bluetooth work (never blocks).
// ---------------------------------------------------------------------------------------------
void bluetooth_service(void)
{
    unsigned int now;
    int result;
    char message[192];
    if (!started) return;
    now=clock_now();
    if ((int32_t)(now-next_service)<0) return;
    // Prefer packets just after a timer tick; leave about 1 ms before the next one.
    // Still verify loop_period_us/LED3 on the real board with Bluetooth active.
    if (IEC0bits.T4IE && T4CONbits.ON && TMR4>2800) return;
    next_service=now+TICKS_MS;       // at most one SPI packet per 1 ms
    if (phase==0){
        if ((int32_t)(now-retry_at)<0) return;
        result=receive_packet(1);  // drain old responses before starting a new AT exchange
        if (result==0) begin_command(COMMAND_ECHO,"ATE=0");
        else if (result<0) communication_failed();
        return;
    }
    if (phase==1 || phase==2){
        if ((unsigned int)(now-command_started)>=AT_TIMEOUT){
            communication_failed();
            return;
        }
    }
    if (phase==1){
        result=send_packet();
        if (result<0) communication_failed();
        else if (result && tx_pos==strlen(command)) phase=2;
        return;
    }
    if (phase==2){
        result=receive_packet(0);
        if (result<0) communication_failed();
        else if (result==1) finish_command();
        return;
    }
    if ((unsigned int)(now-last_query)>=100u*TICKS_MS){
        last_query=now;
        begin_command(COMMAND_LINK,"AT+GAPGETCONN");
    }
    else if (ble_connected && (unsigned int)(now-last_rx)>=20u*TICKS_MS){
        last_rx=now;                    /* RX/heartbeat has priority over telemetry TX */
        begin_command(COMMAND_RX,"AT+BLEUARTRX");
    }
    else if (ble_connected && outgoing[0]){
        snprintf(message,sizeof(message),"AT+BLEUARTTX=%s\\r\\n",outgoing);
        outgoing[0]=0;
        begin_command(COMMAND_TX,message);
    }
}
