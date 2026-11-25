/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/Drone.X/uart.c
 *  What    : UART2 link to the PC (PuTTY and the PC tuner) with an interrupt-driven receive
 *            buffer.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : CURRENT. v3.0 plus the O command. Host tests pass (87 checks). Not yet built in
 *            MPLAB or flown.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initU2(), U2RXHandler(),
 *    getU2(), putU2(), putsU2(), fstr()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include <stdio.h>
# include "uart.h"

// UART2 to the PC through the kit's USB port (J11 -> PuTTY or the PID tuner), 115200 baud.
// J12 pin 8 = RG6 = U2RX, pin 10 = RB14 = U2TX. Never wire those two pins.
//
// Sending: wait for room, then send (class code).
// Receiving: an interrupt drops every incoming character into rx_buf, so no
// tuner command is lost while main() is busy printing or calibrating.

#define RXSIZE 256

volatile char rx_buf[RXSIZE];
volatile int rx_head=0;     // where the interrupt puts the next character
volatile int rx_tail=0;     // where getU2() takes the next one out

// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
void initU2(void)
{
    ANSELGbits.ANSG6=0;     // RG6 = U2RX: digital input
    TRISGbits.TRISG6=1;
    U2RXR=1;                // U2RX <- RG6
    TRISBbits.TRISB14=0;    // RB14 = U2TX
    RPB14R=2;               // U2TX -> RB14
    U2BRG=129;              // 60 MHz / (4 x 130) = 115,384 baud (BRGH = 1)
    U2MODE=0x8008;          // UART ON, BRGH = 1, 8 bits, no parity, 1 stop bit
    U2STA=0x1400;           // receiver + transmitter ON, interrupt on every character received

    IFS4bits.U2RXIF=0;
    IPC36bits.U2RXIP=3;     // priority 3: lower than the control loop (5)
    IEC4bits.U2RXIE=1;
}

// ---------------------------------------------------------------------------------------------
// void __ISR(_UART2_RX_VECTOR, IPL3SOFT) U2RXHandler(void)
//   UART2 receive interrupt: moves every waiting character into the receive ring buffer.
// ---------------------------------------------------------------------------------------------
void __ISR(_UART2_RX_VECTOR, IPL3SOFT) U2RXHandler(void)
{
    int c, next;

    while (U2STAbits.URXDA){                // move every waiting character into the buffer
        c=U2RXREG;
        next=(rx_head+1)%RXSIZE;
        if (next!=rx_tail){                 // buffer full: drop the new character, keep the old ones
            rx_buf[rx_head]=c;
            rx_head=next;
        }
    }
    if (U2STAbits.OERR) U2STAbits.OERR=0;   // if the hardware ever overflowed, clear it so it keeps receiving
    IFS4bits.U2RXIF=0;
}

// ---------------------------------------------------------------------------------------------
// int getU2(void)
//   Returns the next character the PC sent (from the receive buffer), or -1 if there is none.
// ---------------------------------------------------------------------------------------------
int getU2(void)
{
    int c;

    if (rx_tail==rx_head) return -1;        // nothing new
    c=rx_buf[rx_tail];
    rx_tail=(rx_tail+1)%RXSIZE;
    return c;
}

// ---------------------------------------------------------------------------------------------
// int putU2(int c)
//   Sends one character out UART2 to the PC (waits while the transmit buffer is full).
// ---------------------------------------------------------------------------------------------
int putU2(int c)
{
    while (U2STAbits.UTXBF);    // wait for room in the transmit buffer (class code)
    U2TXREG=c;
    return c;
}

// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Sends a text line to the PC over UART2, followed by a new line.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s)
{
    while (*s){
        putU2(*s++);
    }
    putU2('\r');
    putU2('\n');
}

// ---------------------------------------------------------------------------------------------
// char *fstr(char *buf, float v, int decimals)
//   Turns a float into text with a fixed number of decimals (XC32's printf may leave out %f).
// ---------------------------------------------------------------------------------------------
// Float to text without %f (XC32 can leave float printing out of printf).
// fstr(buf, -12.345, 2) -> "-12.35"    fstr(buf, 3.5, 1) -> "+3.5"
// buf needs room for 16 characters.
char *fstr(char *buf, float v, int decimals)
{
    long scale=1;
    long n;
    int i;
    char sign='+';

    for (i=0;i<decimals;i++) scale=scale*10;    // 2 decimals -> 100
    if (v<0){
        sign='-';
        v=-v;
    }
    n=(long)(v*scale+0.5f);                     // round to the last decimal shown
    if (n==0) sign='+';                         // never print "-0.00"
    if (decimals==0) sprintf(buf, "%c%ld", sign, n);
    else if (decimals==1) sprintf(buf, "%c%ld.%01ld", sign, n/scale, n%scale);
    else if (decimals==2) sprintf(buf, "%c%ld.%02ld", sign, n/scale, n%scale);
    else if (decimals==3) sprintf(buf, "%c%ld.%03ld", sign, n/scale, n%scale);
    else sprintf(buf, "%c%ld.%04ld", sign, n/10000, n%10000);    // 4 decimals (most we use)
    return buf;
}
