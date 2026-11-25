/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/bluetooth.c
 *  What    : Bluefruit SPI code from class Labs 5/6: phone Control Pad buttons in, text out.
 *  Version : FC draft   |   Date: 2026-11-04
 *  Status  : Written as practice code before the hardware arrived. Never flown. Superseded by
 *            the SewSaw seesaw code (the IMU moved to SPI).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (I2C2)
 *      3V3 pin 1   GND pin 6   SDA pin 3 (RA3)   SCL pin 5 (RA2)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Bluefruit SPI Friend (shares SPI1 with the IMU)
 *      VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initSPI(), writeSPI(),
 *    u_receive(), ble_send()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <string.h>
# include "bluetooth.h"
# include "delay.h"

// Bluefruit LE SPI Friend on SPI1 (Lab 5 / Lab 6 code)
//sck:  pin 23 (RD1)
//miso: pin 7  (RPA14 -> SDI1)
//mosi: pin 37 (RPD10 <- SDO1)
//cs:   pin 13 (RJ13)   CAPSTONE: moved, pin 11 (RD0) is now ESC 1 and pin 3 is the IMU's SDA
//vin:  pin 2  (+5 V)
//gnd:  pin 9

// ---------------------------------------------------------------------------------------------
// void initSPI(void)
//   Sets up SPI1 (pin remaps, mode, clock speed) and the chip-select pin for this version's
//   wiring.
// ---------------------------------------------------------------------------------------------
void initSPI(void)
{
    TRISJbits.TRISJ13=0;    //cs
    LATJbits.LATJ13=1;

    RPD10R=5;
    SDI1R=13;
    TRISDbits.TRISD1=0;

    SPI1CON=0x8120;
    SPI1BRG=29;             //60MHz/(2*(29+1)) = 1MHz
}

// ---------------------------------------------------------------------------------------------
// int writeSPI(int i)
//   Sends one byte over SPI1 and returns the byte that came back at the same time.
// ---------------------------------------------------------------------------------------------
int writeSPI(int i)
{
    SPI1BUF=i;
    while (!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}

// ---------------------------------------------------------------------------------------------
// int* u_receive(void)
//   Reads what the phone sent (Control Pad button packets) over SPI (Lab 6).
// ---------------------------------------------------------------------------------------------
// Lab 6: read what the phone sent. Control Pad button = ! B button pressed CRC
int* u_receive(void)
{
    static int r[6];
    LATJbits.LATJ13=0;
    delay_100us();
    writeSPI(0x10);
    writeSPI(0x02);
    writeSPI(0x0a);
    writeSPI(0x06);
    for (int i=0;i<6;i++){
        r[i]=writeSPI(0);
    }
    LATJbits.LATJ13=1;
    return r;
}

// ---------------------------------------------------------------------------------------------
// void ble_send(char *s)
//   Sends a string to the phone with SDEP UART_TX packets, 16 characters per packet (Lab 5).
// ---------------------------------------------------------------------------------------------
// Lab 5: send a string to the phone with UART_TX (0x0A01), 16 characters per packet
void ble_send(char *s)
{
    int len=strlen(s);
    int n, i, first, tries;

    while (len>0){
        n=len;
        if (n>16) n=16;
        len=len-n;

        for (tries=0; tries<100; tries++){
            LATJbits.LATJ13=0;
            delay_100us();
            first=writeSPI(0x10);
            if (first!=0xFE) break;      // 0xFE = Bluefruit busy, try again
            LATJbits.LATJ13=1;
            delay_ms(1);
        }
        if (tries==100) return;

        writeSPI(0x01);
        writeSPI(0x0a);
        if (len>0) writeSPI(n+0x80);     // more packets coming
        else writeSPI(n);
        for (i=0; i<n; i++){
            writeSPI(s[i]);
        }
        LATJbits.LATJ13=1;
        delay_ms(1);

        s=s+n;
    }
}
