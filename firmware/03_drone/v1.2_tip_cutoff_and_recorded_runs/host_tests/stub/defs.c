/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs/host_tests/stub/defs.c
 *  What    : Host test support: the stand-in PIC32 registers (plain variables).
 *  Version : v1.2   |   Date: 2026-11-22
 *  Status  : Host tests pass. Not built with XC32.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ============================================================================================ */

#include "p32xxxx.h"
volatile unsigned int cp0;
volatile unsigned int OC1CON;
volatile unsigned int OC1R;
volatile unsigned int OC1RS;
volatile unsigned int OC2CON;
volatile unsigned int OC2R;
volatile unsigned int OC2RS;
volatile unsigned int OC3CON;
volatile unsigned int OC3R;
volatile unsigned int OC3RS;
volatile unsigned int OC4CON;
volatile unsigned int OC4R;
volatile unsigned int OC4RS;
volatile unsigned int PR2;
volatile unsigned int PR4;
volatile unsigned int RPB14R;
volatile unsigned int RPD0R;
volatile unsigned int RPD10R;
volatile unsigned int RPF2R;
volatile unsigned int RPF4R;
volatile unsigned int RPF5R;
volatile unsigned int SDI1R;
volatile unsigned int SPI1BRG;
volatile unsigned int SPI1BUF;
volatile unsigned int SPI1CON;
volatile unsigned int T2CON;
volatile unsigned int T3CON;
volatile unsigned int T4CON;
volatile unsigned int TMR2;
volatile unsigned int TMR3;
volatile unsigned int TMR4;
volatile unsigned int U2BRG;
volatile unsigned int U2MODE;
volatile unsigned int U2RXR;
volatile unsigned int U2RXREG;
volatile unsigned int U2STA;
volatile unsigned int U2TXREG;
volatile __ANSELGbits_t ANSELGbits;
volatile __TRISGbits_t TRISGbits;
volatile __TRISBbits_t TRISBbits;
volatile __IFS4bits_t IFS4bits;
volatile __IPC36bits_t IPC36bits;
volatile __IEC4bits_t IEC4bits;
volatile __U2STAbits_t U2STAbits;
volatile __TRISDbits_t TRISDbits;
volatile __TRISFbits_t TRISFbits;
volatile __ANSELHbits_t ANSELHbits;
volatile __TRISHbits_t TRISHbits;
volatile __LATHbits_t LATHbits;
volatile __ANSELBbits_t ANSELBbits;
volatile __CNPUBbits_t CNPUBbits;
volatile __INTCONbits_t INTCONbits;
volatile __PORTBbits_t PORTBbits;
volatile __IFS0bits_t IFS0bits;
volatile __IPC4bits_t IPC4bits;
volatile __IEC0bits_t IEC0bits;
volatile __TRISKbits_t TRISKbits;
volatile __LATKbits_t LATKbits;
volatile __LATDbits_t LATDbits;
volatile __SPI1STATbits_t SPI1STATbits;

volatile unsigned int SPI1STATCLR;
volatile __LATJbits_t LATJbits; volatile __TRISJbits_t TRISJbits; volatile __TRISAbits_t TRISAbits; volatile __T4CONbits_t T4CONbits;
