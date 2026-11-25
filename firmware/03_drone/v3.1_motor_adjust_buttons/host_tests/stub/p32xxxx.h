/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/host_tests/stub/p32xxxx.h
 *  What    : Host test support: stand-in PIC32 register declarations.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : CURRENT. v3.0 plus the O command. Host tests pass (87 checks). Not yet built in
 *            MPLAB or flown.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  TYPES IN THIS FILE: __ANSELBbits_t, __ANSELGbits_t, __ANSELHbits_t, __CNPUBbits_t,
 *    __IEC0bits_t, __IEC4bits_t, __IFS0bits_t, __IFS4bits_t, __INTCONbits_t, __IPC36bits_t,
 *    __IPC4bits_t, __LATDbits_t, __LATHbits_t, __LATKbits_t, __PORTBbits_t, __SPI1STATbits_t,
 *    __TRISBbits_t, __TRISDbits_t, __TRISFbits_t, __TRISGbits_t, __TRISHbits_t, __TRISKbits_t,
 *    __U2STAbits_t, __LATJbits_t, __TRISJbits_t, __TRISAbits_t, __T4CONbits_t
 * ============================================================================================ */

#ifndef STUB_P32
#define STUB_P32
#define __ISR(v,ipl) __attribute__((used))
#define _TIMER_4_VECTOR 19
#define _UART2_RX_VECTOR 146
#define _CP0_GET_COUNT() (cp0)
extern volatile unsigned int cp0;
#define __builtin_enable_interrupts() ((void)0)
extern volatile unsigned int OC1CON;
extern volatile unsigned int OC1R;
extern volatile unsigned int OC1RS;
extern volatile unsigned int OC2CON;
extern volatile unsigned int OC2R;
extern volatile unsigned int OC2RS;
extern volatile unsigned int OC3CON;
extern volatile unsigned int OC3R;
extern volatile unsigned int OC3RS;
extern volatile unsigned int OC4CON;
extern volatile unsigned int OC4R;
extern volatile unsigned int OC4RS;
extern volatile unsigned int PR2;
extern volatile unsigned int PR4;
extern volatile unsigned int RPB14R;
extern volatile unsigned int RPD0R;
extern volatile unsigned int RPD10R;
extern volatile unsigned int RPF2R;
extern volatile unsigned int RPF4R;
extern volatile unsigned int RPF5R;
extern volatile unsigned int SDI1R;
extern volatile unsigned int SPI1BRG;
extern volatile unsigned int SPI1BUF;
extern volatile unsigned int SPI1CON;
extern volatile unsigned int T2CON;
extern volatile unsigned int T3CON;
extern volatile unsigned int T4CON;
extern volatile unsigned int TMR2;
extern volatile unsigned int TMR3;
extern volatile unsigned int TMR4;
extern volatile unsigned int U2BRG;
extern volatile unsigned int U2MODE;
extern volatile unsigned int U2RXR;
extern volatile unsigned int U2RXREG;
extern volatile unsigned int U2STA;
extern volatile unsigned int U2TXREG;
// TYPE: __ANSELBbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned ANSB12:3; unsigned ANSB13:3; } __ANSELBbits_t; extern volatile __ANSELBbits_t ANSELBbits;
// TYPE: __ANSELGbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned ANSG6:3; } __ANSELGbits_t; extern volatile __ANSELGbits_t ANSELGbits;
// TYPE: __ANSELHbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned ANSH0:3; unsigned ANSH1:3; } __ANSELHbits_t; extern volatile __ANSELHbits_t ANSELHbits;
// TYPE: __CNPUBbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned CNPUB12:3; unsigned CNPUB13:3; } __CNPUBbits_t; extern volatile __CNPUBbits_t CNPUBbits;
// TYPE: __IEC0bits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned T4IE:3; } __IEC0bits_t; extern volatile __IEC0bits_t IEC0bits;
// TYPE: __IEC4bits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned U2RXIE:3; } __IEC4bits_t; extern volatile __IEC4bits_t IEC4bits;
// TYPE: __IFS0bits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned T4IF:3; } __IFS0bits_t; extern volatile __IFS0bits_t IFS0bits;
// TYPE: __IFS4bits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned U2RXIF:3; } __IFS4bits_t; extern volatile __IFS4bits_t IFS4bits;
// TYPE: __INTCONbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned MVEC:3; } __INTCONbits_t; extern volatile __INTCONbits_t INTCONbits;
// TYPE: __IPC36bits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned U2RXIP:3; } __IPC36bits_t; extern volatile __IPC36bits_t IPC36bits;
// TYPE: __IPC4bits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned T4IP:3; } __IPC4bits_t; extern volatile __IPC4bits_t IPC4bits;
// TYPE: __LATDbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned LATD1:3; } __LATDbits_t; extern volatile __LATDbits_t LATDbits;
// TYPE: __LATHbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned LATH0:3; unsigned LATH1:3; unsigned LATH2:3; } __LATHbits_t; extern volatile __LATHbits_t LATHbits;
// TYPE: __LATKbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned LATK1:3; unsigned LATK4:3; } __LATKbits_t; extern volatile __LATKbits_t LATKbits;
// TYPE: __PORTBbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned RB12:3; unsigned RB13:3; } __PORTBbits_t; extern volatile __PORTBbits_t PORTBbits;
// TYPE: __SPI1STATbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned SPIRBF:3; } __SPI1STATbits_t; extern volatile __SPI1STATbits_t SPI1STATbits;
// TYPE: __TRISBbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISB12:3; unsigned TRISB13:3; unsigned TRISB14:3; } __TRISBbits_t; extern volatile __TRISBbits_t TRISBbits;
// TYPE: __TRISDbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISD0:3; unsigned TRISD1:3; unsigned TRISD10:3; } __TRISDbits_t; extern volatile __TRISDbits_t TRISDbits;
// TYPE: __TRISFbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISF2:3; unsigned TRISF4:3; unsigned TRISF5:3; } __TRISFbits_t; extern volatile __TRISFbits_t TRISFbits;
// TYPE: __TRISGbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISG6:3; } __TRISGbits_t; extern volatile __TRISGbits_t TRISGbits;
// TYPE: __TRISHbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISH0:3; unsigned TRISH1:3; unsigned TRISH2:3; } __TRISHbits_t; extern volatile __TRISHbits_t TRISHbits;
// TYPE: __TRISKbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISK1:3; unsigned TRISK4:3; } __TRISKbits_t; extern volatile __TRISKbits_t TRISKbits;
// TYPE: __U2STAbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned OERR:3; unsigned URXDA:3; unsigned UTXBF:3; } __U2STAbits_t; extern volatile __U2STAbits_t U2STAbits;

extern volatile unsigned int SPI1STATCLR;
// TYPE: __LATJbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned LATJ13:3; } __LATJbits_t; extern volatile __LATJbits_t LATJbits;
// TYPE: __TRISJbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISJ13:3; } __TRISJbits_t; extern volatile __TRISJbits_t TRISJbits;
// TYPE: __TRISAbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned TRISA14:3; } __TRISAbits_t; extern volatile __TRISAbits_t TRISAbits;
// TYPE: __T4CONbits_t - Stand-in for the XC32 register bit-field type of the same name (host tests only).
typedef struct { unsigned ON:1; } __T4CONbits_t; extern volatile __T4CONbits_t T4CONbits;
#endif
