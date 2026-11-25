/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/Drone.X/motors.c
 *  What    : ESC pulse outputs: Timer2 at 50 Hz with OC1-OC4, the motor-power relay pin, and
 *            the per-motor trims.
 *  Version : v3.0   |   Date: 2026-11-25
 *  Status  : CURRENT. Built from v2.0 (which builds and flew on the tether). Host tests pass
 *            (78 checks, including a simulated flight with the real 4 ms loop). Not yet built
 *            in MPLAB or flown.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initMotors(), trim(),
 *    set_motor()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include "motors.h"
# include "control.h"
# include "bluefruit.h"

// ESC pulses: Output Compare in PWM mode, all four on Timer2.
// Same timer settings as esc_full_speed.c, which spun your ESCs.
//
// Timer2: 60 MHz / 64 = 937,500 ticks per second = 0.9375 ticks per microsecond
//   PR2 = 18749 -> (18749+1) / 937,500 = 20 ms between pulses = 50 Hz
//   1000 us = 938 ticks (stop)     2000 us = 1875 ticks (full)
//
// To try 400 Hz later (faster ESC response), change two lines:
//   T2CON=0x8030 (1:8)  and  TICKS_PER_US 7.5f   (PR2 stays 18749 -> 2.5 ms)
//
// Pin remap codes (PIC32MZ EF datasheet DS60001320H, Table 12-3):
//   RPD0R=12 -> OC1     RPF2R=11 -> OC2     RPF4R=11 -> OC3     RPF5R=11 -> OC4

#define TICKS_PER_US 0.9375f

// ---------------------------------------------------------------------------------------------
// void initMotors(void)
//   Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC pulses (all start at
//   1000 us = stop).
// ---------------------------------------------------------------------------------------------
void initMotors(void)
{
    // Motor-power relay on RK4 (J12 pin 33): set the latch LOW first, THEN make the pin an
    // output, so it never goes high for even a moment (same order as the relay test program).
    // Before this line the pin is an input (reset state): the relay is off then too.
    LATKbits.LATK4=0;
    TRISKbits.TRISK4=0;
    TRISDbits.TRISD0=0;
    RPD0R=12;               // RD0 = OC1 -> J12 pin 11 = ESC 1 (front-left)
    TRISFbits.TRISF2=0;
    RPF2R=11;               // RF2 = OC2 -> J12 pin 12 = ESC 2 (front-right)
    TRISFbits.TRISF4=0;
    RPF4R=11;               // RF4 = OC3 -> J12 pin 21 = ESC 3 (rear-left)
    TRISFbits.TRISF5=0;
    RPF5R=11;               // RF5 = OC4 -> J12 pin 19 = ESC 4 (rear-right)

    T2CON=0x0000;           // Timer2 off while setting up
    TMR2=0;
    PR2=18749;              // 20 ms frame
    OC1R=938;               // 1000 us = motor off, so the ESCs arm
    OC1RS=938;
    OC2R=938;
    OC2RS=938;
    OC3R=938;
    OC3RS=938;
    OC4R=938;
    OC4RS=938;
    OC1CON=0x8006;          // ON, PWM mode, uses Timer2
    OC2CON=0x8006;
    OC3CON=0x8006;
    OC4CON=0x8006;
    T2CON=0x8060;           // Timer2 ON, 1:64
}

// ---------------------------------------------------------------------------------------------
// float trim(int n, float us)
//   Per-motor trim from bench test 11: 1000 + gain x (pulse - 1000) + offset. A stop (1000 us
//   or less) stays a stop.
// ---------------------------------------------------------------------------------------------
// The trim for ESC n (motors.h): a stop (1000 us) stays exactly 1000 us.
float trim(int n, float us)
{
    float g=1.0f, o=0.0f;
    if (us<=PULSE_OFF) return PULSE_OFF;
    if (n==1){ g=TRIM_GAIN_FL; o=TRIM_OFFSET_FL; }
    if (n==2){ g=TRIM_GAIN_FR; o=TRIM_OFFSET_FR; }
    if (n==3){ g=TRIM_GAIN_RL; o=TRIM_OFFSET_RL; }
    if (n==4){ g=TRIM_GAIN_RR; o=TRIM_OFFSET_RR; }
    us=PULSE_OFF+g*(us-PULSE_OFF)+o;
    if (us<PULSE_OFF) us=PULSE_OFF;
    return us;
}

// ---------------------------------------------------------------------------------------------
// void set_motor(int n, float us)
//   Sets ESC n (1-4): forced to stop unless armed and running, then the motor's trim, then held
//   inside 1000-2000 us.
// ---------------------------------------------------------------------------------------------
void set_motor(int n, float us)
{
    int ticks;

    // Locked off unless armed and running. A lost link (USB or Bluetooth heartbeat) is
    // caught by the 4 ms interrupt for whichever link armed the drone, so the
    // Bluefruit does not have to be connected for a USB session.
    if (motor_mode==0 || kill_latched || !control_ready) us=PULSE_OFF;
    if (!(us>=PULSE_OFF && us<=PULSE_MAX)) us=PULSE_OFF; // includes invalid floats
    us=trim(n, us);                     // per-motor trim from the calibration test
    if (us<PULSE_OFF) us=PULSE_OFF;     // never outside 1000..2000 us
    if (us>PULSE_MAX) us=PULSE_MAX;
    ticks=(int)(us*TICKS_PER_US+0.5f);  // microseconds -> timer ticks (rounded)

    // OCxRS = the next pulse width. The hardware switches to it at the start of
    // the next 20 ms frame, so changing it in the middle of a pulse is safe.
    if (n==1) OC1RS=ticks;              // ESC 1, front-left
    if (n==2) OC2RS=ticks;              // ESC 2, front-right
    if (n==3) OC3RS=ticks;              // ESC 3, rear-left
    if (n==4) OC4RS=ticks;              // ESC 4, rear-right
}
