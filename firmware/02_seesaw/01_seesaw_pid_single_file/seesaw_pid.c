/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/01_seesaw_pid_single_file/seesaw_pid.c
 *  What    : Seesaw PID - single file (I2C IMU, 2 motors)
 *  Version : seesaw single-file   |   Date: 2026-11-12
 *  Status  : Builds. Still uses the IMU on I2C; replaced by the SPI multi-file SewSaw project.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (I2C2)
 *      3V3 pin 1   GND pin 6   SDA pin 3 (RA3)   SCL pin 5 (RA2)
 *    ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC red 5 V wires . NOT connected
 *    Backup: motor B's pulse also comes out on pin 21 (RF4 -> OC3)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initDelay(), delay_ms(),
 *    initMotors(), set_motor(), initU2(), poll_rx(), putU2(), putsU2(), get_line(), initI2C(),
 *    too_long(), I2C_recover(), I2C_wait_for_idle(), I2CStart(), I2CRestart(), I2CStop(),
 *    I2CAck(), I2CNack(), I2CWrite(), I2CRead(), I2CReadsinglebyte(), I2CReadbytes(),
 *    I2CWritebyte(), initIMU(), imu_read_raw(), imu_calibrate(), imu_update(), Controller(),
 *    initControl(), help(), clamp_us(), start_run(), get_numbers(), handle_command(),
 *    dump_run(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include <stdio.h>
# include <stdlib.h>        // atof
# include <math.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz

// =====================================================================
// ELC 275 capstone - SEESAW PID TEST (one axis, two motors)
// Ethan DiMeo
//
// Works with the PID tuner GUI (pid_tuner.py, Serial mode, COM10, 115200)
// or with PuTTY (115200, Terminal -> Local echo: Force on).
//
// WIRING
//   (J12 layout from the parts list / drone wiring diagram)
//   ESC 1 (FL) = motor A: signal -> J12 pin 11 (RD0 = OC1), ground -> pin 14
//                motor on the side that goes UP when the angle is positive
//   ESC 2 (FR) = motor B: signal -> J12 pin 12 (RF2 = OC2), ground -> pin 20
//                (motor B's pulse ALSO comes out on pin 21 = RF4 = OC3, as a
//                 backup: if pin 12 does nothing, move the wire to pin 21)
//   ESC red 5 V wires: NOT connected.  Pins 8 and 10 are the debug UART - no ESCs there.
//   IMU (LSM6DSO, zip-tied in the middle of the seesaw, I2C2):
//     SDA -> RA3   SCL -> RA2   3.3V -> 3.3V   GND -> GND   (never 5 V)
//   SW1 on the kit = KILL
//
// THE CONTROL LAW (Kevin Lynch), every dt = 4 ms in the Timer4 interrupt:
//   e    = setpoint - angle
//   eint = eint + e*dt            (clamped to +/- EINTMAX)
//   edot = 0 - gyro_rate
//   u    = Kp*e + Ki*eint + Kd*edot      (clamped to +/- UMAX)
//   motor A = base A + u,  motor B = base B - u   (each clamped 1100..2000 us)
//
// COMMANDS (one per line)          the GUI sends the first five by itself
//   K,kp,ki,kd     set gains (replies A,kp,ki,kd)
//   S,deg          set the setpoint
//   R,from,to,ms   step test: PID on, hold "from" 1 s, step to "to",
//                  record every 4 ms for ms, then send RUN_BEGIN / D... / RUN_END
//   L,1  L,0       live data on / off
//   X              KILL, motors off
//   G              PID on (holds the setpoint)
//   M,a,b          manual: motor A = a us, motor B = b us, PID off
//   B,a,b          base pulse for motor A and B while the PID runs
//   Z              zero the angle: hold the seesaw level, then send Z
//   ?              help
//
// PROPS: only with the seesaw clamped/tethered, safety glasses, hands clear.
// =====================================================================

// ---------------- settings ----------------
#define USE_PITCH   0       // 0: seesaw tilt = roll  (IMU X arrow points along the pivot bolt)
                            // 1: seesaw tilt = pitch (IMU Y arrow points along the pivot bolt)
#define ESC_400HZ   0       // 0: 50 Hz ESC pulses (what worked on the bench)
                            // 1: 400 Hz (ESC reacts faster, better for tuning) - try once 50 Hz works

#define DT          0.004f  // control loop period, s
#define EINTMAX     20.0f   // integrator clamp, degree-seconds
#define UMAX        500.0f  // biggest correction, us (same as the GUI)
#define PULSE_OFF   1000.0f // us, motor stopped
#define PULSE_IDLE  1100.0f // us, slowest a motor goes while the PID runs (keeps it spinning)
#define PULSE_MAX   2000.0f // us, full throttle
#define TIP_LIMIT   45.0f   // degrees, past this = something is wrong, motors off
#define HOLD_TICKS  250     // step test holds "from" for 250 x 4 ms = 1 s
#define PRE_TICKS   50      // ...and records the last 50 of those (200 ms before the step)
#define REC_MAX     5100    // 20 s after the step + 200 ms before
#define LIVE_EVERY  5       // live line every 5th loop = 50 lines per second

#if ESC_400HZ
#define TICKS_PER_US 7.5f       // Timer2 = 60 MHz / 8,  PR2 18749 -> 2.5 ms (400 Hz)
#else
#define TICKS_PER_US 0.9375f    // Timer2 = 60 MHz / 64, PR2 18749 -> 20 ms (50 Hz)
#endif

// ---------------- shared with the interrupt ----------------
volatile int mode=0;                    // 0 = off, 1 = PID, 2 = manual
volatile float Kp=1.0f, Ki=0.0f, Kd=0.0f;   // start P only: tune Kp, then Kd, then Ki
volatile float setpoint=0.0f;
volatile float baseA=1500.0f, baseB=1500.0f;    // prof: 1.5 ms center
volatile float manA=PULSE_OFF, manB=PULSE_OFF;
volatile float angle_offset=0.0f;
volatile float ramp_us=PULSE_OFF;       // soft start: motors can't jump to full at once
volatile unsigned long loop_count=0;
volatile float now_angle=0, now_A=PULSE_OFF, now_B=PULSE_OFF, now_P=0, now_I=0, now_D=0;
volatile int off_reason=0;              // why the safety turned the motors off: 1 = IMU, 2 = tip limit

// step test
volatile int run_state=0;   // 0 none, 1 holding "from", 2 recording after the step, 3 done, 4 killed
volatile float run_from=0, run_to=0;
volatile int run_ticks=0, hold_count=0, rec_n=0, step_index=0;
float rec_angle[REC_MAX], rec_sp[REC_MAX], rec_A[REC_MAX];
float rec_P[REC_MAX], rec_I[REC_MAX], rec_D[REC_MAX];

int live_on=1;

// =====================================================================
// Delays: Timer3, 60 MHz / 256 = 234,375 counts per second -> 234 = 1 ms
// =====================================================================
// ---------------------------------------------------------------------------------------------
// void initDelay(void)
//   Starts Timer3 (1:256 prescale) so delay_ms() can count milliseconds.
// ---------------------------------------------------------------------------------------------
void initDelay(void)
{
    T3CON=0x8070;
}

// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
void delay_ms(int n)
{
    for (int i=0;i<n;i++){
        TMR3=0;
        while(TMR3<234);
    }
}

// =====================================================================
// Motors: Output Compare in PWM mode on Timer2
// =====================================================================
// ---------------------------------------------------------------------------------------------
// void initMotors(void)
//   Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC pulses (all start at
//   1000 us = stop).
// ---------------------------------------------------------------------------------------------
void initMotors(void)
{
    TRISDbits.TRISD0=0;
    RPD0R=12;               // RD0 = OC1 = motor A (J12 pin 11, ESC 1 FL)
    TRISFbits.TRISF2=0;
    RPF2R=11;               // RF2 = OC2 = motor B (J12 pin 12, ESC 2 FR)
    TRISFbits.TRISF4=0;
    RPF4R=11;               // RF4 = OC3 = motor B backup (J12 pin 21)

    T2CON=0x0000;
    TMR2=0;
    PR2=18749;
    OC1R=PULSE_OFF*TICKS_PER_US;    // all start at 1000 us so the ESCs arm
    OC1RS=PULSE_OFF*TICKS_PER_US;
    OC2R=PULSE_OFF*TICKS_PER_US;
    OC2RS=PULSE_OFF*TICKS_PER_US;
    OC3R=PULSE_OFF*TICKS_PER_US;
    OC3RS=PULSE_OFF*TICKS_PER_US;
    OC1CON=0x8006;          // PWM mode, Timer2
    OC2CON=0x8006;
    OC3CON=0x8006;
#if ESC_400HZ
    T2CON=0x8030;           // Timer2 ON, 1:8
#else
    T2CON=0x8060;           // Timer2 ON, 1:64
#endif
}

// ---------------------------------------------------------------------------------------------
// void set_motor(int n, float us)
//   Sets the ESC pulse for motor n (1-4); forced to 'stop' unless the drone is armed and
//   running.
// ---------------------------------------------------------------------------------------------
void set_motor(int n, float us)
{
    int ticks;
    if (us<PULSE_OFF) us=PULSE_OFF;     // never outside 1-2 ms
    if (us>PULSE_MAX) us=PULSE_MAX;
    ticks=us*TICKS_PER_US+0.5f;
    if (n==1) OC1RS=ticks;      // motor A: pin 11
    if (n==2){                  // motor B: pin 12 (and pin 21 backup)
        OC2RS=ticks;
        OC3RS=ticks;
    }
}

// =====================================================================
// UART2 -> kit USB port (COM10), 115200
// Received bytes go into rx_buf. putU2 also checks for received bytes
// while it waits, so nothing is lost while the PIC is printing.
// =====================================================================
#define RX_SIZE 256
volatile char rx_buf[RX_SIZE];
volatile int rx_head=0, rx_tail=0;
int rx_line_start=1;

// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
void initU2(void)
{
    ANSELGbits.ANSG6=0;
    TRISGbits.TRISG6=1;
    U2RXR=1;                // RG6 = U2RX
    ANSELBbits.ANSB14=0;
    TRISBbits.TRISB14=0;
    RPB14R=2;               // RB14 = U2TX
    U2BRG=129;              // 60 MHz / (4 x 130) = 115384 ~ 115200
    U2MODE=0x8008;
    U2STA=0x1400;
}

// ---------------------------------------------------------------------------------------------
// void poll_rx(void)
//   Reads every waiting UART2 character into the line buffer; an X at the start of a line kills
//   the motors at once.
// ---------------------------------------------------------------------------------------------
void poll_rx(void)
{
    char c;
    if (U2STAbits.OERR) U2STAbits.OERR=0;   // overrun stops the receiver, clear it
    while (U2STAbits.URXDA){
        c=U2RXREG;
        if (c=='X' && rx_line_start==1){    // KILL right away, even in the middle of printing
            mode=0;
            if (run_state==1 || run_state==2) run_state=4;
        }
        rx_line_start=(c=='\r' || c=='\n');
        rx_buf[rx_head]=c;
        rx_head=(rx_head+1)%RX_SIZE;
    }
}

// ---------------------------------------------------------------------------------------------
// int putU2(int c)
//   Sends one character out UART2 to the PC (waits while the transmit buffer is full).
// ---------------------------------------------------------------------------------------------
int putU2(int c)
{
    while (U2STAbits.UTXBF) poll_rx();
    U2TXREG=c;
    return c;
}

// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Sends a text line to the PC over UART2, followed by a new line.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s)
{
    while (*s) putU2(*s++);
    putU2('\r');
    putU2('\n');
}

// one whole line (ends on \r or \n) -> line[], returns 1 when a line is ready
char line[64];
int line_len=0;

// ---------------------------------------------------------------------------------------------
// int get_line(void)
//   Collects received characters until a full command line has arrived. 1 = a line is ready.
// ---------------------------------------------------------------------------------------------
int get_line(void)
{
    char c;
    poll_rx();
    while (rx_tail!=rx_head){
        c=rx_buf[rx_tail];
        rx_tail=(rx_tail+1)%RX_SIZE;
        if (c=='\r' || c=='\n'){
            if (line_len>0){
                line[line_len]=0;
                line_len=0;
                return 1;
            }
        }
        else if (line_len<63){
            line[line_len]=c;
            line_len++;
        }
    }
    return 0;
}

// =====================================================================
// IMU: SparkFun LSM6DSO on I2C2 (class code from Lab 5 + imu.c)
// =====================================================================
int imu_addr=0x6B;              // 0x6B default, 0x6A with the ADR jumper cut (initIMU finds it)
#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12
#define OUTX_L_G    0x22        // 12 bytes: gyro X,Y,Z then accel X,Y,Z
#define ALPHA       0.98f       // complementary filter: 98% gyro, 2% accelerometer
#define GYRO_SCALE  0.0175f     // 17.5 mdps per count at +/-500 dps
#define RAD_TO_DEG  57.29578f

volatile float roll=0, pitch=0, roll_rate=0, pitch_rate=0;
volatile int imu_error=0;
float gx_bias=0, gy_bias=0;

// ---------------------------------------------------------------------------------------------
// void initI2C(void)
//   Sets up I2C2 (pins 3/5) at 100 kHz with the internal pull-ups on.
// ---------------------------------------------------------------------------------------------
void initI2C(void)
{
    CNPUAbits.CNPUA3=1;
    CNPUAbits.CNPUA2=1;
    I2C2BRG=295;            // 100 kHz (what worked with the ID check)
    I2C2CON=0x8000;
    delay_ms(1);
}

// CAPSTONE: the class I2C code waits with "while (...);" loops. If the bus
// glitches (motor noise, loose wire) one of them never ends, and because the
// IMU is read inside the 4 ms interrupt, the whole PIC freezes: no PuTTY
// output, motors stuck. Now every wait gives up after I2C_TIMEOUT passes
// (about 1 ms; one byte at 100 kHz takes 0.09 ms) and sets i2c_stuck.
// (20000 was too long: a dead IMU stretched the 4 ms loop to 13 ms.)
#define I2C_TIMEOUT 2000
int i2c_stuck=0;

// ---------------------------------------------------------------------------------------------
// int too_long(int *t)
//   Timeout counter for the I2C wait loops: 1 = waited too long (marks the bus as stuck).
// ---------------------------------------------------------------------------------------------
// call inside every I2C wait loop: returns 1 when it has waited too long
int too_long(int *t)
{
    (*t)++;
    if (*t>I2C_TIMEOUT){
        i2c_stuck=1;
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------------------------
// void I2C_recover(void)
//   Turns I2C2 off and on again to clear a bus collision, and reports the error.
// ---------------------------------------------------------------------------------------------
// turn the I2C module off and on again (clears a bus collision), report the error
void I2C_recover(void)
{
    I2C2CONbits.ON=0;
    I2C2STATbits.BCL=0;
    I2C2STATbits.IWCOL=0;
    I2C2CONbits.ON=1;
    i2c_stuck=0;
    imu_error=1;
}

// ---------------------------------------------------------------------------------------------
// void I2C_wait_for_idle(void)
//   Waits until the I2C2 module is idle (gives up after a timeout).
// ---------------------------------------------------------------------------------------------
void I2C_wait_for_idle(void)
{
    int t=0;
    while(I2C2CON & 0x1F) if (too_long(&t)) return;
    while(I2C2STATbits.TRSTAT) if (too_long(&t)) return;
}

// ---------------------------------------------------------------------------------------------
// void I2CStart(void)
//   Sends an I2C start condition.
// ---------------------------------------------------------------------------------------------
void I2CStart(void)
{
    int t=0;
    if (i2c_stuck) return;
    I2C_wait_for_idle();
    I2C2CONbits.SEN=1;
    while (I2C2CONbits.SEN) if (too_long(&t)) return;
}

// ---------------------------------------------------------------------------------------------
// void I2CRestart(void)
//   Sends an I2C repeated-start condition.
// ---------------------------------------------------------------------------------------------
void I2CRestart(void)
{
    int t=0;
    if (i2c_stuck) return;
    I2C_wait_for_idle();
    I2C2CONbits.RSEN=1;
    while (I2C2CONbits.RSEN) if (too_long(&t)) return;
}

// ---------------------------------------------------------------------------------------------
// void I2CStop(void)
//   Sends an I2C stop condition.
// ---------------------------------------------------------------------------------------------
void I2CStop(void)
{
    int t=0;
    if (i2c_stuck) return;
    I2C_wait_for_idle();
    I2C2CONbits.PEN=1;
    while (I2C2CONbits.PEN) if (too_long(&t)) return;
}

// ---------------------------------------------------------------------------------------------
// void I2CAck(void)
//   Answers a received byte with ACK (more bytes wanted).
// ---------------------------------------------------------------------------------------------
void I2CAck(void)
{
    int t=0;
    if (i2c_stuck) return;
    I2C_wait_for_idle();
    I2C2CONbits.ACKDT=0;
    I2C2CONbits.ACKEN=1;
    while (I2C2CONbits.ACKEN) if (too_long(&t)) return;
}

// ---------------------------------------------------------------------------------------------
// void I2CNack(void)
//   Answers a received byte with NACK (last byte).
// ---------------------------------------------------------------------------------------------
void I2CNack(void)
{
    int t=0;
    if (i2c_stuck) return;
    I2C_wait_for_idle();
    I2C2CONbits.ACKDT=1;
    I2C2CONbits.ACKEN=1;
    while (I2C2CONbits.ACKEN) if (too_long(&t)) return;
}

// ---------------------------------------------------------------------------------------------
// void I2CWrite(int data)
//   Sends one byte on I2C2.
// ---------------------------------------------------------------------------------------------
void I2CWrite(int data)
{
    int t=0;
    if (i2c_stuck) return;
    I2C2TRN=data;
    while (I2C2STATbits.TBF) if (too_long(&t)) return;
    I2C_wait_for_idle();
    if (I2C2STATbits.ACKSTAT) imu_error=1;  // no answer: flag it instead of hanging
}

// ---------------------------------------------------------------------------------------------
// int I2CRead(void)
//   Receives one byte on I2C2.
// ---------------------------------------------------------------------------------------------
int I2CRead(void)
{
    int t=0;
    if (i2c_stuck) return 0;
    I2C2CONbits.RCEN=1;
    while (I2C2CONbits.RCEN) if (too_long(&t)) return 0;
    return I2C2RCV & 0xFF;
}

// ---------------------------------------------------------------------------------------------
// int I2CReadsinglebyte(int address, int reg)
//   Reads one register of the I2C device at address.
// ---------------------------------------------------------------------------------------------
int I2CReadsinglebyte(int address, int reg)
{
    int a;
    I2CStart();
    I2CWrite(address<<1);
    I2CWrite(reg);
    I2CRestart();
    I2CWrite((address<<1)+1);
    a=I2CRead();
    I2CNack();
    I2CStop();
    if (i2c_stuck) I2C_recover();
    return a;
}

// ---------------------------------------------------------------------------------------------
// void I2CReadbytes(int address, int reg, int b[], int n)
//   Reads n registers in a row from the I2C device at address into b[].
// ---------------------------------------------------------------------------------------------
void I2CReadbytes(int address, int reg, int b[], int n)
{
    I2CStart();
    I2CWrite(address<<1);
    I2CWrite(reg);
    I2CRestart();
    I2CWrite((address<<1)+1);
    for (int i=0; i<n; i++){
        b[i]=I2CRead();
        if (i<n-1) I2CAck();
        else       I2CNack();
    }
    I2CStop();
    if (i2c_stuck) I2C_recover();
}

// ---------------------------------------------------------------------------------------------
// void I2CWritebyte(int address, int reg, int val)
//   Writes one value to register reg of the I2C device at address.
// ---------------------------------------------------------------------------------------------
void I2CWritebyte(int address, int reg, int val)
{
    I2CStart();
    I2CWrite(address<<1);
    I2CWrite(reg);
    I2CWrite(val);
    I2CStop();
    if (i2c_stuck) I2C_recover();
}

// ---------------------------------------------------------------------------------------------
// int initIMU(void)
//   Starts I2C2, finds the IMU at address 0x6B or 0x6A, writes its settings. Returns WHO_AM_I
//   (should be 0x6C).
// ---------------------------------------------------------------------------------------------
int initIMU(void)
{
    int id;
    initI2C();
    id=I2CReadsinglebyte(0x6B, WHO_AM_I);     // try the default address
    if (id==0x6C) imu_addr=0x6B;
    else{
        id=I2CReadsinglebyte(0x6A, WHO_AM_I); // ADR jumper cut -> 0x6A
        if (id==0x6C) imu_addr=0x6A;
    }
    I2CWritebyte(imu_addr, CTRL3_C, 0x44);    // BDU + auto-increment
    I2CWritebyte(imu_addr, CTRL1_XL, 0x78);   // accel 833 Hz, +/-4 g
    I2CWritebyte(imu_addr, CTRL2_G, 0x74);    // gyro 833 Hz, +/-500 dps
    delay_ms(100);
    return id;
}

// ---------------------------------------------------------------------------------------------
// void imu_read_raw(int r[6])
//   Reads gyro X,Y,Z and accel X,Y,Z as signed raw counts (I2C version).
// ---------------------------------------------------------------------------------------------
// r[0..2] = gyro X,Y,Z   r[3..5] = accel X,Y,Z  (signed counts)
void imu_read_raw(int r[6])
{
    int b[12];
    I2CReadbytes(imu_addr, OUTX_L_G, b, 12);
    for (int i=0; i<6; i++){
        r[i]=(short)((b[2*i+1]<<8) | b[2*i]);
    }
}

// ---------------------------------------------------------------------------------------------
// void imu_calibrate(void)
//   Averages the IMU while the drone sits still and level to find the gyro and accel offsets
//   (retries if it moved).
// ---------------------------------------------------------------------------------------------
// seesaw must sit still: average 500 gyro readings = bias
void imu_calibrate(void)
{
    int r[6];
    float sx=0, sy=0, ax=0, ay=0, az=0;
    for (int i=0; i<500; i++){
        imu_read_raw(r);
        sx=sx+r[0]*GYRO_SCALE;
        sy=sy+r[1]*GYRO_SCALE;
        ax=ax+r[3];
        ay=ay+r[4];
        az=az+r[5];
        delay_ms(2);
    }
    gx_bias=sx/500;
    gy_bias=sy/500;
    roll =atan2f(ay, az)*RAD_TO_DEG;
    pitch=atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;
}

// ---------------------------------------------------------------------------------------------
// void imu_update(void)
//   Runs every 4 ms: reads the IMU and updates roll, pitch, yaw with the complementary filter
//   (alpha 0.98).
// ---------------------------------------------------------------------------------------------
// every 4 ms from the interrupt
void imu_update(void)
{
    int r[6];
    float ax, ay, az;
    imu_error=0;                // check this read only (a NACK below sets it again)
    imu_read_raw(r);
    if (imu_error==1) return;   // bad read: keep the last good angle
    roll_rate = r[0]*GYRO_SCALE-gx_bias;        // + = side the IMU's Y arrow points to goes up
    pitch_rate=-(r[1]*GYRO_SCALE-gy_bias);      // + = side the IMU's X arrow points to goes up
    ax=r[3];
    ay=r[4];
    az=r[5];
    roll =ALPHA*(roll +roll_rate *DT)+(1-ALPHA)*atan2f(ay, az)*RAD_TO_DEG;
    pitch=ALPHA*(pitch+pitch_rate*DT)+(1-ALPHA)*atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;
}

// =====================================================================
// The control loop: Timer4 interrupt every 4 ms (250 Hz)
// sense -> PID -> motors -> record
// =====================================================================
// ---------------------------------------------------------------------------------------------
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
//   Timer4 interrupt, every 4 ms: read IMU -> angle filter -> PID -> motor pulses, plus all
//   safety checks.
// ---------------------------------------------------------------------------------------------
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
{
    static float eint=0;
    float angle, rate, e, edot, P, I, D, u, A, B;

    // 1. sense
    imu_update();
    if (USE_PITCH==1){
        angle=pitch;
        rate=pitch_rate;
    }
    else{
        angle=roll;
        rate=roll_rate;
    }
    angle=angle-angle_offset;

    // 2. safety while the PID flies (manual mode doesn't use the angle, so it isn't cut)
    //    off_reason tells main() to print why
    if (mode==1 && imu_error==1){
        mode=0;
        off_reason=1;
    }
    if (mode==1 && (angle>TIP_LIMIT || angle<-TIP_LIMIT)){
        mode=0;
        off_reason=2;
    }
    if (mode==0 && (run_state==1 || run_state==2)) run_state=4;

    // 3. soft start: the motor limit climbs 1000 -> 2000 us in 1 s
    if (mode==0) ramp_us=PULSE_OFF;
    else if (ramp_us<PULSE_MAX) ramp_us=ramp_us+4.0f;

    // 4. step test: after the soft start, hold "from" 1 s, then step to "to"
    if (run_state==1 && ramp_us>=PULSE_MAX){
        hold_count++;
        if (hold_count>=HOLD_TICKS){
            setpoint=run_to;
            step_index=rec_n;       // this sample is t = 0
            run_state=2;
        }
    }

    // 5. PID (Lynch)
    e=setpoint-angle;
    edot=0-rate;
    if (mode!=1 || ramp_us<PULSE_MAX) eint=0;   // only integrate while the PID really flies
    else eint=eint+e*DT;
    if (eint> EINTMAX) eint= EINTMAX;           // anti-windup
    if (eint<-EINTMAX) eint=-EINTMAX;
    P=Kp*e;
    I=Ki*eint;
    D=Kd*edot;
    u=P+I+D;
    if (u> UMAX) u= UMAX;
    if (u<-UMAX) u=-UMAX;

    // 6. motors
    if (mode==1){
        // side A too high -> angle > setpoint -> e < 0 -> u < 0
        // -> motor A slows, motor B speeds up -> side A comes back down
        A=baseA+u;
        B=baseB-u;
        if (A<PULSE_IDLE) A=PULSE_IDLE;
        if (A>PULSE_MAX)  A=PULSE_MAX;
        if (B<PULSE_IDLE) B=PULSE_IDLE;
        if (B>PULSE_MAX)  B=PULSE_MAX;
    }
    else if (mode==2){
        A=manA;
        B=manB;
    }
    else{
        A=PULSE_OFF;
        B=PULSE_OFF;
    }
    if (A>ramp_us) A=ramp_us;       // soft start
    if (B>ramp_us) B=ramp_us;
    set_motor(1, A);
    set_motor(2, B);

    // 7. keep the latest values for the live data lines
    now_angle=angle;
    now_A=A;
    now_B=B;
    now_P=P;
    now_I=I;
    now_D=D;

    // 8. step test recording
    if ((run_state==1 && hold_count>HOLD_TICKS-PRE_TICKS) || run_state==2){
        if (rec_n<REC_MAX){
            rec_angle[rec_n]=angle;
            rec_sp[rec_n]=setpoint;
            rec_A[rec_n]=A;
            rec_P[rec_n]=P;
            rec_I[rec_n]=I;
            rec_D[rec_n]=D;
            rec_n++;
        }
        if (run_state==2 && (rec_n-step_index>=run_ticks || rec_n>=REC_MAX)) run_state=3;
    }

    LATHbits.LATH0=(mode!=0);       // LED1 = motors on
    loop_count++;
    IFS0bits.T4IF=0;
}

// ---------------------------------------------------------------------------------------------
// void initControl(void)
//   Sets up Timer4 to interrupt every 4 ms; that interrupt runs the whole flight loop.
// ---------------------------------------------------------------------------------------------
void initControl(void)
{
    T4CON=0x0000;
    TMR4=0;
    PR4=3749;               // (3749+1) x 64 / 60 MHz = 4 ms
    IFS0bits.T4IF=0;
    IPC4bits.T4IP=5;
    IEC0bits.T4IE=1;
    INTCONbits.MVEC=1;
    __builtin_enable_interrupts();
    T4CON=0x8060;           // Timer4 ON, 1:64
}

// =====================================================================
// Commands
// =====================================================================
// ---------------------------------------------------------------------------------------------
// void help(void)
//   Prints the list of tuner commands.
// ---------------------------------------------------------------------------------------------
void help(void)
{
    putsU2("Commands: K,kp,ki,kd  S,deg  R,from,to,ms  L,1/L,0  X=KILL");
    putsU2("          G=PID on  M,a,b=manual us  B,a,b=base us  Z=zero angle  ?=help");
}

// ---------------------------------------------------------------------------------------------
// float clamp_us(float us)
//   Clamps a pulse to 1000-2000 us.
// ---------------------------------------------------------------------------------------------
float clamp_us(float us)
{
    if (us<PULSE_OFF) us=PULSE_OFF;
    if (us>PULSE_MAX) us=PULSE_MAX;
    return us;
}

// ---------------------------------------------------------------------------------------------
// void start_run(float from, float to, int ms)
//   Starts a recorded step test from angle 'from' to angle 'to', lasting ms milliseconds.
// ---------------------------------------------------------------------------------------------
void start_run(float from, float to, int ms)
{
    if (run_state!=0) return;       // one test at a time
    if (ms>20000) ms=20000;
    if (ms<100) ms=100;
    run_from=from;
    run_to=to;
    run_ticks=ms/4;
    hold_count=0;
    rec_n=0;
    step_index=0;
    setpoint=from;
    mode=1;                         // PID on (soft start if the motors were off)
    run_state=1;                    // last: the interrupt starts the test now
    putsU2("Step test started");
}

// ---------------------------------------------------------------------------------------------
// int get_numbers(char *s, float v[], int max)
//   Reads up to max numbers after the commas in a command ('K,1.5,0,0.4'). Returns how many.
// ---------------------------------------------------------------------------------------------
// "K,1.5,0,0.4" -> v[0]=1.5 v[1]=0 v[2]=0.4, returns how many numbers it found.
// (Used instead of sscanf: sscanf needs a heap, atof does not.)
int get_numbers(char *s, float v[], int max)
{
    int n=0;
    while (*s && n<max){
        if (*s==','){
            v[n]=atof(s+1);     // atof reads the number right after the comma
            n++;
        }
        s++;
    }
    return n;
}

// ---------------------------------------------------------------------------------------------
// void handle_command(char *s)
//   Runs one tuner command line (K, S, R, L, G, M, B, Z, X, ?).
// ---------------------------------------------------------------------------------------------
void handle_command(char *s)
{
    char out[80];
    float v[3];
    int n;

    n=get_numbers(s, v, 3);

    if (s[0]=='K'){
        if (n==3){
            Kp=v[0];
            Ki=v[1];
            Kd=v[2];
        }
        sprintf(out, "A,%.4f,%.4f,%.4f", Kp, Ki, Kd);
        putsU2(out);
    }
    else if (s[0]=='S'){
        if (n==1 && run_state==0) setpoint=v[0];
    }
    else if (s[0]=='R'){
        if (n==3) start_run(v[0], v[1], (int)v[2]);
    }
    else if (s[0]=='L'){
        live_on=(s[2]=='1');
    }
    else if (s[0]=='X'){
        mode=0;
        if (run_state==1 || run_state==2) run_state=4;
        putsU2("KILL - motors off");
    }
    else if (s[0]=='G'){
        mode=1;
        putsU2("PID on");
    }
    else if (s[0]=='M'){
        if (n==2){
            manA=clamp_us(v[0]);
            manB=clamp_us(v[1]);
            mode=2;
            sprintf(out, "Manual: A %.0f us  B %.0f us", manA, manB);
            putsU2(out);
        }
    }
    else if (s[0]=='B'){
        if (n==2){
            baseA=clamp_us(v[0]);
            baseB=clamp_us(v[1]);
        }
        sprintf(out, "Base: A %.0f us  B %.0f us", baseA, baseB);
        putsU2(out);
    }
    else if (s[0]=='Z'){
        angle_offset=angle_offset+now_angle;
        sprintf(out, "Angle zeroed (offset %.2f deg)", angle_offset);
        putsU2(out);
    }
    else if (s[0]=='?'){
        help();
    }
    else{
        putsU2("unknown command, type ? for help");
    }
}

// ---------------------------------------------------------------------------------------------
// void dump_run(void)
//   Sends the recorded step test to the GUI between RUN_BEGIN and RUN_END.
// ---------------------------------------------------------------------------------------------
// send the recorded step test to the GUI
void dump_run(void)
{
    char s[100];
    putsU2("RUN_BEGIN");
    for (int i=0; i<rec_n; i++){
        sprintf(s, "D,%d,%.2f,%.2f,%.0f,%.1f,%.1f,%.1f", (i-step_index)*4,
                rec_angle[i], rec_sp[i], rec_A[i], rec_P[i], rec_I[i], rec_D[i]);
        putsU2(s);
        WDTCONbits.WDTCLRKEY=0x5743;    // long dump: keep the watchdog happy
    }
    putsU2("RUN_END");
    run_state=0;
}

// =====================================================================
// main: setup, then commands + data. The flying is in the interrupt.
// =====================================================================
// ---------------------------------------------------------------------------------------------
// void main(void)
//   Program entry: sets everything up, then reads tuner commands and sends live data; the PID
//   runs in the Timer4 interrupt.
// ---------------------------------------------------------------------------------------------
void main(void)
{
    char s[100];
    int id;
    unsigned long last_live=0;
    unsigned long last_blink=0;

    initDelay();
    initMotors();           // right away: both ESCs see a steady 1000 us and arm
    initU2();

    TRISHbits.TRISH0=0;     // LED1 = motors on
    LATHbits.LATH0=0;
    TRISHbits.TRISH1=0;     // LED2 = heartbeat: blinks while the program runs
    LATHbits.LATH1=0;
    ANSELBbits.ANSB12=0;    // SW1 (RB12) = KILL, input with pull-up
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;

    putsU2("");
    putsU2("Seesaw PID test - Ethan DiMeo");
    id=initIMU();
    sprintf(s, "WHO_AM_I = 0x%X at address 0x%X (LSM6DSO should be 0x6C)", id, imu_addr);
    putsU2(s);
    putsU2("Calibrating gyro - keep the seesaw still");
    imu_calibrate();
    sprintf(s, "Angle now: roll %.1f  pitch %.1f  (using %s, must be within +/-45)",
            roll, pitch, USE_PITCH ? "pitch" : "roll");
    putsU2(s);
    putsU2("ESC signals: motor A = J12 pin 11 (RD0), motor B = pin 12 (RF2), backup pin 21 (RF4)");
    initControl();          // 4 ms control loop starts here
    help();

    while (1){
        WDTCONbits.WDTCLRKEY=0x5743;        // clear the watchdog (config.bits turns it on)

        if (loop_count-last_blink>=62){     // 62 x 4 ms = about 0.25 s
            last_blink=loop_count;
            LATHINV=0x2;                    // toggle LED2: main AND the interrupt are alive
        }

        if (PORTBbits.RB12==0){             // SW1 = KILL
            mode=0;
            if (run_state==1 || run_state==2) run_state=4;
        }

        while (get_line()) handle_command(line);

        if (run_state==3) dump_run();
        if (run_state==4){
            putsU2("Step test stopped (KILL, tip limit or IMU error)");
            run_state=0;
        }

        if (off_reason==1){
            putsU2("MOTORS OFF: the IMU did not answer (check SDA, SCL, 3.3V, GND)");
            off_reason=0;
        }
        if (off_reason==2){
            sprintf(s, "MOTORS OFF: angle %.1f deg is past the %.0f deg tip limit", now_angle, TIP_LIMIT);
            putsU2(s);
            putsU2("  -> IMU flat, chip side up? right axis (USE_PITCH)? hold level + send Z");
            off_reason=0;
        }

        if (live_on && loop_count-last_live>=LIVE_EVERY){
            last_live=loop_count;
            // D,t_ms,angle,setpoint,motorA,P,I,D,motorB   (the GUI uses the first 8)
            sprintf(s, "D,%lu,%.2f,%.2f,%.0f,%.1f,%.1f,%.1f,%.0f", loop_count*4,
                    now_angle, setpoint, now_A, now_P, now_I, now_D, now_B);
            putsU2(s);
        }
    }
}
