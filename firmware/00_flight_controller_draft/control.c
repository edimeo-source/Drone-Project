/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/control.c
 *  What    : The 4 ms control loop (Timer4 interrupt): IMU -> filter -> PID -> motors, plus
 *            arming and every safety check.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): armed_limit(), Controller(),
 *    initControl()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include "control.h"
# include "imu.h"
# include "pid.h"
# include "motors.h"

// The control loop, Lynch style: ONE timer interrupt at a fixed rate does
// sense -> compute -> actuate. main() only handles the phone.
// Timer4: 60 MHz / 64 = 937,500 counts per second
// period = (PR4+1) x 64 / 60 MHz = 3750 x 64 / 60 MHz = 4 ms (250 Hz)

#define EINTMAX     20.0f       // integrator clamp, degree-seconds
#define UMAX        200.0f      // biggest correction from one axis, us
#define YAW_UMAX    100.0f      // yaw gets less so it can't steal lift
#define EINT_ON_US  1200.0f     // below this throttle the drone is on the ground: hold integrators at 0
#define TIP_LIMIT   45.0f       // tilted past this = crashed, cut the motors

volatile int armed=0;
volatile float throttle_us=PULSE_OFF;
volatile float Kp=1.0f, Ki=0.0f, Kd=0.0f;   // start P only, Lynch's tuning order: P, then D, then I
volatile float Kp_yaw=1.0f, Ki_yaw=0.0f;
volatile unsigned long loop_count=0;

// ---------------------------------------------------------------------------------------------
// float armed_limit(float us)
//   While armed, keeps a motor pulse between idle and full so no motor stops in the air.
// ---------------------------------------------------------------------------------------------
// while armed, keep every motor between idle and full so none of them stops in the air
float armed_limit(float us)
{
    if (us<PULSE_IDLE) us=PULSE_IDLE;
    if (us>PULSE_MAX) us=PULSE_MAX;
    return us;
}

// ---------------------------------------------------------------------------------------------
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
//   Timer4 interrupt, every 4 ms: read IMU -> angle filter -> PID -> motor pulses, plus all
//   safety checks.
// ---------------------------------------------------------------------------------------------
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
{
    static float eint_roll=0, eint_pitch=0, eint_yaw=0;   // Lynch: integrator state is static
    float e, edot;
    float u_roll, u_pitch, u_yaw;
    float T;

    LATHbits.LATH2=1;       // LED3 on: put the scope here to see how long the loop takes

    // 1. sense
    imu_update();

    // 2. roll: reference 0 degrees (level), e = r - s, edot = rdot - sdot = 0 - roll_rate
    e=0-roll;
    edot=0-roll_rate;
    u_roll=pid(e, edot, Kp, Ki, Kd, &eint_roll, EINTMAX, UMAX);

    // 3. pitch: same thing
    e=0-pitch;
    edot=0-pitch_rate;
    u_pitch=pid(e, edot, Kp, Ki, Kd, &eint_pitch, EINTMAX, UMAX);

    // 4. yaw: no angle to hold, so hold the spin RATE at 0 (PI, no D)
    e=0-yaw_rate;
    u_yaw=pid(e, 0, Kp_yaw, Ki_yaw, 0, &eint_yaw, EINTMAX, YAW_UMAX);

    // 5. safety
    if (imu_error==1) armed=0;
    if (roll>TIP_LIMIT || roll<-TIP_LIMIT || pitch>TIP_LIMIT || pitch<-TIP_LIMIT) armed=0;

    // 6. mixer -> ESC pulses
    T=throttle_us;
    if (T<EINT_ON_US){          // on the ground: don't let the integrators wind up
        eint_roll=0;
        eint_pitch=0;
        eint_yaw=0;
    }

    if (armed==1){
        LATHbits.LATH0=1;       // LED1 = armed
        // + u_roll speeds up the left side, + u_pitch the front, + u_yaw one diagonal pair
        set_motor(1, armed_limit(T+u_roll+u_pitch+u_yaw));     // front left
        set_motor(2, armed_limit(T-u_roll+u_pitch-u_yaw));     // front right
        set_motor(3, armed_limit(T+u_roll-u_pitch-u_yaw));     // back left
        set_motor(4, armed_limit(T-u_roll-u_pitch+u_yaw));     // back right
    }
    else{
        LATHbits.LATH0=0;
        motors_off();
    }

    loop_count++;
    LATHbits.LATH2=0;
    IFS0bits.T4IF=0;        // clear the flag last
}

// ---------------------------------------------------------------------------------------------
// void initControl(void)
//   Sets up Timer4 to interrupt every 4 ms; that interrupt runs the whole flight loop.
// ---------------------------------------------------------------------------------------------
void initControl(void)
{
    TRISHbits.TRISH0=0;     // LED1 armed
    TRISHbits.TRISH2=0;     // LED3 loop timing
    LATHbits.LATH0=0;
    LATHbits.LATH2=0;

    T4CON=0x0000;
    TMR4=0;
    PR4=3749;               // 4 ms
    IFS0bits.T4IF=0;
    IPC4bits.T4IP=5;
    IEC0bits.T4IE=1;
    INTCONbits.MVEC=1;
    asm("ei");
    T4CON=0x8060;           // Timer4 ON, 1:64
}
