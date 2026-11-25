/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/02_SewSaw_step0-1/SewSaw.X/control.c
 *  What    : The 4 ms control loop (Timer4 interrupt): IMU -> filter -> PID -> motors, plus
 *            arming and every safety check.
 *  Version : SewSaw v1   |   Date: 2026-11-16
 *  Status  : WORKED: step 0 (IMU check) passed and step 1 ran on the board Nov 16.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC pins 11/12 are reserved for the seesaw motors but not driven yet (motors.c arrives
 *      in step 2-3)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): motor_limit(), Controller(),
 *    initControl(), control_pause(), control_resume()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include "control.h"
# include "imu.h"
# include "pid.h"
# include "tuner.h"
# include "noise.h"

// The control loop, Lynch style: ONE timer interrupt at a fixed rate does
// sense -> compute -> (Step 3: actuate). main() only talks to the PC.
// Timer4: 60 MHz / 64 = 937,500 counts per second
// period = (PR4+1) x 64 / 60 MHz = 3750 x 64 / 60 MHz = 4 ms (250 times a second)
//
// Seesaw: the beam tilts in ROLL (IMU X arrow along the pivot rod, text facing you).
//   motor A = left end  (ESC on pin 11, Step 3) = 1500 + u
//   motor B = right end (ESC on pin 12, Step 3) = 1500 - u
//   left end up -> roll + -> e < 0 -> u < 0 -> A slows, B speeds up -> left end comes down
// No motors yet: motor_a and motor_b are only calculated and sent to the PC.

#define EINTMAX   20.0f     // integrator clamp, degree-seconds (same as the tuner)
#define UMAX      500.0f    // biggest correction, us (same as the tuner)
#define BASE_US   1500.0f   // motor command with no correction (prof's 300 counts)
#define MOTOR_MIN 1000.0f   // 1 ms
#define MOTOR_MAX 2000.0f   // 2 ms
#define MOTOR_OFF 1000.0f   // 1 ms = ESC off

volatile float Kp=1.0f, Ki=0.0f, Kd=0.0f;  // start with P only (Lynch's order: P, then D, then I)
volatile float setpoint=0.0f;
volatile int pid_on=1;
volatile float u=0, p_term=0, i_term=0, d_term=0;
volatile float motor_a=MOTOR_OFF, motor_b=MOTOR_OFF;
volatile unsigned long loop_count=0;
volatile float loop_period_us=4000.0f, loop_busy_us=0;

float eint=0;                   // Lynch: the integrator state lives with the loop
unsigned int last_count=0;      // core timer at the start of the last loop

// ---------------------------------------------------------------------------------------------
// float motor_limit(float us)
//   Clamps a motor pulse to the allowed range.
// ---------------------------------------------------------------------------------------------
float motor_limit(float us)
{
    if (us<MOTOR_MIN) us=MOTOR_MIN;
    if (us>MOTOR_MAX) us=MOTOR_MAX;
    return us;
}

// ---------------------------------------------------------------------------------------------
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
//   Timer4 interrupt, every 4 ms: read IMU -> angle filter -> PID -> motor pulses, plus all
//   safety checks.
// ---------------------------------------------------------------------------------------------
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
{
    unsigned int now;
    float e, edot;

    LATHbits.LATH2=1;                   // LED3 on: scope this pin to see the loop (4 ms apart)

    now=_CP0_GET_COUNT();               // core timer: 60 counts per microsecond
    loop_period_us=(now-last_count)/60.0f;
    last_count=now;

    // 1. sense
    imu_update();

    // 2. a step test from the tuner may change the setpoint
    tuner_setpoint_step();

    // 3. PID, Lynch form. Reference = setpoint, sensed = roll (the seesaw angle).
    e=setpoint-roll;                    // e = r - s
    edot=0-roll_rate;                   // edot = rdot - sdot (the setpoint holds still, so rdot = 0)
    u=pid(e, edot, Kp, Ki, Kd, &eint, EINTMAX, UMAX);
    p_term=Kp*e;                        // the three parts of u, for the tuner's graph
    i_term=Ki*eint;
    d_term=Kd*edot;

    // 4. seesaw mixer
    if (pid_on==1 && imu_error==0){
        motor_a=motor_limit(BASE_US+u);
        motor_b=motor_limit(BASE_US-u);
        LATHbits.LATH0=1;               // LED1 = motor output on
    }
    else{
        motor_a=MOTOR_OFF;
        motor_b=MOTOR_OFF;
        eint=0;                         // don't wind up the integrator while off
        LATHbits.LATH0=0;
    }
    // Step 3 adds: set_motor(1, motor_a); set_motor(2, motor_b);

    // 5. recordings (tuner step test, noise test)
    tuner_record();
    noise_sample();

    loop_count++;
    loop_busy_us=(_CP0_GET_COUNT()-now)/60.0f;
    LATHbits.LATH2=0;                   // LED3 off
    IFS0bits.T4IF=0;                    // clear the flag last
}

// ---------------------------------------------------------------------------------------------
// void initControl(void)
//   Sets up Timer4 to interrupt every 4 ms; that interrupt runs the whole flight loop.
// ---------------------------------------------------------------------------------------------
void initControl(void)
{
    eint=0;
    last_count=_CP0_GET_COUNT();

    T4CON=0x0000;           // Timer4 off while setting it up
    TMR4=0;
    PR4=3749;               // 3750 counts x 64 / 60 MHz = 4 ms
    IFS0bits.T4IF=0;
    IPC4bits.T4IP=5;        // priority 5 (the UART receive is 3)
    IEC0bits.T4IE=1;
    T4CON=0x8060;           // Timer4 ON, 1:64
}

// ---------------------------------------------------------------------------------------------
// void control_pause(void)
//   Motors off and the 4 ms loop stopped (used while calibrating or printing reports).
// ---------------------------------------------------------------------------------------------
void control_pause(void)
{
    IEC0bits.T4IE=0;        // the loop stops running
}

// ---------------------------------------------------------------------------------------------
// void control_resume(void)
//   Clears the PID and restarts the 4 ms loop.
// ---------------------------------------------------------------------------------------------
void control_resume(void)
{
    eint=0;
    last_count=_CP0_GET_COUNT();
    IFS0bits.T4IF=0;
    IEC0bits.T4IE=1;        // the loop runs again
}
