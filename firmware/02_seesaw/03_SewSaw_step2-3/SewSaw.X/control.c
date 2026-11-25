/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/control.c
 *  What    : The 4 ms control loop (Timer4 interrupt): IMU -> filter -> PID -> motors, plus
 *            arming and every safety check.
 *  Version : SewSaw v2   |   Date: 2026-11-16
 *  Status  : Written and handed over; the seesaw stage was marked done Nov 18 (no separate
 *            hardware log).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC red 5 V wires . NOT connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): Controller(), initControl(),
 *    motors_off(), control_pause(), control_resume()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include "control.h"
# include "imu.h"
# include "pid.h"
# include "motors.h"
# include "tuner.h"
# include "noise.h"

// The control loop, Lynch style: ONE timer interrupt at a fixed rate does
// sense -> compute -> actuate. main() only talks to the PC.
// Timer4: 60 MHz / 64 = 937,500 counts per second
// period = (PR4+1) x 64 / 60 MHz = 3750 x 64 / 60 MHz = 4 ms (250 times a second)
//
// Seesaw: the beam tilts in ROLL (IMU X arrow along the pivot rod, text facing you).
//   motor A = left end  (ESC 1, pin 11) = base A + u
//   motor B = right end (ESC 2, pin 12) = base B - u
//   left end up -> angle + -> e < 0 -> u < 0 -> A slows, B speeds up -> left end comes down
//
// Motor modes (motor_mode):
//   0 = OFF     both ESCs at 1000 us. This is how it starts after power-up or reset.
//   1 = PID     tuner: "PID on" button (G) or a step test (R)
//   2 = MANUAL  tuner: "Run manual" button (M,a,b), PID off
// Safety, checked right here every 4 ms (so it works even while main() is busy):
//   SW1 pressed, IMU stopped answering, or angle past TIP_LIMIT -> OFF.
//   Soft start: after OFF, the pulses can only climb 1000 -> 2000 us over 1 s.

#define EINTMAX   20.0f     // integrator clamp, degree-seconds (same as the tuner)
#define UMAX      500.0f    // biggest correction, us (same as the tuner)
#define TIP_LIMIT 45.0f     // degrees: past this something is wrong, motors off
#define RAMP_STEP 4.0f      // soft start: +4 us every 4 ms = 1000 us per second

volatile float Kp=1.0f, Ki=0.0f, Kd=0.0f;  // start with P only (Lynch's order: P, then D, then I)
volatile float setpoint=0.0f;
volatile int motor_mode=0;                  // OFF at power-up: nothing spins until you ask
volatile float base_a=1500.0f, base_b=1500.0f;  // prof: 1.5 ms center
volatile float man_a=PULSE_OFF, man_b=PULSE_OFF;
volatile float angle_offset=0.0f;

volatile float angle=0;
volatile float u=0, p_term=0, i_term=0, d_term=0;
volatile float motor_a=PULSE_OFF, motor_b=PULSE_OFF;
volatile float ramp_us=PULSE_OFF;
volatile int off_reason=0;
volatile unsigned long loop_count=0;
volatile float loop_period_us=4000.0f, loop_busy_us=0;

float eint=0;                   // Lynch: the integrator state lives with the loop
unsigned int last_count=0;      // core timer at the start of the last loop

// ---------------------------------------------------------------------------------------------
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
//   Timer4 interrupt, every 4 ms: read IMU -> angle filter -> PID -> motor pulses, plus all
//   safety checks.
// ---------------------------------------------------------------------------------------------
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
{
    unsigned int now;
    float e, edot, a, b;

    LATHbits.LATH2=1;                   // LED3 on: scope this pin to see the loop (4 ms apart)

    now=_CP0_GET_COUNT();               // core timer: 60 counts per microsecond
    loop_period_us=(now-last_count)/60.0f;
    last_count=now;

    // 1. sense
    imu_update();
    angle=roll-angle_offset;            // the seesaw angle (Z zeroes it)

    // 2. safety: any of these turns the motors off
    if (motor_mode!=0 && PORTBbits.RB12==0){                    // SW1 = KILL
        motor_mode=0;
        off_reason=OFF_SW1;
    }
    if (motor_mode!=0 && imu_error==1){                         // IMU stopped answering
        motor_mode=0;
        off_reason=OFF_IMU;
    }
    if (motor_mode==1 && (angle>TIP_LIMIT || angle<-TIP_LIMIT)){    // tipped over
        motor_mode=0;
        off_reason=OFF_TIP;
    }

    // 3. soft start: the highest pulse allowed climbs 1000 -> 2000 us in 1 s
    if (motor_mode==0) ramp_us=PULSE_OFF;
    else if (ramp_us<PULSE_MAX) ramp_us=ramp_us+RAMP_STEP;

    // 4. a step test from the tuner may change the setpoint
    tuner_setpoint_step();

    // 5. PID, Lynch form. Reference = setpoint, sensed = the seesaw angle.
    e=setpoint-angle;                   // e = r - s
    edot=0-roll_rate;                   // edot = rdot - sdot (the setpoint holds still, so rdot = 0)
    u=pid(e, edot, Kp, Ki, Kd, &eint, EINTMAX, UMAX);
    if (motor_mode!=1 || ramp_us<PULSE_MAX) eint=0;     // only integrate while the PID really flies
    p_term=Kp*e;                        // the three parts of u, for the tuner's graph
    i_term=Ki*eint;
    d_term=Kd*edot;

    // 6. motors
    if (motor_mode==1){                 // PID: seesaw mixer
        a=base_a+u;
        b=base_b-u;
        if (a<PULSE_IDLE) a=PULSE_IDLE; // keep both props turning
        if (b<PULSE_IDLE) b=PULSE_IDLE;
    }
    else if (motor_mode==2){            // manual
        a=man_a;
        b=man_b;
    }
    else{                               // off
        a=PULSE_OFF;
        b=PULSE_OFF;
    }
    if (a>ramp_us) a=ramp_us;           // soft start
    if (b>ramp_us) b=ramp_us;
    if (a>PULSE_MAX) a=PULSE_MAX;
    if (b>PULSE_MAX) b=PULSE_MAX;
    motor_a=a;
    motor_b=b;
    set_motor(1, motor_a);              // ESC 1, pin 11
    set_motor(2, motor_b);              // ESC 2, pin 12
    LATHbits.LATH0=(motor_mode!=0);     // LED1 = motors on

    // 7. recordings (tuner step test, noise test)
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
// void motors_off(void)
//   KILL: every ESC to 1000 us, latch the kill, and (with a relay) open the motor-power relay.
// ---------------------------------------------------------------------------------------------
void motors_off(void)
{
    motor_mode=0;
    ramp_us=PULSE_OFF;
    motor_a=PULSE_OFF;
    motor_b=PULSE_OFF;
    set_motor(1, PULSE_OFF);    // right now, not at the next loop
    set_motor(2, PULSE_OFF);
    LATHbits.LATH0=0;
}

// ---------------------------------------------------------------------------------------------
// void control_pause(void)
//   Motors off and the 4 ms loop stopped (used while calibrating or printing reports).
// ---------------------------------------------------------------------------------------------
void control_pause(void)
{
    motors_off();           // the loop is about to stop, so the ESCs must not keep the last pulse
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
