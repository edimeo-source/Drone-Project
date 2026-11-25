/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v1.2_tip_cutoff_and_recorded_runs/Drone.X/control.c
 *  What    : The 4 ms control loop (Timer4 interrupt): IMU -> filter -> PID -> motors, plus
 *            arming and every safety check.
 *  Version : v1.2   |   Date: 2026-11-22
 *  Status  : Host tests pass. Not built with XC32.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): Controller(), initControl(),
 *    pid_reset(), motors_off(), control_arm_common(), control_arm_ble(), control_arm_usb(),
 *    control_heartbeat_ble(), control_heartbeat_usb(), control_can_start(), control_pause(),
 *    control_resume(), calibrate_now()
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
# include "uart.h"
# include "bluefruit.h"

// The control loop. ONE timer interrupt every 4 ms does everything, in the
// order on the professor's board:
//
//   imu()  ->  calc()  ->  e = a_sp - a_m  ->  PWM = Kp*e + Ki*int(e) + Kd*de/dt  ->  data[i] = ...
//
// main() only talks to the PC, so printing can never slow this loop down.
// Timer4: 60 MHz / 64 = 937,500 counts per second
// period = (PR4+1) x 64 / 60 MHz = 3750 x 64 / 60 MHz = 4 ms (250 times a second)
//
// DRONE, first stage (professor's plan): only PITCH is controlled.
// The front pair and the back pair each get ONE pulse, like the seesaw with each motor doubled:
//   front pair (ESC 1 + ESC 2) = base_front + u
//   back pair  (ESC 3 + ESC 4) = base_back  - u
//   nose up -> angle + -> e < 0 -> u < 0 -> front slows, back speeds up -> nose comes down
// Roll is NOT controlled yet (the tethers hold it). It is still watched for the tip limit.
//
// Motor modes (motor_mode):
//   0 = OFF     all ESCs at 1000 us. This is how it starts after power-up or reset.
//   1 = PID     tuner: "PID on" button (G) or a step test (R)
//   2 = MANUAL  tuner: "Run manual" button (M,front,back), PID off
// Safety, checked right here every 4 ms (so it works even while main() is busy):
//   SW1 pressed, IMU stopped answering, or pitch/roll past TIP_LIMIT -> OFF.
//   Soft start: after OFF, the pulses can only climb 1000 -> 2000 us over 1 s.

#define EINTMAX   20.0f     // integrator clamp, degree-seconds (same as the tuner)
#define UMAX      500.0f    // biggest correction, us (same as the tuner)
#define TIP_LIMIT 75.0f     // degrees, + and -: past this the motors go off (ARM also needs to be inside it)
#define RAMP_STEP 4.0f      // soft start: +4 us every 4 ms = 1000 us per second

volatile float Kp=1.0f, Ki=0.0f, Kd=0.0f;  // start with P only (professor: Kp first, then Ki)
volatile float setpoint=0.0f;
volatile int motor_mode=0;                  // OFF at power-up: nothing spins until you ask
volatile int kill_latched=1;                // phone ARM; required after startup or any stop
volatile int control_ready=0;
volatile int control_link_source=0; /* 0=none, 1=BLE, 2=USB serial */
static volatile unsigned int ble_heartbeat_count=0;
static volatile unsigned int usb_heartbeat_count=0;
static unsigned int relay_powered_count=0;
#define LINK_TIMEOUT_COUNTS (350u*60000u) /* CP0 = 60 MHz */
#define RELAY_STARTUP_COUNTS (2000u*60000u) /* allow ESCs to initialize at 1000 us */
volatile float base_front=1500.0f, base_back=1500.0f;
volatile float man_front=PULSE_OFF, man_back=PULSE_OFF;
volatile float angle_offset=0.0f;
volatile float eint=0;                      // Lynch: the integrator state lives with the loop

volatile float angle=0;
volatile float u=0, p_term=0, i_term=0, d_term=0;
volatile float motor_front=PULSE_OFF, motor_back=PULSE_OFF;
volatile float ramp_us=PULSE_OFF;
volatile int off_reason=0;
volatile unsigned long loop_count=0;
volatile float loop_period_us=4000.0f, loop_busy_us=0;

unsigned int last_count=0;      // core timer at the start of the last loop

// ---------------------------------------------------------------------------------------------
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
//   Timer4 interrupt, every 4 ms: read IMU -> angle filter -> PID -> motor pulses, plus all
//   safety checks.
// ---------------------------------------------------------------------------------------------
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
{
    unsigned int now;
    float e, edot, f, b;

    LATHbits.LATH2=1;                   // LED3 on: scope this pin to see the loop (4 ms apart)

    // The core timer counts at 60 MHz (60 counts per microsecond), so the
    // difference between two loops is the real loop period. Proof of 4 ms.
    now=_CP0_GET_COUNT();
    loop_period_us=(now-last_count)/60.0f;
    last_count=now;

    /* No SPI/BLE work here. This watchdog only compares timestamps refreshed by main(). */
    if (!kill_latched){
        unsigned int link_age=0;
        if (control_link_source==1) link_age=now-ble_heartbeat_count;
        else if (control_link_source==2) link_age=now-usb_heartbeat_count;
        else link_age=LINK_TIMEOUT_COUNTS+1u;
        if (link_age>LINK_TIMEOUT_COUNTS ||
            (control_link_source==1 && !bluetooth_link_ok())){
            motors_off();
            off_reason=OFF_BLE;
        }
    }
    if (kill_latched) motor_mode=0;

    // 1. imu() + calc(): read the sensor, complementary filter -> roll, pitch
    imu_update();
    angle=pitch-angle_offset;           // the drone's pitch (Z zeroes it)

    // 2. safety: any of these turns the motors off
    if ((motor_mode!=0 || !kill_latched) && PORTBbits.RB12==0){  // SW1 also kills armed-idle permission
        motors_off();
        off_reason=OFF_SW1;
    }
    if ((motor_mode!=0 || !kill_latched) && imu_error==1){                         // IMU stopped answering
        control_ready=0;
        motors_off();
        off_reason=OFF_IMU;
    }
    if ((motor_mode!=0 || !kill_latched) && (angle>TIP_LIMIT || angle<-TIP_LIMIT ||
                          roll>TIP_LIMIT || roll<-TIP_LIMIT)){  // tipped over
        motors_off();
        off_reason=OFF_TIP;
    }

    // 3. soft start: the highest pulse allowed climbs 1000 -> 2000 us in 1 s
    if (motor_mode==0) ramp_us=PULSE_OFF;
    else if (ramp_us<PULSE_MAX) ramp_us=ramp_us+RAMP_STEP;

    // 4. a step test from the tuner may change the setpoint
    tuner_setpoint_step();

    // 5. error and PID, Lynch form: e = reference - sensed  (board: e = a_sp - a_m)
    e=setpoint-angle;
    // de/dt: the setpoint holds still, so de/dt = -(rate the drone is turning).
    // Using the gyro's rate directly is cleaner than subtracting two noisy angles,
    // and a setpoint step can't cause a spike in the D term ("derivative kick").
    edot=0-pitch_rate;
    u=pid(e, edot, Kp, Ki, Kd, (float *)&eint, EINTMAX, UMAX);
    if (motor_mode!=1 || ramp_us<PULSE_MAX) eint=0;     // only integrate while the PID really flies
    p_term=Kp*e;                        // the three parts of u, for the tuner's graph
    i_term=Ki*eint;
    d_term=Kd*edot;

    // 6. PWM: mixer, then clamp to 1-2 ms
    if (motor_mode==1){                 // PID
        f=base_front+u;
        b=base_back-u;
        if (f<PULSE_IDLE) f=PULSE_IDLE; // keep all props turning
        if (b<PULSE_IDLE) b=PULSE_IDLE;
    }
    else if (motor_mode==2){            // manual
        f=man_front;
        b=man_back;
    }
    else{                               // off
        f=PULSE_OFF;
        b=PULSE_OFF;
    }
    if (f>ramp_us) f=ramp_us;           // soft start
    if (b>ramp_us) b=ramp_us;
    if (f>PULSE_MAX) f=PULSE_MAX;       // never more than 2 ms
    if (b>PULSE_MAX) b=PULSE_MAX;
    motor_front=f;
    motor_back=b;
    set_motor(1, motor_front);          // ESC 1, front-left
    set_motor(2, motor_front);          // ESC 2, front-right
    set_motor(3, motor_back);           // ESC 3, rear-left
    set_motor(4, motor_back);           // ESC 4, rear-right
    LATHbits.LATH0=(motor_mode!=0);     // LED1 = motors on

    // 7. data[i] = ...: save this sample if a step test is recording (sent after the run)
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
// void pid_reset(void)
//   Clears the PID's integral so an old error does not carry into the next run.
// ---------------------------------------------------------------------------------------------
void pid_reset(void)
{
    eint=0;                 // old error must not carry into the next run (no "windup")
}

// ---------------------------------------------------------------------------------------------
// void motors_off(void)
//   KILL: every ESC to 1000 us, latch the kill, and (with a relay) open the motor-power relay.
// ---------------------------------------------------------------------------------------------
void motors_off(void)
{
    int was_enabled=IEC0bits.T4IE;
    kill_latched=1;         // driver checks this too, so a preemption cannot restore throttle
    ble_arm_ready=0;        // drain any already-queued ARM text before accepting a fresh rearm
    LATKbits.LATK4=0;       // open motor-power relay immediately on every stop path
    control_link_source=0;
    IEC0bits.T4IE=0;
    motor_mode=0;
    ramp_us=PULSE_OFF;
    motor_front=PULSE_OFF;
    motor_back=PULSE_OFF;
    set_motor(1, PULSE_OFF);    // right now, not at the next loop
    set_motor(2, PULSE_OFF);
    set_motor(3, PULSE_OFF);
    set_motor(4, PULSE_OFF);
    pid_reset();
    LATHbits.LATH0=0;
    if (run_state!=0){
        run_state=0;
        run_stopped=1;
    }
    IEC0bits.T4IE=was_enabled;
}

// ---------------------------------------------------------------------------------------------
// static int control_arm_common(int source)
//   Shared ARM steps for the phone (source 1) and the PC (source 2).
// ---------------------------------------------------------------------------------------------
static int control_arm_common(int source)
{
    int ok=0, was_enabled=IEC0bits.T4IE;
    unsigned int now;
    IEC0bits.T4IE=0;
    now=_CP0_GET_COUNT();
    if (source==1 && (!bluetooth_link_ok() || !ble_arm_ready)) goto done;
    if (kill_latched && control_ready && !imu_error && !noise_on && PORTBbits.RB12 && motor_mode==0 &&
        angle<=TIP_LIMIT && angle>=-TIP_LIMIT && roll<=TIP_LIMIT && roll>=-TIP_LIMIT){
        /* Ensure minimum ESC pulses are already programmed before closing relay. */
        set_motor(1,PULSE_OFF); set_motor(2,PULSE_OFF);
        set_motor(3,PULSE_OFF); set_motor(4,PULSE_OFF);
        motor_front=PULSE_OFF; motor_back=PULSE_OFF; ramp_us=PULSE_OFF;
        pid_reset();
        LATKbits.LATK4=1;             /* active-HIGH relay: ESC bus now powered */
        relay_powered_count=now;
        control_link_source=source;
        if (source==1) ble_heartbeat_count=now;
        else usb_heartbeat_count=now;
        kill_latched=0;               /* permission only; does not command throttle */
        ok=1;
    }
done:
    IEC0bits.T4IE=was_enabled;
    return ok;
}

// ---------------------------------------------------------------------------------------------
// int control_arm_ble(void)
//   ARM request that came from the phone over Bluetooth.
// ---------------------------------------------------------------------------------------------
int control_arm_ble(void) { return control_arm_common(1); }
// ---------------------------------------------------------------------------------------------
// int control_arm_usb(void)
//   ARM request that came from the PC over USB.
// ---------------------------------------------------------------------------------------------
int control_arm_usb(void) { return control_arm_common(2); }

// ---------------------------------------------------------------------------------------------
// void control_heartbeat_ble(void)
//   Records a heartbeat from the phone.
// ---------------------------------------------------------------------------------------------
void control_heartbeat_ble(void) { ble_heartbeat_count=_CP0_GET_COUNT(); }
// ---------------------------------------------------------------------------------------------
// void control_heartbeat_usb(void)
//   Records a heartbeat from the PC.
// ---------------------------------------------------------------------------------------------
void control_heartbeat_usb(void) { usb_heartbeat_count=_CP0_GET_COUNT(); }

// ---------------------------------------------------------------------------------------------
// int control_can_start(void)
//   1 if a run may start now: armed, IMU OK, ESC start-up time over, heartbeat fresh, SW1 not
//   pressed.
// ---------------------------------------------------------------------------------------------
int control_can_start(void)
{
    unsigned int now=_CP0_GET_COUNT();
    // motor_mode may already be 1 or 2: the tuner sends M again every time the manual
    // sliders move, and a step test may start while the PID is on.
    if (kill_latched || !control_ready || imu_error || noise_on || !PORTBbits.RB12 ||
        control_link_source==0) return 0;
    if ((unsigned int)(now-relay_powered_count)<RELAY_STARTUP_COUNTS) return 0;
    if (control_link_source==1)
        return bluetooth_link_ok() && (unsigned int)(now-ble_heartbeat_count)<LINK_TIMEOUT_COUNTS;
    if (control_link_source==2)
        return (unsigned int)(now-usb_heartbeat_count)<LINK_TIMEOUT_COUNTS;
    return 0;
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
    pid_reset();
    last_count=_CP0_GET_COUNT();
    IFS0bits.T4IF=0;
    IEC0bits.T4IE=1;        // the loop runs again
}

// ---------------------------------------------------------------------------------------------
// void calibrate_now(void)
//   Recalibrates the IMU with the motors off (PuTTY key c or SW2), then resumes the loop.
// ---------------------------------------------------------------------------------------------
void calibrate_now(void)
{
    control_ready=0;
    control_pause();                    // motors off, and stop the 4 ms loop while we read the IMU here
    putsU2(" Calibrating (motors off): keep the drone still and LEVEL for 2 s...");
    if (imu_calibrate()==1 && !imu_error){
        imu_reset_angles();
        control_ready=!imu_error;
    }
    LATHbits.LATH1=control_ready;        // LED2 = calibration succeeded
    angle_offset=0;                     // calibration makes "level" 0 again
    control_resume();
}
