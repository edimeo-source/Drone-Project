/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/Drone.X/control.c
 *  What    : The 4 ms control loop (Timer4 interrupt): IMU -> filter -> roll PID + pitch PID ->
 *            4-motor mixer, plus arming, soft start and every safety check.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : Built in MPLAB on the laptop (Nov 25). v3.0 plus the O command. Host tests pass
 *            (87 checks). Replaced by v3.2 (same code, Bluefruit CS on pin 40).
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): slew(), graph_angle(),
 *    graph_setpoint(), ramp_done(), Controller(), initControl(), pid_reset(), motors_off(),
 *    control_arm(), control_heartbeat(), control_can_start(), control_pause(),
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

// The control loop. ONE timer interrupt every 4 ms does everything, in the
// order on the professor's board:
//
//   imu()  ->  calc()  ->  e = a_sp - a_m  ->  PWM = Kp*e + Ki*int(e) + Kd*de/dt  ->  data[i] = ...
//
// main() only talks to the PC and the phone, so printing can never slow this loop down.
// Timer4: 60 MHz / 64 = 937,500 counts per second
// period = (PR4+1) x 64 / 60 MHz = 3750 x 64 / 60 MHz = 4 ms (250 times a second)
//
// v3.0 - TWO PID CONTROLLERS (professor's notes, Nov 25):
//   PID 1, ROLL : e_roll  = target roll  - current roll   -> roll compensation  RC
//   PID 2, PITCH: e_pitch = target pitch - current pitch  -> pitch compensation PC
//   Each motor = desired throttle T + its pitch share + its roll share:
//
//                 FRONT
//     ESC 1 FL = T + PC + RC     ESC 2 FR = T + PC - RC
//     ESC 3 RL = T - PC + RC     ESC 4 RR = T - PC - RC
//                 BACK
//
//   Pitch (nose up/down): front pair vs back pair, + PC on the front, - PC on the back.
//   Roll (left/right):    left pair vs right pair.
//   The signs come from the IMU (imu.h): roll + = LEFT side up, pitch + = nose up.
//     left side low -> roll < 0 -> e_roll > 0 -> RC > 0 -> left motors faster -> left side comes up
//     nose low      -> pitch < 0 -> e_pitch > 0 -> PC > 0 -> front motors faster -> nose comes up
//   (The professor drew the roll signs the other way round; that only depends on which way
//    "roll +" points. With this IMU, + RC must go to the LEFT motors or the drone flips.)
//
//   ONE AXIS AT A TIME, like the seesaw:
//     axis_mode 1 = ROLL only:  PC = 0, so FL = RL = T + RC and FR = RR = T - RC
//     axis_mode 2 = PITCH only: RC = 0, so FL = FR = T + PC and RL = RR = T - PC
//     axis_mode 3 = BOTH:       both controllers added together: all 4 pulses different
//
// Motor modes (motor_mode):
//   0 = OFF     all ESCs at 1000 us. This is how it starts after power-up or reset.
//   1 = PID     tuner: "PID on" (G) or a step test (R)
//   2 = MANUAL  tuner: M,t  /  M,front,back  /  M,fl,fr,rl,rr   (PID off) - for finding liftoff
// Safety, checked right here every 4 ms (so it works even while main() is busy):
//   SW1 pressed, IMU stopped answering, or pitch/roll past TIP_LIMIT -> OFF.
// Soft start: the throttle T (and each manual pulse) climbs from 1000 us at ramp_rate
// (100 us per second unless changed with U,rate), so 1000 -> 1600 us takes 6 s.
// The integrals stay at 0 until the throttle has reached T.

#define EINTMAX   20.0f     // integrator clamp, degree-seconds (same as the tuner)
#define UMAX      500.0f    // biggest correction from one controller, us (same as the tuner)
#define DT_S      0.004f    // loop time, seconds

volatile float Kp_roll=1.0f, Ki_roll=0.0f, Kd_roll=0.0f;     // start with P only (professor: Kp first)
volatile float Kp_pitch=1.0f, Ki_pitch=0.0f, Kd_pitch=0.0f;
volatile float sp_roll=0.0f, sp_pitch=0.0f;
volatile int axis_mode=2;                   // PITCH only at power-up (what v2.0 did)
volatile int motor_mode=0;                  // OFF at power-up: nothing spins until you ask
volatile int kill_latched=1;                // 1 = disarmed. ARM is needed after power-up and after any stop
volatile int control_ready=0;
volatile int control_link_source=0;         // who armed: 0 = nobody, 1 = phone, 2 = PC

// HEARTBEAT FAILSAFE: while armed, the phone (or the PC tuner) sends "H" every 100 ms.
// If no H arrives for 500 ms, this interrupt turns the motors off and opens the relay.
// So a dead app, a dropped Bluetooth link or a crashed main() all stop the motors.
// The core timer counts 60,000 times per ms (120 MHz / 2).
static volatile unsigned int heartbeat_count=0;     // core timer at the last H
static unsigned int relay_powered_count=0;          // core timer when the relay closed
#define HEARTBEAT_COUNTS     (500u*60000u)  // 500 ms: 5 missed H in a row (one Bluetooth hiccup is fine)
#define RELAY_STARTUP_COUNTS (2000u*60000u) // 2 s at 1000 us after the relay closes: the ESCs beep and arm

volatile float throttle=1500.0f;            // desired throttle T: set it just above liftoff (B,T)
volatile float ramp_rate=100.0f;            // us per second (U,rate)
volatile float man_us[5]={PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF};
volatile float roll_offset=0.0f, pitch_offset=0.0f;
volatile float eint_roll=0, eint_pitch=0;   // Lynch: the integrator states live with the loop

volatile float ang_roll=0, ang_pitch=0;
volatile float roll_comp=0, pitch_comp=0;
volatile float p_term=0, i_term=0, d_term=0;
volatile float thr_now=PULSE_OFF;
volatile float motor_us[5]={PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF};
float man_now[5]={PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF};   // manual pulses after the soft start
volatile int off_reason=0;
volatile unsigned long loop_count=0;
volatile float loop_period_us=4000.0f, loop_busy_us=0;

unsigned int last_count=0;      // core timer at the start of the last loop

// ---------------------------------------------------------------------------------------------
// float slew(float now, float target)
//   Moves a pulse toward its target by at most ramp_rate x 4 ms per loop, up or down (the soft
//   start, also used in manual mode).
// ---------------------------------------------------------------------------------------------
// Move 'now' toward 'target' by at most ramp_rate x 4 ms (both directions).
float slew(float now, float target)
{
    float step=ramp_rate*DT_S;
    if (now<target){
        now=now+step;
        if (now>target) now=target;
    }
    else if (now>target){
        now=now-step;
        if (now<target) now=target;
    }
    return now;
}

// ---------------------------------------------------------------------------------------------
// float graph_angle(void)
//   The angle the tuner graphs: roll in Roll-only mode, pitch otherwise.
// ---------------------------------------------------------------------------------------------
float graph_angle(void)     { return (axis_mode==1) ? ang_roll : ang_pitch; }
// ---------------------------------------------------------------------------------------------
// float graph_setpoint(void)
//   The target the tuner graphs: the roll target in Roll-only mode, the pitch target otherwise.
// ---------------------------------------------------------------------------------------------
float graph_setpoint(void)  { return (axis_mode==1) ? sp_roll : sp_pitch; }
// ---------------------------------------------------------------------------------------------
// int ramp_done(void)
//   1 once the soft start has brought the throttle all the way up to T.
// ---------------------------------------------------------------------------------------------
int ramp_done(void)         { return thr_now==throttle; }

// ---------------------------------------------------------------------------------------------
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
//   Timer4 interrupt, every 4 ms: IMU -> filter -> roll PID + pitch PID -> 4-motor mixer (each
//   ESC its own pulse), plus every safety check.
// ---------------------------------------------------------------------------------------------
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller(void)
{
    unsigned int now;
    float e, edot, m[5];
    int n, flying, int_roll, int_pitch;

    LATHbits.LATH2=1;                   // LED3 on: scope this pin to see the loop (4 ms apart)

    // The core timer counts at 60 MHz (60 counts per microsecond), so the
    // difference between two loops is the real loop period. Proof of 4 ms.
    now=_CP0_GET_COUNT();
    loop_period_us=(now-last_count)/60.0f;
    last_count=now;

    // heartbeat check: only compares two times (no SPI, no Bluetooth in the interrupt)
    if (!kill_latched && now-heartbeat_count>HEARTBEAT_COUNTS){
        motors_off();
        off_reason=OFF_BLE;
    }
    if (kill_latched) motor_mode=0;

    // 1. imu() + calc(): read the sensor, complementary filter -> roll, pitch
    imu_update();
    ang_roll=roll-roll_offset;          // Z zeroes both
    ang_pitch=pitch-pitch_offset;

    // 2. safety: any of these turns the motors off
    if ((motor_mode!=0 || !kill_latched) && PORTBbits.RB12==0){  // SW1 also kills armed-idle permission
        motors_off();
        off_reason=OFF_SW1;
    }
    if ((motor_mode!=0 || !kill_latched) && imu_error==1){       // IMU stopped answering
        control_ready=0;
        motors_off();
        off_reason=OFF_IMU;
    }
    if ((motor_mode!=0 || !kill_latched) && (pitch>TIP_LIMIT || pitch<-TIP_LIMIT ||
                          roll>TIP_LIMIT || roll<-TIP_LIMIT)){  // tipped over
        motors_off();
        off_reason=OFF_TIP;
    }

    // 3. soft start: the throttle climbs (or falls) toward T at ramp_rate
    if (motor_mode==1) thr_now=slew(thr_now, throttle);
    else thr_now=PULSE_OFF;

    // 4. a step test from the tuner may change a setpoint
    tuner_setpoint_step();

    // 5. the two PIDs, Lynch form: e = reference - sensed.
    // de/dt: the setpoint holds still, so de/dt = -(rate the drone is turning).
    // Using the gyro's rate directly is cleaner than subtracting two noisy angles,
    // and a setpoint step can't cause a spike in the D term ("derivative kick").
    // Integrate only while really flying that axis (soft start done, axis switched on):
    // otherwise the I gain is 0 for this loop and the integral is cleared.
    flying=(motor_mode==1 && ramp_done());
    int_roll=(flying && axis_mode!=2);
    int_pitch=(flying && axis_mode!=1);

    e=sp_roll-ang_roll;                             // PID 1: ROLL
    edot=0-roll_rate;
    roll_comp=pid(e, edot, Kp_roll, int_roll ? Ki_roll : 0, Kd_roll, (float *)&eint_roll, EINTMAX, UMAX);
    if (!int_roll) eint_roll=0;
    if (axis_mode==1){                              // graph the roll terms
        p_term=Kp_roll*e;
        i_term=Ki_roll*eint_roll;
        d_term=Kd_roll*edot;
    }

    e=sp_pitch-ang_pitch;                           // PID 2: PITCH
    edot=0-pitch_rate;
    pitch_comp=pid(e, edot, Kp_pitch, int_pitch ? Ki_pitch : 0, Kd_pitch, (float *)&eint_pitch, EINTMAX, UMAX);
    if (!int_pitch) eint_pitch=0;
    if (axis_mode!=1){                              // graph the pitch terms
        p_term=Kp_pitch*e;
        i_term=Ki_pitch*eint_pitch;
        d_term=Kd_pitch*edot;
    }

    if (axis_mode==1) pitch_comp=0;                 // one axis at a time: the other gets 0
    if (axis_mode==2) roll_comp=0;

    // 6. PWM: the mixer, then 1-2 ms
    if (motor_mode==1){                             // PID
        m[1]=thr_now+pitch_comp+roll_comp;          // ESC 1 front-left
        m[2]=thr_now+pitch_comp-roll_comp;          // ESC 2 front-right
        m[3]=thr_now-pitch_comp+roll_comp;          // ESC 3 rear-left
        m[4]=thr_now-pitch_comp-roll_comp;          // ESC 4 rear-right
        for (n=1;n<=4;n++)
            if (m[n]<PULSE_IDLE) m[n]=PULSE_IDLE;   // keep all props turning
    }
    else if (motor_mode==2){                        // manual: each pulse climbs at ramp_rate
        for (n=1;n<=4;n++){
            man_now[n]=slew(man_now[n], man_us[n]);
            m[n]=man_now[n];
        }
    }
    else{                                           // off
        for (n=1;n<=4;n++){
            man_now[n]=PULSE_OFF;
            m[n]=PULSE_OFF;
        }
    }
    for (n=1;n<=4;n++){
        if (m[n]>PULSE_MAX) m[n]=PULSE_MAX;         // never more than 2 ms
        motor_us[n]=m[n];
        set_motor(n, m[n]);                         // ESC 1-4 (motors.c adds the per-motor trims)
    }
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
    pid_reset();
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
//   Clears both PIDs' integrals so an old error does not carry into the next run.
// ---------------------------------------------------------------------------------------------
void pid_reset(void)
{
    eint_roll=0;            // old error must not carry into the next run (no "windup")
    eint_pitch=0;
}

// ---------------------------------------------------------------------------------------------
// void motors_off(void)
//   KILL: every ESC to 1000 us, latch the kill, and (with a relay) open the motor-power relay.
// ---------------------------------------------------------------------------------------------
void motors_off(void)
{
    int was_enabled=IEC0bits.T4IE, n;
    kill_latched=1;         // set_motor() checks this too, so nothing can turn a motor back on
    LATKbits.LATK4=0;       // RK4 (J12 pin 33) LOW = relay off = motor power CUT, on every stop
    control_link_source=0;
    IEC0bits.T4IE=0;
    motor_mode=0;
    thr_now=PULSE_OFF;
    for (n=1;n<=4;n++){
        man_now[n]=PULSE_OFF;
        motor_us[n]=PULSE_OFF;
        set_motor(n, PULSE_OFF);    // right now, not at the next loop
    }
    pid_reset();
    LATHbits.LATH0=0;
    if (run_state!=0){
        run_state=0;
        run_stopped=1;
    }
    IEC0bits.T4IE=was_enabled;
}

// ---------------------------------------------------------------------------------------------
// int control_arm(int source)
//   ARM: safety checks, 1000 us to every ESC, then the relay closes so the ESCs power up seeing
//   'stop'.
// ---------------------------------------------------------------------------------------------
// ARM: checks, then 1000 us to all ESCs, THEN the relay closes (ESCs power up seeing "stop").
// Same relay logic as your tested main_bluefruit_relay_test.c: RK4 HIGH = motor power on.
int control_arm(int source)
{
    int ok=0, was_enabled=IEC0bits.T4IE, n;
    unsigned int now;

    IEC0bits.T4IE=0;                    // the 4 ms loop can't run in the middle of this
    now=_CP0_GET_COUNT();
    if (kill_latched && control_ready && !imu_error && !noise_on && PORTBbits.RB12 && motor_mode==0 &&
        pitch<=TIP_LIMIT && pitch>=-TIP_LIMIT && roll<=TIP_LIMIT && roll>=-TIP_LIMIT){
        for (n=1;n<=4;n++){
            set_motor(n, PULSE_OFF);    // 1000 us before power reaches the ESCs
            motor_us[n]=PULSE_OFF;
            man_now[n]=PULSE_OFF;
        }
        thr_now=PULSE_OFF;
        pid_reset();
        LATKbits.LATK4=1;               // RK4 HIGH = relay on = motor power ON
        relay_powered_count=now;
        heartbeat_count=now;            // the heartbeat clock starts now
        control_link_source=source;
        kill_latched=0;                 // armed: motors may run once a start command comes
        ok=1;
    }
    IEC0bits.T4IE=was_enabled;
    return ok;
}

// ---------------------------------------------------------------------------------------------
// void control_heartbeat(int source)
//   Records a heartbeat from the link that armed the drone (a stale heartbeat stops the
//   motors).
// ---------------------------------------------------------------------------------------------
void control_heartbeat(int source)
{
    if (source==control_link_source) heartbeat_count=_CP0_GET_COUNT();  // only the link that armed counts
}

// ---------------------------------------------------------------------------------------------
// int control_can_start(void)
//   1 if a run may start now: armed, IMU OK, ESC start-up time over, heartbeat fresh, SW1 not
//   pressed.
// ---------------------------------------------------------------------------------------------
int control_can_start(void)
{
    unsigned int now=_CP0_GET_COUNT();

    if (kill_latched || !control_ready || imu_error || noise_on || !PORTBbits.RB12) return 0;
    if (now-relay_powered_count<RELAY_STARTUP_COUNTS) return 0;    // ESCs still starting up
    if (now-heartbeat_count>HEARTBEAT_COUNTS) return 0;            // heartbeat stale
    return 1;
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
//   Recalibrates the IMU with the motors off (PuTTY key c or SW2), clears the roll and pitch
//   zero offsets, then resumes the loop.
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
    roll_offset=0;                      // calibration makes "level" 0 again
    pitch_offset=0;
    control_resume();
}
