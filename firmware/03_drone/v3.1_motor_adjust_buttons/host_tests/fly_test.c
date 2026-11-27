/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/host_tests/fly_test.c
 *  What    : Host test: flies a simulated drone (physics model in place of the IMU) with the
 *            real 4 ms loop: levels each axis, holds the demo angles, a wrong sign ends at the
 *            tip cutoff.
 *  Version : v3.1   |   Date: 2026-11-25
 *  Status  : Built in MPLAB on the laptop (Nov 25). v3.0 plus the O command. Host tests pass
 *            (87 checks). Replaced by v3.2 (same code, Bluefruit CS on pin 40).
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): putsU2(), initSPI(),
 *    imu_who_am_i(), initIMU(), imu_calibrate(), imu_reset_angles(), thrust(), imu_update(),
 *    tick(), run_for(), reset_state(), start(), main()
 * ============================================================================================ */

/* v3.0 host test: FLY a simulated drone with the real 4 ms loop (control.c, pid.c, tuner.c, motors.c).
   This replaces imu.c with a small physics model, so it checks that every sign in the two PIDs
   and the mixer really brings the drone back to level - not just that the numbers look right.

   Model (F450, X frame, rigid body turning about its center):
     thrust of one motor   T(p) = 8 N x ((p - 1000) / 1000)^2        (about 800 g at full throttle)
     arm to each axis      d = 0.225 m x sin 45 = 0.159 m
     roll torque  (+ = LEFT side up) = d x [(T_FL + T_RL) - (T_FR + T_RR)]
     pitch torque (+ = NOSE up)      = d x [(T_FL + T_FR) - (T_RL + T_RR)]
     inertia I = 0.0075 kg m^2 about both axes
   The IMU gives the loop the true angles and rates with the same signs as imu.h. */
#include "p32xxxx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "control.h"
#include "motors.h"
#include "imu.h"
#include "tuner.h"
void Controller(void);
// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Test stand-in for the UART: throws the text away.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s){ (void)s; }
extern volatile int data_complete; extern volatile int run_total;
extern float data_roll[], data_pitch[];

/* ---- stand-in IMU = the physics model ---- */
volatile float roll=0, pitch=0, yaw=0, roll_rate=0, pitch_rate=0, yaw_rate=0, ax=0, ay=0, az=1, temp_c=25;
volatile int imu_error=0;
int check_count=0;
// ---------------------------------------------------------------------------------------------
// void initSPI(void)
//   Simulated drone: no SPI (the physics model replaces the IMU).
// ---------------------------------------------------------------------------------------------
void initSPI(void){}
// ---------------------------------------------------------------------------------------------
// int imu_who_am_i(void)
//   Simulated drone: answers 0x6C like a real LSM6DSO.
// ---------------------------------------------------------------------------------------------
int imu_who_am_i(void){ return 0x6C; }
// ---------------------------------------------------------------------------------------------
// int initIMU(void)
//   Simulated drone: nothing to set up (0 = every setting OK).
// ---------------------------------------------------------------------------------------------
int initIMU(void){ return 0; }
// ---------------------------------------------------------------------------------------------
// int imu_calibrate(void)
//   Simulated drone: nothing to calibrate.
// ---------------------------------------------------------------------------------------------
int imu_calibrate(void){ return 1; }
// ---------------------------------------------------------------------------------------------
// void imu_reset_angles(void)
//   Simulated drone: nothing to do (the model sets the angles).
// ---------------------------------------------------------------------------------------------
void imu_reset_angles(void){}
static float strength[5]={0,1,1,1,1};   /* 1 = normal motor; 0.94 = 6% weak */
static float roll_sign=1;               /* -1 = roll torque the wrong way (IMU mounted flipped) */
static float cg_offset=0;               /* m: battery off-center to the right -> thrust x offset tips it */
// ---------------------------------------------------------------------------------------------
// static float thrust(int n)
//   Simulated drone: thrust of motor n from its pulse, strength x 8 N x ((p - 1000) / 1000)^2.
// ---------------------------------------------------------------------------------------------
static float thrust(int n){ float p=motor_us[n]; if (p<=1000) return 0; float x=(p-1000)/1000.0f; return strength[n]*8.0f*x*x; }
// ---------------------------------------------------------------------------------------------
// void imu_update(void)
//   Simulated drone, every 4 ms: the 4 thrusts make roll and pitch torques (arm 0.159 m, I =
//   0.0075 kg m^2), integrated into the rates and angles the PIDs read.
// ---------------------------------------------------------------------------------------------
void imu_update(void){
    const float d=0.159f, I=0.0075f, dt=0.004f, R2D=57.29578f;
    float total=thrust(1)+thrust(2)+thrust(3)+thrust(4);
    float tr=roll_sign*d*((thrust(1)+thrust(3))-(thrust(2)+thrust(4)))+total*cg_offset;
    float tp=d*((thrust(1)+thrust(2))-(thrust(3)+thrust(4)));
    roll_rate+=tr/I*R2D*dt;  pitch_rate+=tp/I*R2D*dt;
    roll_rate*=0.998f; pitch_rate*=0.998f;          /* a little air damping */
    if (total==0){ roll_rate*=0.9f; pitch_rate*=0.9f; }   /* motors off: the tether / table holds it */
    roll+=roll_rate*dt;      pitch+=pitch_rate*dt;
}

static int fails=0;
#define CHECK(c,msg) do{ if(c) printf("PASS  %s\n",msg); else {printf("FAIL  %s\n",msg); fails++;} }while(0)
// ---------------------------------------------------------------------------------------------
// static void tick(void)
//   Test helper: moves the simulated clock 4 ms forward and runs the control interrupt once.
// ---------------------------------------------------------------------------------------------
static void tick(void){ cp0+=4u*60000u; Controller(); }
// ---------------------------------------------------------------------------------------------
// static void run_for(int loops)
//   Test helper: runs the 4 ms control loop 'loops' times.
// ---------------------------------------------------------------------------------------------
static void run_for(int loops){ for(int i=0;i<loops;i++){ if(i%25==0) control_heartbeat(2); tick(); } }
// ---------------------------------------------------------------------------------------------
// static void reset_state(float r, float p)
//   Puts the simulated drone at a roll and pitch angle, at rest.
// ---------------------------------------------------------------------------------------------
static void reset_state(float r, float p){ roll=r; pitch=p; roll_rate=0; pitch_rate=0; }
// ---------------------------------------------------------------------------------------------
// static void start(int axis, float r0, float p0)
//   Test helper: KILL, ARM, wait for the ESCs, select the axis, targets 0, tilt the drone, then
//   G (PID hold).
// ---------------------------------------------------------------------------------------------
static void start(int axis, float r0, float p0){
    tuner_command("X"); off_reason=0;
    reset_state(0,0); tick(); control_heartbeat(2); control_arm(2); run_for(520);
    char a[8]; sprintf(a,"A,%d",axis); tuner_command(a);
    tuner_command("S,0,0"); tuner_command("SR,0"); tuner_command("SP,0");
    reset_state(r0,p0); tuner_command("G");
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Host test program: runs the firmware code on the PC with stand-in registers and prints
//   PASS/FAIL for each check.
// ---------------------------------------------------------------------------------------------
int main(void){
    SPI1STATbits.SPIRBF=1; PORTBbits.RB12=1; IEC0bits.T4IE=1; control_ready=1; cp0=1000;
    tuner_command("B,1600"); tuner_command("KR,1.0,0,0.25"); tuner_command("KP,1.0,0,0.25");

    /* 1. BOTH: tilted 10 deg roll and -8 deg pitch -> both come back to level */
    start(3, 10, -8); run_for(2500);                                   /* 10 s (6 s soft start + 4 s) */
    printf("      BOTH after 10 s: roll %.2f  pitch %.2f deg\n", roll, pitch);
    CHECK(kill_latched==0 && fabsf(roll)<0.5f && fabsf(pitch)<0.5f, "BOTH: roll +10 / pitch -8 deg -> level (both PIDs, all 4 motors)");

    /* 2. ROLL only: roll is leveled, pitch is left alone (the tether holds it) */
    start(1, 10, 5); run_for(2500);
    printf("      ROLL only after 10 s: roll %.2f  pitch %.2f deg\n", roll, pitch);
    CHECK(kill_latched==0 && fabsf(roll)<0.5f && fabsf(pitch-5)<0.2f, "ROLL only: roll -> level, pitch untouched (PC = 0)");

    /* 3. PITCH only: pitch is leveled, roll is left alone */
    start(2, 6, -10); run_for(2500);
    printf("      PITCH only after 10 s: roll %.2f  pitch %.2f deg\n", roll, pitch);
    CHECK(kill_latched==0 && fabsf(pitch)<0.5f && fabsf(roll-6)<0.2f, "PITCH only: pitch -> level, roll untouched (RC = 0)");

    /* 4. a 6% weak front-left motor + battery off-center: P+D leaves an error, the I term removes it */
    strength[1]=0.94f; cg_offset=0.004f;
    start(3, 0, 0); run_for(3000);
    float r_pd=roll, p_pd=pitch;
    tuner_command("KR,1.0,0.6,0.25"); tuner_command("KP,1.0,0.6,0.25");
    start(3, 0, 0); run_for(5000);                                     /* 20 s */
    printf("      weak FL + battery 4 mm off-center: P+D leaves roll %.2f pitch %.2f deg; with Ki: roll %.2f pitch %.2f deg\n", r_pd, p_pd, roll, pitch);
    CHECK(kill_latched==0 && fabsf(r_pd)>0.5f && fabsf(roll)<0.2f && fabsf(pitch)<0.2f, "Ki removes the steady tilt from an unequal motor");
    strength[1]=1; cg_offset=0;
    tuner_command("KR,1.0,0,0.25"); tuner_command("KP,1.0,0,0.25");

    /* 5. demo (professor): hold roll 5 deg, then pitch 10 deg, then both */
    start(3, 0, 0); run_for(2000);
    tuner_command("SR,5"); run_for(750);
    CHECK(fabsf(roll-5)<0.5f && fabsf(pitch)<0.5f, "demo: SR,5 -> roll 5 deg, pitch stays 0");
    tuner_command("SR,0"); tuner_command("SP,10"); run_for(750);
    CHECK(fabsf(pitch-10)<0.5f && fabsf(roll)<0.5f, "demo: SP,10 -> pitch 10 deg, roll back to 0");
    tuner_command("S,5,10"); run_for(750);
    printf("      demo S,5,10: roll %.2f  pitch %.2f deg\n", roll, pitch);
    CHECK(fabsf(roll-5)<0.5f && fabsf(pitch-10)<0.5f, "demo: S,5,10 -> roll 5 and pitch 10 together");

    /* 6. recorded step test on ROLL: the recording shows the roll going 0 -> 5 */
    start(1, 0, 0); tuner_command("X"); off_reason=0;
    reset_state(0,0); tick(); control_heartbeat(2); control_arm(2); run_for(520);
    tuner_command("A,1"); tuner_command("SR,0");
    tuner_command("R,0,5,3000");
    int loops=0; while(!data_complete && loops<5000){ run_for(1); loops++; }
    printf("      roll step test: %.2f deg at the step, %.2f deg 3 s later\n", data_roll[50], data_roll[run_total-1]);
    CHECK(data_complete && fabsf(data_roll[50])<0.3f && fabsf(data_roll[run_total-1]-5)<0.5f, "ROLL step test recorded: 0 -> 5 deg");
    while(data_complete) tuner_send_run();

    /* 7. why the roll sign matters: with the roll torque reversed (IMU flipped), it can't level -
          it tips past 60 deg and the tip cutoff stops the motors and opens the relay */
    roll_sign=-1; start(1, 3, 0); run_for(2500);
    CHECK(kill_latched==1 && off_reason==OFF_TIP && LATKbits.LATK4==0, "wrong roll sign: drone tips over -> tip cutoff, relay open (so check the signs props-off first)");

    printf(fails? "\n%d FAILED\n":"\nall passed\n", fails);
    return fails;
}
