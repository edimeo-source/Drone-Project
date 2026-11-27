/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.1_motor_adjust_buttons/Drone.X/tuner.c
 *  What    : PID-tuner protocol (PC or phone): axis, roll and pitch gains and targets,
 *            throttle, soft start, live +/- per motor, recorded step tests, live data.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): tuner_set_ble_output(),
 *    tuner_reply(), axis_name(), read_numbers(), clamp_us(), tuner_help(), reply_offsets(),
 *    reply_gains(), set_gains(), tuner_command(), plus_pair(), minus_pair(), tuner_send_live(),
 *    tuner_send_run(), round_i(), tuner_send_run_phone(), tuner_run_is_phone(),
 *    tuner_setpoint_step(), tuner_record()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include "tuner.h"
# include "control.h"
# include "motors.h"
# include "imu.h"
# include "uart.h"
# include "bluefruit.h"

// Commands from the PHONE APP (Bluetooth) or the PC TUNER (USB, pid_tuner_ble.py).
// Both use the same commands; each reply goes back to whichever one asked.
//
// v3.0: TWO PIDs (roll, pitch). Pick the axis first, tune it alone, then the other, then both.
// v3.1: O,fl,fr,rl,rr = live +/- per motor (the app's motor buttons), e.g. to stop a yaw spin.
// Tuner -> PIC (one line each):
//   A,1 / A,2 / A,3  axis: 1 = ROLL only, 2 = PITCH only, 3 = BOTH (motors must be off)
//   K,kp,ki,kd       gains of the selected axis (BOTH: both axes get them)  (PIC answers A,kp,ki,kd)
//   KR,kp,ki,kd      roll gains      KP,kp,ki,kd   pitch gains           (PIC answers AR / AP)
//   S,deg            setpoint of the selected axis (BOTH: S,roll,pitch, or one number for both)
//   SR,deg  SP,deg   roll setpoint / pitch setpoint, any time
//   R,from,to,ms     step test on the selected axis (BOTH: both axes step): PID on, soft start,
//                    hold "from" 1 s, step to "to", record ms
//   G                PID on, hold the setpoints (no step test)
//   B,T              desired throttle T for all 4 motors, us   (B,f,b from an old app: T = average)
//   M,t              manual, PID off: all 4 motors at t us     - for finding liftoff (professor's step 1)
//   M,f,b            manual: front pair f, back pair b         M,fl,fr,rl,rr   each motor
//   U,rate           soft-start speed, us per second (25..1000, starts at 100)
//   L,1  /  L,0      live data on / off (PC)
//   X                KILL: all motors off
//   Z                zero the angles: hold the drone level, then send Z (roll AND pitch)
//   C                calibrate (motors off, drone still and level for 2 s). Also clears an IMU error.
//   ?                list the commands
//   (ARM, H, STOP, STATUS are handled in main.c)
// PIC -> PC tuner (USB):
//   D,t_ms,angle,setpoint,plus_us,P,I,D,minus_us,roll,pitch        one sample
//     angle/setpoint/P/I/D are the graphed axis (roll in ROLL mode, else pitch).
//     plus/minus = the pulse of the pair that gets +comp / -comp:
//     PITCH: front pair / back pair     ROLL: left pair / right pair     BOTH: front / back average
//   BOTH mode adds:  ,sp_roll,sp_pitch,esc1,esc2,esc3,esc4,RC,PC
//   RUN_BEGIN ... D lines ... RUN_END                 a recorded step test (t_ms from the step)
// PIC -> phone (Bluetooth is slow, ~1-3 kB/s at best, so the phone gets short lines):
//   A,kp,ki,kd   AR,...  AP,...                      gains now in use
//   AXIS,n,NAME                                      axis now selected
//   BASE,T,T                                         throttle now in use (two equal numbers: old apps read front,back)
//   RUN_BEGIN,samples,pre,from,to,T,T,kp,ki,kd,axis,kp_r,ki_r,kd_r
//       kp,ki,kd = graphed axis (pitch in BOTH), kp_r.. = roll gains
//   ROLL or PITCH:  RD,k,a,u,a,u,...   8 samples a line: a = angle x 10 (0.1 deg), u = RC or PC (us)
//   BOTH:           RD,k,r,p,rc,pc,...  4 samples a line: r, p = roll, pitch x 10
//   RUN_END
//   The phone rebuilds the 4 pulses with the same mixer as the 4 ms loop (1100..2000 us).
//
// Professor's rule for step tests: record EVERY 4 ms sample into arrays inside the
// interrupt (data[i] = ...), send NOTHING while recording, set data_complete = 1 when
// the arrays are full, and only then does main() send everything at once.
// That way printing can never make the 4 ms loop miss a sample.


static int tuner_ble_output=0;     // 1 while a command from the phone is being handled
static int run_phone=0;           // 1 = the step test was started from the phone (send the data there)
static int run_axis=2;            // axis_mode when the step test started
static int dump_k=-1;             // step-test dump: -1 = not started, else the next sample to send
// ---------------------------------------------------------------------------------------------
// void tuner_set_ble_output(int enabled)
//   Chooses where tuner replies go: 1 = phone (Bluetooth), 0 = PC (USB).
// ---------------------------------------------------------------------------------------------
void tuner_set_ble_output(int enabled) { tuner_ble_output=enabled; }

// ---------------------------------------------------------------------------------------------
// void tuner_reply(const char *text)
//   Sends a reply to whichever link sent the command (phone or PC).
// ---------------------------------------------------------------------------------------------
void tuner_reply(const char *text)  // to whichever link sent the command
{
    if (tuner_ble_output) ble_print(text);
    else putsU2((char *)text);
}

#define PRE_STEP   50       // samples kept from before the step (0.2 s)
#define RUN_MAX    5050     // longest recording: 0.2 s + 20 s = 5050 samples (13 arrays x 20 KB = 263 KB of 512 KB RAM)
#define PHONE_MAX_MS 15000  // from the phone: 15 s at most (it takes the slow Bluetooth link 15-60 s to send)
#define HOLD_LOOPS 250      // hold the start angle 1 s (after the soft start) before the step

volatile int display_on=1;
volatile int live_on=0;
volatile int run_state=0;
volatile int data_complete=0;
volatile int run_stopped=0;
volatile int run_count=0;
volatile int run_total=0;
volatile float run_from=0, run_to=0;

// the recording: one entry every 4 ms
float data_roll[RUN_MAX], data_pitch[RUN_MAX], data_sp_roll[RUN_MAX], data_sp_pitch[RUN_MAX];
float data_m1[RUN_MAX], data_m2[RUN_MAX], data_m3[RUN_MAX], data_m4[RUN_MAX];
float data_rc[RUN_MAX], data_pc[RUN_MAX], data_p[RUN_MAX], data_i[RUN_MAX], data_d[RUN_MAX];

// ---------------------------------------------------------------------------------------------
// const char *axis_name(int a)
//   Returns ROLL, PITCH or BOTH for axis 1, 2 or 3.
// ---------------------------------------------------------------------------------------------
const char *axis_name(int a)
{
    if (a==1) return "ROLL";
    if (a==2) return "PITCH";
    return "BOTH";
}

// ---------------------------------------------------------------------------------------------
// int read_numbers(char *line, float *n)
//   Reads up to 4 numbers after the first comma ('M,1200,1250,1300,1350' -> 4 values). Returns
//   how many.
// ---------------------------------------------------------------------------------------------
// Read up to 4 numbers after the command letters: "K,2.5,0,0.4" -> 2.5, 0, 0.4 and
// "KR,1,0,0.2" -> 1, 0, 0.2. Returns how many it found.
int read_numbers(char *line, float *n)
{
    char *p;
    int found=0;

    p=strchr(line, ',');        // the first comma ends the command letters
    while (p!=0 && found<4){
        p++;
        n[found]=atof(p);
        found++;
        p=strchr(p, ',');       // the next comma
    }
    return found;
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
// void tuner_help(void)
//   Lists the tuner commands (v3.1 adds O,fl,fr,rl,rr).
// ---------------------------------------------------------------------------------------------
void tuner_help(void)
{
    tuner_reply("Axis: A,1=ROLL  A,2=PITCH  A,3=BOTH (motors off).  ARM  H  X=KILL  ?=help");
    tuner_reply("K,kp,ki,kd (selected axis)  KR,..  KP,..   S,deg  SR,deg  SP,deg   R,from,to,ms   G=PID on");
    tuner_reply("B,T throttle   M,t / M,f,b / M,fl,fr,rl,rr manual   U,us_per_s ramp   Z=zero  C=calibrate  STATUS");
    tuner_reply("O,fl,fr,rl,rr = live +/- us per motor (any time, +/-200)   O = show them");
}

// ---------------------------------------------------------------------------------------------
// void reply_offsets(void)
//   Replies OFFS,fl,fr,rl,rr: the live +/- (us) each motor is using.
// ---------------------------------------------------------------------------------------------
void reply_offsets(void)        // OFFS,fl,fr,rl,rr  (live +/- per motor, us)
{
    char s[60];
    sprintf(s, "OFFS,%d,%d,%d,%d", (int)motor_adj[1], (int)motor_adj[2], (int)motor_adj[3], (int)motor_adj[4]);
    tuner_reply(s);
}

// ---------------------------------------------------------------------------------------------
// void reply_gains(char which)
//   Replies with gains in use: AR = roll, AP = pitch, A = the selected axis (pitch in Both).
// ---------------------------------------------------------------------------------------------
void reply_gains(char which)    // 'R' = AR line, 'P' = AP line, 0 = A line (selected axis, BOTH -> pitch)
{
    char s[80], t1[16], t2[16], t3[16];
    if (which=='R' || (which==0 && axis_mode==1))
        sprintf(s, "%s,%s,%s,%s", which ? "AR" : "A", fstr(t1, Kp_roll, 4), fstr(t2, Ki_roll, 4), fstr(t3, Kd_roll, 4));
    else
        sprintf(s, "%s,%s,%s,%s", which ? "AP" : "A", fstr(t1, Kp_pitch, 4), fstr(t2, Ki_pitch, 4), fstr(t3, Kd_pitch, 4));
    tuner_reply(s);
}

// ---------------------------------------------------------------------------------------------
// void set_gains(int axis, float kp, float ki, float kd)
//   Stores Kp, Ki, Kd for one axis (1 = roll, 2 = pitch).
// ---------------------------------------------------------------------------------------------
void set_gains(int axis, float kp, float ki, float kd)     // axis 1 = roll, 2 = pitch
{
    if (axis==1){
        Kp_roll=kp;
        Ki_roll=ki;
        Kd_roll=kd;
    }
    else{
        Kp_pitch=kp;
        Ki_pitch=ki;
        Kd_pitch=kd;
    }
}

// ---------------------------------------------------------------------------------------------
// void tuner_command(char *line)
//   Runs one tuner command: A axis, K/KR/KP gains, S/SR/SP targets, R step test, G hold, M
//   manual, B throttle, U soft start, O motor +/-, Z, C, X, L.
// ---------------------------------------------------------------------------------------------
void tuner_command(char *line)
{
    char s[100], t1[16], t2[16];
    float n[4]={0, 0, 0, 0};
    int found;

    display_on=0;                           // the tuner is talking: stop the PuTTY lines
    found=read_numbers(line, n);

    if ((line[0]=='R' || line[0]=='G' || line[0]=='M') && !control_can_start()){
        tuner_reply("Start blocked. Send ARM and wait for ESC startup, then start again.");
        return;
    }

    if (line[0]=='A'){                      // A,1 / A,2 / A,3: which axis is tuned
        if (found==1){
            if (motor_mode!=0 || run_state!=0)
                tuner_reply("Axis not changed: stop the motors first");
            else if (n[0]>=1 && n[0]<=3){
                axis_mode=(int)n[0];
                pid_reset();
            }
        }
        sprintf(s, "AXIS,%d,%s", axis_mode, axis_name(axis_mode));
        tuner_reply(s);
    }
    else if (line[0]=='K'){                 // K,kp,ki,kd  /  KR,...  /  KP,...
        if (line[1]=='R' || line[1]=='P'){
            if (found==3) set_gains(line[1]=='R' ? 1 : 2, n[0], n[1], n[2]);
            reply_gains(line[1]);
        }
        else{
            if (found==3){
                if (axis_mode!=2) set_gains(1, n[0], n[1], n[2]);     // ROLL or BOTH
                if (axis_mode!=1) set_gains(2, n[0], n[1], n[2]);     // PITCH or BOTH
            }
            reply_gains(0);
        }
    }
    else if (line[0]=='S' && line[1]!='T'){ // S,deg  /  S,roll,pitch  /  SR,deg  /  SP,deg  (not STATUS)
        if (run_state==0){
            if (line[1]=='R' && found==1) sp_roll=n[0];
            else if (line[1]=='P' && found==1) sp_pitch=n[0];
            else if (line[1]==','){
                if (axis_mode==3 && found==2){
                    sp_roll=n[0];
                    sp_pitch=n[1];
                }
                else if (found==1){
                    if (axis_mode!=2) sp_roll=n[0];
                    if (axis_mode!=1) sp_pitch=n[0];
                }
            }
            pid_reset();                    // new target: start the integrals fresh
        }
        sprintf(s, "SETPOINT,%s,%s", fstr(t1, sp_roll, 1), fstr(t2, sp_pitch, 1));
        tuner_reply(s);
    }
    else if (line[0]=='R'){                 // R,from,to,ms
        if (found==3 && run_state==0 && data_complete==0){
            if (n[2]>20000) n[2]=20000;     // the recording holds 20 s at most
            if (tuner_ble_output && n[2]>PHONE_MAX_MS) n[2]=PHONE_MAX_MS;
            run_phone=tuner_ble_output;     // the data goes back to whoever started the test
            if (n[2]<100) n[2]=100;
            run_from=n[0];
            run_to=n[1];
            run_axis=axis_mode;
            run_total=PRE_STEP+(int)(n[2]/4);   // one sample every 4 ms
            run_count=HOLD_LOOPS;
            if (axis_mode!=2) sp_roll=n[0];
            if (axis_mode!=1) sp_pitch=n[0];
            pid_reset();                    // professor: no old error carried into a new run
            run_stopped=0;
            motor_mode=1;                   // PID on (soft start if the motors were off)
            run_state=1;                    // set last: the interrupt takes it from here
            sprintf(s, "Step test started on %s (no data is sent until it is done)", axis_name(axis_mode));
            tuner_reply(s);
        }
    }
    else if (line[0]=='L'){                 // L,1 or L,0: live D lines to the PC (the phone gets P lines anyway)
        if (!tuner_ble_output) live_on=(line[2]=='1');
    }
    else if (line[0]=='X'){                 // KILL
        motors_off();
        if (run_state!=0 || run_stopped==1){
            run_state=0;
            tuner_reply("RUN_END");
        }
        run_stopped=0;                      // already ended here, main() must not end it again
        tuner_reply("KILL - motors off");
    }
    else if (line[0]=='G'){                 // PID on, hold the setpoints
        pid_reset();
        motor_mode=1;
        sprintf(s, "PID on (%s)", axis_name(axis_mode));
        tuner_reply(s);
    }
    else if (line[0]=='M'){                 // manual: M,t  /  M,f,b  /  M,fl,fr,rl,rr
        if (found==1 || found==2 || found==4){
            if (found==1){
                man_us[1]=man_us[2]=man_us[3]=man_us[4]=clamp_us(n[0]);
            }
            else if (found==2){
                man_us[1]=man_us[2]=clamp_us(n[0]);     // front pair
                man_us[3]=man_us[4]=clamp_us(n[1]);     // back pair
            }
            else{
                man_us[1]=clamp_us(n[0]);
                man_us[2]=clamp_us(n[1]);
                man_us[3]=clamp_us(n[2]);
                man_us[4]=clamp_us(n[3]);
            }
            motor_mode=2;
            sprintf(s, "Manual: FL %d  FR %d  RL %d  RR %d us", (int)man_us[1], (int)man_us[2],
                    (int)man_us[3], (int)man_us[4]);
            tuner_reply(s);
        }
    }
    else if (line[0]=='B'){                 // B,T  (B,f,b from an old app: the average)
        if (found==1) throttle=clamp_us(n[0]);
        if (found==2) throttle=clamp_us((n[0]+n[1])/2);
        sprintf(s, "BASE,%d,%d", (int)throttle, (int)throttle);
        tuner_reply(s);
    }
    else if (line[0]=='U'){                 // U,rate: soft-start speed in us per second
        if (found==1){
            if (n[0]<25) n[0]=25;
            if (n[0]>1000) n[0]=1000;
            ramp_rate=n[0];
        }
        sprintf(s, "RAMP,%d us/s", (int)ramp_rate);
        tuner_reply(s);
    }
    else if (line[0]=='O'){                 // O,fl,fr,rl,rr: live +/- us per motor (app buttons), works while flying
        if (found==4){
            set_motor_adj(1, n[0]);
            set_motor_adj(2, n[1]);
            set_motor_adj(3, n[2]);
            set_motor_adj(4, n[3]);
        }
        reply_offsets();                    // O alone: just report
    }
    else if (line[0]=='Z'){                 // zero both angles where the drone is now
        roll_offset=roll_offset+ang_roll;
        pitch_offset=pitch_offset+ang_pitch;
        sprintf(s, "Angles zeroed (offsets roll %s, pitch %s deg)", fstr(t1, roll_offset, 2), fstr(t2, pitch_offset, 2));
        tuner_reply(s);
    }
    else if (line[0]=='C'){                 // calibrate (also clears an IMU error, no reflash needed)
        if (run_state!=0 || run_stopped==1){
            run_state=0;
            tuner_reply("RUN_END");
        }
        run_stopped=0;
        calibrate_now();
        tuner_reply("Calibrated");
    }
    else if (line[0]=='?'){
        tuner_help();
    }
    else{
        sprintf(s, "Unknown command: %.40s  (? = help)", line);
        tuner_reply(s);
    }
}

// ---------------------------------------------------------------------------------------------
// float plus_pair(int axis, float m1, float m2, float m3)
//   The '+' side for the D line: average of the left pair (ESC 1, 3) in Roll only, of the front
//   pair (ESC 1, 2) otherwise.
// ---------------------------------------------------------------------------------------------
// The pair pulses for the D lines: the pair that gets +comp and the pair that gets -comp.
float plus_pair(int axis, float m1, float m2, float m3)     // ROLL: left (ESC 1, 3), else front (ESC 1, 2)
{
    return (axis==1) ? (m1+m3)/2 : (m1+m2)/2;
}

// ---------------------------------------------------------------------------------------------
// float minus_pair(int axis, float m2, float m3, float m4)
//   The '-' side for the D line: average of the right pair (ESC 2, 4) in Roll only, of the back
//   pair (ESC 3, 4) otherwise.
// ---------------------------------------------------------------------------------------------
float minus_pair(int axis, float m2, float m3, float m4)    // ROLL: right (ESC 2, 4), else back (ESC 3, 4)
{
    return (axis==1) ? (m2+m4)/2 : (m3+m4)/2;
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_live(void)
//   Sends one live D line: graphed angle and target, + side, P/I/D, - side, then roll and
//   pitch.
// ---------------------------------------------------------------------------------------------
void tuner_send_live(void)
{
    char s[120], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16], t7[16], t8[16], t9[16];

    sprintf(s, "D,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s", loop_count*4,
            fstr(t1, graph_angle(), 2), fstr(t2, graph_setpoint(), 2),
            fstr(t3, plus_pair(axis_mode, motor_us[1], motor_us[2], motor_us[3]), 0),
            fstr(t4, p_term, 1), fstr(t5, i_term, 1), fstr(t6, d_term, 1),
            fstr(t7, minus_pair(axis_mode, motor_us[2], motor_us[3], motor_us[4]), 0),
            fstr(t8, ang_roll, 2), fstr(t9, ang_pitch, 2));
    putsU2(s);
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_run(void)
//   Sends the recorded step test to the PC, one D line per call (Both adds both targets, the 4
//   ESC pulses, RC and PC).
// ---------------------------------------------------------------------------------------------
void tuner_send_run(void)
{
    // ONE line per call (main calls it once per 4 ms loop while data_complete = 1).
    // main() keeps reading commands between lines: the H heartbeat and X still work
    // while a 20 s recording is being sent.
    char s[200], t[18][16];
    int k;

    if (dump_k<0){
        putsU2("RUN_BEGIN");
        dump_k=0;
        return;
    }
    if (dump_k<run_total){
        k=dump_k;                   // the pairs and the graphed axis of the RECORDED run (run_axis)
        sprintf(s, "D,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s", (k-PRE_STEP)*4,
                fstr(t[0], run_axis==1 ? data_roll[k] : data_pitch[k], 2),
                fstr(t[1], run_axis==1 ? data_sp_roll[k] : data_sp_pitch[k], 2),
                fstr(t[2], plus_pair(run_axis, data_m1[k], data_m2[k], data_m3[k]), 0),
                fstr(t[3], data_p[k], 1), fstr(t[4], data_i[k], 1), fstr(t[5], data_d[k], 1),
                fstr(t[6], minus_pair(run_axis, data_m2[k], data_m3[k], data_m4[k]), 0),
                fstr(t[7], data_roll[k], 2), fstr(t[8], data_pitch[k], 2));
        if (run_axis==3){           // BOTH: every motor and both controllers
            sprintf(s+strlen(s), ",%s,%s,%s,%s,%s,%s,%s,%s",
                    fstr(t[9], data_sp_roll[k], 1), fstr(t[10], data_sp_pitch[k], 1),
                    fstr(t[11], data_m1[k], 0), fstr(t[12], data_m2[k], 0),
                    fstr(t[13], data_m3[k], 0), fstr(t[14], data_m4[k], 0),
                    fstr(t[15], data_rc[k], 1), fstr(t[16], data_pc[k], 1));
        }
        putsU2(s);
        dump_k++;
        return;
    }
    putsU2("RUN_END");
    dump_k=-1;
    data_complete=0;                        // ready for the next step test
}

// ---------------------------------------------------------------------------------------------
// int round_i(float v)
//   Rounds a float to the nearest int.
// ---------------------------------------------------------------------------------------------
int round_i(float v)                        // nearest whole number
{
    if (v<0) return (int)(v-0.5f);
    return (int)(v+0.5f);
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_run_phone(void)
//   Sends the recorded step test to the phone when the Bluetooth queue has room:
//   angle/correction pairs, or roll, pitch, RC, PC in Both.
// ---------------------------------------------------------------------------------------------
// Step-test data to the PHONE. main() calls this every pass while data_complete = 1.
// It only adds a line when the Bluetooth send queue has room, so it never waits.
// Nothing else goes to the phone until RUN_END (except replies and DISARMED).
void tuner_send_run_phone(void)
{
    char s[160], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16], t7[16], t8[16];
    int k, n;
    float kp, ki, kd;

    if (!ble_connected){                    // phone gone: drop the data, ready for the next test
        dump_k=-1;
        data_complete=0;
        return;
    }
    if (ble_room()<180) return;             // queue busy: try again next pass
    if (dump_k<0){
        kp=(run_axis==1) ? Kp_roll : Kp_pitch;
        ki=(run_axis==1) ? Ki_roll : Ki_pitch;
        kd=(run_axis==1) ? Kd_roll : Kd_pitch;
        sprintf(s, "RUN_BEGIN,%d,%d,%s,%s,%d,%d,%s,%s,%s,%d,%s,%s,%s", run_total, PRE_STEP,
                fstr(t1, run_from, 1), fstr(t2, run_to, 1), (int)throttle, (int)throttle,
                fstr(t3, kp, 4), fstr(t4, ki, 4), fstr(t5, kd, 4), run_axis,
                fstr(t6, Kp_roll, 4), fstr(t7, Ki_roll, 4), fstr(t8, Kd_roll, 4));
        ble_print(s);
        dump_k=0;
        return;
    }
    if (dump_k<run_total){
        sprintf(s, "RD,%d", dump_k);        // index of the first sample on this line
        if (run_axis==3){                   // BOTH: roll, pitch, RC, PC - 4 samples a line
            for (k=dump_k; k<dump_k+4 && k<run_total; k++){
                n=strlen(s);
                sprintf(s+n, ",%d,%d,%d,%d", round_i(data_roll[k]*10), round_i(data_pitch[k]*10),
                        round_i(data_rc[k]), round_i(data_pc[k]));
            }
        }
        else{                               // one axis: angle and its compensation - 8 samples a line
            for (k=dump_k; k<dump_k+8 && k<run_total; k++){
                n=strlen(s);
                if (run_axis==1) sprintf(s+n, ",%d,%d", round_i(data_roll[k]*10), round_i(data_rc[k]));
                else sprintf(s+n, ",%d,%d", round_i(data_pitch[k]*10), round_i(data_pc[k]));
            }
        }
        ble_print(s);
        dump_k=k;
        return;
    }
    ble_print("RUN_END");
    dump_k=-1;
    data_complete=0;                        // ready for the next step test
}

// ---------------------------------------------------------------------------------------------
// int tuner_run_is_phone(void)
//   1 if the step test in progress was started from the phone.
// ---------------------------------------------------------------------------------------------
int tuner_run_is_phone(void) { return run_phone; }

// ---- called from the 4 ms interrupt ----

// ---------------------------------------------------------------------------------------------
// void tuner_setpoint_step(void)
//   Runs every 4 ms in a step test: waits for the soft start, holds the start angle 1 s, then
//   steps the selected axis (both in Both).
// ---------------------------------------------------------------------------------------------
void tuner_setpoint_step(void)
{
    if (run_state!=0 && motor_mode!=1){
        run_state=0;                        // the motors were turned off: main() ends the test
        run_stopped=1;
        return;
    }
    if (run_state==1){                      // holding the start angle for 1 s
        if (run_axis!=2) sp_roll=run_from;
        if (run_axis!=1) sp_pitch=run_from;
        if (!ramp_done()) return;           // wait until the soft start has reached the throttle
        run_count--;
        if (run_count<=0) run_state=2;      // run_count is 0 now: recording starts at sample 0
    }
    else if (run_state==2){
        if (run_count==PRE_STEP){           // THE STEP (t = 0)
            if (run_axis!=2) sp_roll=run_to;
            if (run_axis!=1) sp_pitch=run_to;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// void tuner_record(void)
//   Runs every 4 ms during a step test: stores roll, pitch, both targets, the 4 ESC pulses, RC,
//   PC and the graphed axis's P/I/D.
// ---------------------------------------------------------------------------------------------
void tuner_record(void)
{
    if (run_state!=2) return;

    data_roll[run_count]=ang_roll;          // data[i] = ...
    data_pitch[run_count]=ang_pitch;
    data_sp_roll[run_count]=sp_roll;
    data_sp_pitch[run_count]=sp_pitch;
    data_m1[run_count]=motor_us[1];
    data_m2[run_count]=motor_us[2];
    data_m3[run_count]=motor_us[3];
    data_m4[run_count]=motor_us[4];
    data_rc[run_count]=roll_comp;
    data_pc[run_count]=pitch_comp;
    data_p[run_count]=p_term;
    data_i[run_count]=i_term;
    data_d[run_count]=d_term;
    run_count++;
    if (run_count>=run_total){
        run_state=0;
        motor_mode=0;                       // professor: motors stop at the end of every run (1000 us,
                                            // still armed, relay stays closed), so nothing is being
                                            // tuned while the data is sent and the next run starts fresh
        data_complete=1;                    // professor's flag: main() sends it all now
    }
}
