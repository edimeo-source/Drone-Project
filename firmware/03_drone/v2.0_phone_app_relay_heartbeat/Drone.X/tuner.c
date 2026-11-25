/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/Drone.X/tuner.c
 *  What    : PID-tuner protocol (PC or phone): gains, setpoint, recorded step tests, live data.
 *  Version : v2.0   |   Date: 2026-11-23
 *  Status  : Builds in MPLAB X with XC32 and was flown on the tethered bench (Nov 24). Host
 *            tests pass. The copy on the laptop has TIP_LIMIT 60 (this repo copy: 75). Replaced
 *            by v3.0 (two PIDs).
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
 *    tuner_reply(), read_numbers(), clamp_us(), tuner_help(), tuner_command(),
 *    tuner_send_live(), tuner_send_run(), round_i(), tuner_send_run_phone(),
 *    tuner_run_is_phone(), tuner_setpoint_step(), tuner_record()
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
// Tuner -> PIC (one line each):
//   K,kp,ki,kd       new gains                         (PIC answers A,kp,ki,kd)
//   S,deg            new pitch setpoint                (integral reset)
//   R,from,to,ms     step test: PID on, soft start, hold "from" 1 s, step to "to", record ms
//   L,1  /  L,0      live data on / off
//   X                KILL: all motors off
//   G                PID on, hold the setpoint (no step test)
//   M,f,b            manual: PID off, front pair = f us, back pair = b us
//   B,f,b            PID base pulse of the front pair and back pair (B alone = just report it)
//   Z                zero the angle: hold the drone level, then send Z
//   C                calibrate (motors off, drone still and level for 2 s). Also clears an IMU error.
//   ?                list the commands
//   (ARM, H, STOP, STATUS are handled in main.c)
// PIC -> PC tuner (USB):
//   D,t_ms,angle,setpoint,front_us,P,I,D,back_us     one sample
//   RUN_BEGIN ... D lines ... RUN_END                 a recorded step test (t_ms from the step)
// PIC -> phone (Bluetooth is slow, ~100-300 bytes/s, so the phone gets short lines):
//   A,kp,ki,kd                                       gains now in use
//   BASE,front,back                                  base pulses now in use
//   RUN_BEGIN,samples,pre,from,to,base_f,base_b,kp,ki,kd   then
//   RD,k,a,u,a,u,...   8 samples per line from sample k: a = angle x 10 (0.1 deg), u = PID output (us)
//   RUN_END
//   The phone rebuilds the motor pulses with the same mixer as the 4 ms loop:
//   front = base_f + u, back = base_b - u, each kept inside 1100..2000 us. About 8 bytes a
//   sample, so a 3 s test (800 samples, ~6 kB) takes about 20-60 s to reach the phone.
//
// Professor's rule for step tests: record EVERY 4 ms sample into arrays inside the
// interrupt (data[i] = ...), send NOTHING while recording, set data_complete = 1 when
// the arrays are full, and only then does main() send everything at once.
// That way printing can never make the 4 ms loop miss a sample.


static int tuner_ble_output=0;     // 1 while a command from the phone is being handled
static int run_phone=0;           // 1 = the step test was started from the phone (send the data there)
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
#define RUN_MAX    5050     // longest recording: 0.2 s + 20 s = 5050 samples (8 arrays x 20 KB = 162 KB of 512 KB RAM)
#define PHONE_MAX_MS 5000   // from the phone: 5 s at most (1300 samples, ~10 kB over the slow Bluetooth link)
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
float data_angle[RUN_MAX], data_sp[RUN_MAX], data_front[RUN_MAX], data_back[RUN_MAX];
float data_p[RUN_MAX], data_i[RUN_MAX], data_d[RUN_MAX], data_u[RUN_MAX];

// ---------------------------------------------------------------------------------------------
// int read_numbers(char *line, float *n1, float *n2, float *n3)
//   Reads up to 3 numbers after the command letter: 'K,2.5,0,0.4' -> 2.5, 0, 0.4. Returns how
//   many.
// ---------------------------------------------------------------------------------------------
// read up to 3 numbers after the letter: "K,2.5,0,0.4" -> 2.5, 0, 0.4. Returns how many it found.
int read_numbers(char *line, float *n1, float *n2, float *n3)
{
    char *p;

    if (line[1]!=',') return 0; // no numbers at all
    p=line+2;                   // skip "K,"
    *n1=atof(p);
    p=strchr(p, ',');           // find the next comma
    if (p==0) return 1;
    p++;
    *n2=atof(p);
    p=strchr(p, ',');
    if (p==0) return 2;
    p++;
    *n3=atof(p);
    return 3;
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
//   Sends the list of tuner commands.
// ---------------------------------------------------------------------------------------------
void tuner_help(void)
{
    tuner_reply("Commands: ARM  H  X=KILL  K,kp,ki,kd  S,deg  R,from,to,ms  G=PID on  M,front,back");
    tuner_reply("          B,front,back  Z=zero  C=calibrate  STATUS  L,1/L,0 (PC only)  ?=help");
}

// ---------------------------------------------------------------------------------------------
// void tuner_command(char *line)
//   Runs one command line from the PID tuner (K gains, S setpoint, R step test, G, M, B, X, Z,
//   ...).
// ---------------------------------------------------------------------------------------------
void tuner_command(char *line)
{
    char s[100], t1[16], t2[16], t3[16];
    float n1=0, n2=0, n3=0;
    int found;

    display_on=0;                           // the tuner is talking: stop the PuTTY lines
    found=read_numbers(line, &n1, &n2, &n3);

    if ((line[0]=='R' || line[0]=='G' || line[0]=='M') && !control_can_start()){
        tuner_reply("Start blocked. Send ARM and wait for ESC startup, then start again.");
        return;
    }

    if (line[0]=='K'){                      // K,kp,ki,kd
        if (found==3){
            Kp=n1;
            Ki=n2;
            Kd=n3;
        }
        sprintf(s, "A,%s,%s,%s", fstr(t1, Kp, 4), fstr(t2, Ki, 4), fstr(t3, Kd, 4));
        tuner_reply(s);
    }
    else if (line[0]=='S'){                 // S,deg
        if (found==1 && run_state==0){
            setpoint=n1;
            pid_reset();                    // new target: start the integral fresh
        }
    }
    else if (line[0]=='R'){                 // R,from,to,ms
        if (found==3 && run_state==0 && data_complete==0){
            if (n3>20000) n3=20000;         // the recording holds 20 s at most
            if (tuner_ble_output && n3>PHONE_MAX_MS) n3=PHONE_MAX_MS;
            run_phone=tuner_ble_output;     // the data goes back to whoever started the test
            if (n3<100) n3=100;
            run_from=n1;
            run_to=n2;
            run_total=PRE_STEP+(int)(n3/4); // one sample every 4 ms
            run_count=HOLD_LOOPS;
            setpoint=n1;
            pid_reset();                    // professor: no old error carried into a new run
            run_stopped=0;
            motor_mode=1;                   // PID on (soft start if the motors were off)
            run_state=1;                    // set last: the interrupt takes it from here
            tuner_reply("Step test started (no data is sent until it is done)");
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
    else if (line[0]=='G'){                 // PID on, hold the setpoint
        pid_reset();
        motor_mode=1;
        tuner_reply("PID on");
    }
    else if (line[0]=='M'){                 // M,front,back manual
        if (found==2){
            man_front=clamp_us(n1);
            man_back=clamp_us(n2);
            motor_mode=2;
            sprintf(s, "Manual: front %s us  back %s us", fstr(t1, man_front, 0), fstr(t2, man_back, 0));
            tuner_reply(s);
        }
    }
    else if (line[0]=='B'){                 // B,front,back PID base
        if (found==2){
            base_front=clamp_us(n1);
            base_back=clamp_us(n2);
        }
        sprintf(s, "BASE,%d,%d", (int)base_front, (int)base_back);
        tuner_reply(s);
    }
    else if (line[0]=='Z'){                 // zero the angle where the drone is now
        angle_offset=angle_offset+angle;
        sprintf(s, "Angle zeroed (offset %s deg)", fstr(t1, angle_offset, 2));
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
// void tuner_send_live(void)
//   Sends one live data line (D,...) to the tuner graph.
// ---------------------------------------------------------------------------------------------
void tuner_send_live(void)
{
    char s[110], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16], t7[16];

    sprintf(s, "D,%lu,%s,%s,%s,%s,%s,%s,%s", loop_count*4,
            fstr(t1, angle, 2), fstr(t2, setpoint, 2), fstr(t3, motor_front, 0),
            fstr(t4, p_term, 1), fstr(t5, i_term, 1), fstr(t6, d_term, 1),
            fstr(t7, motor_back, 0));
    putsU2(s);
}

// ---------------------------------------------------------------------------------------------
// void tuner_send_run(void)
//   Sends the recorded step test to the PC, one line per call, so commands still work
//   meanwhile.
// ---------------------------------------------------------------------------------------------
void tuner_send_run(void)
{
    // ONE line per call (main calls it once per 4 ms loop while data_complete = 1).
    // A 46-character line takes about 4 ms at 115200 baud, so this still runs at the
    // speed of the serial port, but main() keeps reading commands between lines:
    // the H heartbeat and X still work while a 20 s recording is being sent.
    char s[110], t1[16], t2[16], t3[16], t4[16], t5[16], t6[16], t7[16];
    int k;

    if (dump_k<0){
        putsU2("RUN_BEGIN");
        dump_k=0;
        return;
    }
    if (dump_k<run_total){
        k=dump_k;
        sprintf(s, "D,%d,%s,%s,%s,%s,%s,%s,%s", (k-PRE_STEP)*4,
                fstr(t1, data_angle[k], 2), fstr(t2, data_sp[k], 2), fstr(t3, data_front[k], 0),
                fstr(t4, data_p[k], 1), fstr(t5, data_i[k], 1), fstr(t6, data_d[k], 1),
                fstr(t7, data_back[k], 0));
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
//   Sends the recorded step test to the phone, only when the Bluetooth queue has room.
// ---------------------------------------------------------------------------------------------
// Step-test data to the PHONE. main() calls this every pass while data_complete = 1.
// It only adds a line when the Bluetooth send queue has room, so it never waits.
// Nothing else goes to the phone until RUN_END (except replies and DISARMED).
void tuner_send_run_phone(void)
{
    char s[140], t1[16], t2[16], t3[16], t4[16], t5[16];
    int k, n;

    if (!ble_connected){                    // phone gone: drop the data, ready for the next test
        dump_k=-1;
        data_complete=0;
        return;
    }
    if (ble_room()<160) return;             // queue busy: try again next pass
    if (dump_k<0){
        sprintf(s, "RUN_BEGIN,%d,%d,%s,%s,%d,%d,%s,%s,%s", run_total, PRE_STEP,
                fstr(t1, run_from, 1), fstr(t2, run_to, 1), (int)base_front, (int)base_back,
                fstr(t3, Kp, 4), fstr(t4, Ki, 4), fstr(t5, Kd, 4));
        ble_print(s);
        dump_k=0;
        return;
    }
    if (dump_k<run_total){
        sprintf(s, "RD,%d", dump_k);        // index of the first sample on this line
        for (k=dump_k; k<dump_k+8 && k<run_total; k++){
            n=strlen(s);
            sprintf(s+n, ",%d,%d", round_i(data_angle[k]*10), round_i(data_u[k]));
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
//   Runs every 4 ms: holds the start angle, then makes the setpoint step for a step test.
// ---------------------------------------------------------------------------------------------
void tuner_setpoint_step(void)
{
    if (run_state!=0 && motor_mode!=1){
        run_state=0;                        // the motors were turned off: main() ends the test
        run_stopped=1;
        return;
    }
    if (run_state==1){                      // holding the start angle for 1 s
        setpoint=run_from;
        if (ramp_us<PULSE_MAX) return;      // wait until the soft start is done
        run_count--;
        if (run_count<=0) run_state=2;      // run_count is 0 now: recording starts at sample 0
    }
    else if (run_state==2){
        if (run_count==PRE_STEP) setpoint=run_to;   // THE STEP (t = 0)
    }
}

// ---------------------------------------------------------------------------------------------
// void tuner_record(void)
//   Runs every 4 ms during a step test: stores angle, setpoint, motor pulses and P/I/D terms.
// ---------------------------------------------------------------------------------------------
void tuner_record(void)
{
    if (run_state!=2) return;

    data_angle[run_count]=angle;            // data[i] = ...
    data_sp[run_count]=setpoint;
    data_front[run_count]=motor_front;
    data_back[run_count]=motor_back;
    data_p[run_count]=p_term;
    data_i[run_count]=i_term;
    data_d[run_count]=d_term;
    data_u[run_count]=u;                    // the PID output (us) that went into the mixer
    run_count++;
    if (run_count>=run_total){
        run_state=0;
        motor_mode=0;                       // professor: motors stop at the end of every run (1000 us,
                                            // still armed, relay stays closed), so nothing is being
                                            // tuned while the data is sent and the next run starts fresh
        data_complete=1;                    // professor's flag: main() sends it all now
    }
}
