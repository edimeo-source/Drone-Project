/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/Drone.X/main.c
 *  What    : Program entry: start-up, then the main loop (PuTTY keys, tuner/phone commands,
 *            status).
 *  Version : v2.0   |   Date: 2026-11-23
 *  Status  : CURRENT. Host tests pass (simulated Bluefruit, arming, relay, heartbeat, cutoff).
 *            Not yet compiled with XC32 or flown.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): init_buttons_leds(), deadband(),
 *    print_help(), report(), off_text(), print_header(), print_line(), handle_key(),
 *    handle_line(), main()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include <stdio.h>
# include <string.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz, watchdogs off (same file as Lab 6)
# include "delay.h"
# include "uart.h"
# include "imu.h"
# include "motors.h"
# include "control.h"
# include "tuner.h"
# include "noise.h"
# include "bluefruit.h"

// ELC 275 capstone - DRONE v2.0: pitch PID on 4 ESCs, phone app over Bluetooth, motor-power relay
// Ethan DiMeo
//
// Every 4 ms the Timer4 interrupt (control.c) reads the IMU, runs the complementary
// filter and the PID, and sends the pulses to the 4 ESCs (professor's two PWM groups):
//   front pair (ESC 1 pin 11 + ESC 2 pin 12) = base_front + u
//   back pair  (ESC 3 pin 21 + ESC 4 pin 19) = base_back  - u
// Only PITCH is controlled (nose up/down). Two groups can't control roll: that would need
// the left and right motors to get different pulses. Roll is watched for the tip limit.
//
// main() does everything that is NOT time-critical:
//   - commands from the phone app (Bluetooth) and from the PC (USB: PuTTY or the PC tuner)
//   - status lines, and the step-test data after each run
//
// Hardware (J12): IMU and Bluefruit share SCK 23, MOSI 37, MISO 7 (splitter wires).
//   IMU CS 16, 3.3 V from pin 1.  Bluefruit CS 13, VIN 5 V from pin 2.  Relay CH1 pin 33 (RK4).
//   IMU in the middle of the bottom plate, X arrow to the TRUE FRONT (end opposite the battery pads).
//
// PuTTY (115200 baud) keys: c = recalibrate, z = zero yaw, n = 60 s noise test, x = KILL, h = help
// Kit buttons: SW1 = KILL, SW2 = recalibrate. Never press SW3 (it is the serial transmit pin).

#define RATE_DEADBAND 0.3f      // deg/s: ON SCREEN ONLY, smaller rates show as 0.0.
                                // The PID always uses the real numbers.

int noise_waiting=0;            // 1 = a noise test is running, print the report when it ends
int lines=0;                    // data lines printed since the last header

// ---------------------------------------------------------------------------------------------
// void init_buttons_leds(void)
//   Sets LED1-LED3 (RH0-RH2) as outputs and SW1/SW2 (RB12/RB13) as inputs with pull-ups.
// ---------------------------------------------------------------------------------------------
void init_buttons_leds(void)
{
    ANSELHbits.ANSH0=0;         // LED1 (RH0) = motors on
    ANSELHbits.ANSH1=0;         // LED2 (RH1) = IMU OK and calibrated
    TRISHbits.TRISH0=0;         // LED3 (RH2) = high during each 4 ms loop (scope it)
    TRISHbits.TRISH1=0;
    TRISHbits.TRISH2=0;
    LATHbits.LATH0=0;
    LATHbits.LATH1=0;
    LATHbits.LATH2=0;

    ANSELBbits.ANSB12=0;        // SW1 (RB12) and SW2 (RB13): digital inputs,
    ANSELBbits.ANSB13=0;        // pressed = 0, so they need the pull-ups
    TRISBbits.TRISB12=1;
    TRISBbits.TRISB13=1;
    CNPUBbits.CNPUB12=1;
    CNPUBbits.CNPUB13=1;
}

// ---------------------------------------------------------------------------------------------
// float deadband(float v, float band)
//   Returns 0 if v is inside +/-band, otherwise v (stops tiny noise from moving the motors).
// ---------------------------------------------------------------------------------------------
float deadband(float v, float band)
{
    if (v<band && v>-band) return 0;
    return v;
}

// ---------------------------------------------------------------------------------------------
// void print_help(void)
//   Prints the keys, kit buttons and sign conventions.
// ---------------------------------------------------------------------------------------------
void print_help(void)
{
    putsU2("");
    putsU2(" Keys : c = recalibrate (still + level)   z = zero yaw   n = 60 s noise test   x = KILL   h = help");
    putsU2(" Kit  : SW1 = KILL   SW2 = recalibrate   (never press SW3: it is the serial transmit pin)");
    putsU2(" Signs: pitch + = nose up (X arrow = front), roll + = left side up, yaw + = nose turns left");
    putsU2(" PID controls PITCH: front pair (ESC 1+2) = base + u,  back pair (ESC 3+4) = base - u");
    putsU2(" Motors start OFF and the relay open (RK4 low = motor power cut). ARM closes the relay;");
    putsU2(" the app then sends H every 100 ms. No H for 500 ms = motors off, relay open, ARM again.");
    putsU2(" Wait 2 s after ARM (ESCs start up), then start: step test, PID on, or manual.");
    putsU2(" Phone app: Connect -> ARM -> step test. KILL works any time. A plain Bluetooth");
    putsU2(" terminal can send STOP / STATUS, but it can't send the heartbeat, so it can't ARM.");
}

// ---------------------------------------------------------------------------------------------
// void report(char *s)
//   Sends a status line to the PC and, if a phone is connected, to the phone too.
// ---------------------------------------------------------------------------------------------
// Status lines the tuner app reads (it stops its heartbeat on DISARMED).
// They go to USB and, if a Bluetooth central is connected, to Bluetooth too.
void report(char *s)
{
    putsU2(s);
    ble_print(s);       // does nothing if no phone is connected
}

// ---------------------------------------------------------------------------------------------
// const char *off_text(int why)
//   Returns the reason the motors were turned off, as text (SW1, IMU, tip, remote STOP, link
//   lost).
// ---------------------------------------------------------------------------------------------
const char *off_text(int why)
{
    if (why==OFF_SW1) return "SW1 pressed";
    if (why==OFF_IMU) return "IMU stopped answering";
    if (why==OFF_TIP) return "pitch or roll past 75 deg";
    if (why==OFF_REMOTE) return "remote STOP";
    if (why==OFF_BLE) return "heartbeat or link lost";
    return "stopped by command (X, calibrate or noise test)";
}

// ---------------------------------------------------------------------------------------------
// void print_header(void)
//   Prints the column names for the live PuTTY lines.
// ---------------------------------------------------------------------------------------------
void print_header(void)
{
    char s[120], t[16], k1[16], k2[16], k3[16];

    sprintf(s, "LSM6DSO 833 Hz | loop 4 ms | %s C | Kp %s Ki %s Kd %s | keys: c z n x h",
            fstr(t, temp_c, 1), fstr(k1, Kp, 3), fstr(k2, Ki, 3), fstr(k3, Kd, 3));
    putsU2(s);
    putsU2(" time(s)|  PITCH   ROLL    YAW deg|PITCH/s ROLL/s  YAW/s deg/s|      u FRONT  BACK us");
}

// ---------------------------------------------------------------------------------------------
// void print_line(void)
//   Prints one line of live readings (angles, rates, motor pulses) to PuTTY.
// ---------------------------------------------------------------------------------------------
void print_line(void)
{
    char s[120], t[16], r[16], p[16], y[16], rr[16], pr[16], yr[16], uu[16];

    if (lines%30==0) print_header();    // the column names again every 30 lines
    lines++;

    sprintf(s, "%7s |%7s%7s%7s    |%7s%7s%7s      |%7s%6d%6d",
            fstr(t, loop_count*0.004f, 2),
            fstr(p, pitch, 1), fstr(r, roll, 1), fstr(y, yaw, 1),
            fstr(pr, deadband(pitch_rate, RATE_DEADBAND), 1),
            fstr(rr, deadband(roll_rate, RATE_DEADBAND), 1),
            fstr(yr, deadband(yaw_rate, RATE_DEADBAND), 1),
            fstr(uu, u, 1), (int)motor_front, (int)motor_back);
    putsU2(s);
}

// ---------------------------------------------------------------------------------------------
// void handle_key(int c)
//   Runs one PuTTY key: c = calibrate, z = zero yaw, n = noise test, x = KILL, h = help.
// ---------------------------------------------------------------------------------------------
void handle_key(int c)
{
    live_on=0;                          // someone is typing in PuTTY: back to the PuTTY lines
    display_on=1;

    if (c=='c') calibrate_now();
    else if (c=='z'){
        yaw=0;
        putsU2(" Yaw zeroed.");
    }
    else if (c=='n'){
        motors_off();                   // motors shake the table
        putsU2(" Noise test (motors off): keep the drone completely STILL for 60 s (don't touch the table)...");
        display_on=0;
        noise_start();
        noise_waiting=1;
    }
    else if (c=='x'){
        motors_off();
        putsU2(" KILL - motors off.");
    }
    else if (c=='h') print_help();
    lines=0;                            // print the column names next
}

// ---------------------------------------------------------------------------------------------
// void handle_line(char *line, int phone)
//   Runs one command line from the phone (phone = 1) or the PC (phone = 0); the reply goes back
//   the same way.
// ---------------------------------------------------------------------------------------------
// One command line from the phone (phone = 1) or the PC (phone = 0).
// The reply goes back to the same place.
void handle_line(char *line, int phone)
{
    char s[80], t1[16], t2[16];
    int source=phone ? 1 : 2;           // 1 = phone, 2 = PC (control.c's numbering)

    tuner_set_ble_output(phone);
    if (strcmp(line, "H")==0) control_heartbeat(source);   // heartbeat: no reply
    else if (strcmp(line, "ARM")==0){
        if (control_arm(source)) tuner_reply("ARMED: relay on, ESCs at 1000 us - wait 2 s, then start");
        else tuner_reply("DISARMED,ARM refused - needs calibration, SW1 up, within 75 deg, motors off");
    }
    else if (strcmp(line, "STOP")==0 || strcmp(line, "KILL")==0) tuner_command("X");
    else if (strcmp(line, "STATUS")==0){
        sprintf(s, "STATUS,%s,pitch %s,roll %s", kill_latched ? "disarmed" : "armed",
                fstr(t1, angle, 1), fstr(t2, roll, 1));
        tuner_reply(s);
    }
    else tuner_command(line);           // K, S, R, G, M, B, Z, C, X, L, ? (tuner.c)
    tuner_set_ble_output(0);
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: board, IMU, ESCs, relay and Bluefruit set up, then PC and phone command
//   lines every pass; the flying runs in Timer4.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char s[120];
    char line[48];          // a PC command being received, like "K,2.5,0,0.4"
    char pline[48];         // a phone command
    int len=0;
    int c, id, bad;
    int err_shown=0;
    int was_armed=0;        // 1 = main has seen the drone armed (so it reports DISARMED once)
    int live_paused=0;      // 1 = the "live data paused" note was already printed
    unsigned long last=0;

    initDelay();            // Timer3 first, everything else uses delays
    initMotors();           // relay pin LOW first (power cut), then 1000 us pulses so the ESCs stay off
    initU2();
    TRISJbits.TRISJ13=0;    // Bluefruit CS: output, HIGH = not selected, before any SPI traffic
    LATJbits.LATJ13=1;
    initSPI();              // SPI1 + IMU CS (both CS lines high)
    init_buttons_leds();
    INTCONbits.MVEC=1;              // multi-vector interrupts
    __builtin_enable_interrupts();  // interrupts on (UART receive now, the 4 ms loop later)

    putsU2("");
    putsU2("=====================================================================");
    putsU2(" ELC275 drone v2.0 - pitch PID, 4 ESCs (pins 11, 12, 21, 19), relay pin 33");
    putsU2("=====================================================================");

    // 1. name first: WHO_AM_I must read 0x6C
    id=imu_who_am_i();
    while (id!=0x6C){
        sprintf(s, " WHO_AM_I = 0x%02X, want 0x6C. If 0x7F: the IMU's SDO / MISO wire (pin 7) isn't connected.", id);
        putsU2(s);
        delay_ms(1000);
        id=imu_who_am_i();
    }
    putsU2(" IMU  : LSM6DSO (WHO_AM_I = 0x6C) - 3-axis gyroscope + 3-axis accelerometer");
    putsU2(" Range: gyro +/-500 deg/s, accel +/-4 g, both 833 Hz with the chip's low-pass filters on");

    // 2. settings, each read back
    putsU2(" Settings:");
    bad=initIMU();
    while (bad>0){
        motors_off();
        putsU2(" IMU settings failed - motors locked off. Check SDO pin 7. Retrying.");
        delay_ms(1000);
        bad=initIMU();
    }

    // 3. calibrate, then start the 4 ms loop. The Bluefruit starts in the main loop
    //    (it has had the 2+ s of IMU start-up to boot; it needs about 1 s).
    calibrate_now();
    initControl();
    print_help();
    lines=0;

    while (1){
        // 1. Bluetooth: link check, read the phone, send queued text (waits for the module)
        ble_service();
        while (ble_get_line(pline)) handle_line(pline, 1);

        // 2. PC: PuTTY keys and tuner commands
        c=getU2();
        while (c!=-1){
            if (len==0 && c>='a' && c<='z'){
                handle_key(c);              // PuTTY: one lowercase key, no Enter needed
            }
            else if (c=='\n' || c=='\r'){   // end of a tuner line
                if (len>0){
                    line[len]=0;
                    handle_line(line, 0);
                    len=0;
                    lines=0;
                }
            }
            else if (len<47){               // tuner commands start with a capital letter
                line[len]=c;
                len++;
            }
            c=getU2();
        }

        // 3. kit buttons
        if (PORTBbits.RB12==0){             // SW1 = KILL (the 4 ms interrupt also checks it)
            motors_off();
            off_reason=OFF_SW1;             // printed (and reported) once, below
            while (PORTBbits.RB12==0);
            delay_ms(20);
        }
        if (PORTBbits.RB13==0){             // SW2 = recalibrate
            motors_off();
            while (PORTBbits.RB13==0);
            delay_ms(20);
            calibrate_now();
            lines=0;
        }

        // 4. step-test data for the phone: added whenever the send queue has room
        if (data_complete==1 && tuner_run_is_phone()) tuner_send_run_phone();

        // 5. once per new 4 ms loop
        if (loop_count!=last){
            last=loop_count;

            // Armed -> disarmed, for ANY reason: tell the phone and the PC, so the app
            // stops its heartbeat and shows why.
            if (!kill_latched) was_armed=1;
            else if (was_armed){
                sprintf(s, "DISARMED,%s", off_text(off_reason));
                report(s);
                was_armed=0;
            }

            if (off_reason!=0){                                     // the interrupt turned the motors off
                if (off_reason==OFF_SW1) putsU2(" SW1: KILL - motors off, relay open.");
                if (off_reason==OFF_IMU) putsU2(" IMU stopped answering - motors off. Press Calibrate (C) or c.");
                if (off_reason==OFF_TIP) putsU2(" Pitch or roll past 75 deg - motors off, relay open.");
                if (off_reason==OFF_REMOTE) putsU2(" Remote STOP - relay open; ARM required.");
                if (off_reason==OFF_BLE) putsU2(" Heartbeat or link lost - relay open; ARM required.");
                off_reason=0;
            }
            if (run_stopped==1){                                    // step test stopped early
                tuner_set_ble_output(tuner_run_is_phone());
                tuner_reply("RUN_END");
                tuner_set_ble_output(0);
                putsU2(" Step test stopped: the motors were turned off.");
                run_stopped=0;
            }

            // Professor's rule: record every 4 ms sample into arrays, send nothing while
            // recording or while the motors run, and send the arrays after the run.
            if (data_complete==1 && !tuner_run_is_phone()) tuner_send_run();   // to the PC, one line per loop
            else if (data_complete==1 || run_state!=0) ;                       // recording / sending to the phone
            else if (motor_mode!=0){                                           // motors running: quiet
                if (live_on==1 && live_paused==0)
                    putsU2(" Live data paused while the motors run - step tests send all their data when they finish.");
                live_paused=1;
            }
            else{
                live_paused=0;
                // phone: pitch and roll 4 times a second while the motors are off (~16 bytes x 4 = 64 B/s)
                if (ble_connected && last%62==0 && ble_room()>200){
                    sprintf(s, "P,%d,%d,%d", round_i(angle*10), round_i(roll*10), !kill_latched);
                    ble_print(s);
                }
                if (live_on==1 && last%2==0) tuner_send_live();         // PC tuner: every 8 ms (125/s)
                else if (display_on==1 && last%10==0) print_line();     // PuTTY: every 40 ms
            }

            if (noise_waiting==1){
                if (noise_on==0){                                   // the 60 s are up
                    noise_report();
                    noise_waiting=0;
                    display_on=1;
                    lines=0;
                }
                else if (noise_n>0 && noise_n%1250==0){             // every 5 s
                    sprintf(s, " ...noise test: %ld s left, keep still", (15000-noise_n)/250);
                    putsU2(s);
                }
            }

            if (imu_error==1 && err_shown==0){
                putsU2(" IMU stopped answering - motors off. Check the SDO/MISO wire (pin 7), then press c (or Calibrate).");
                LATHbits.LATH1=0;
                err_shown=1;
            }
            if (imu_error==0) err_shown=0;
        }
    }
}
