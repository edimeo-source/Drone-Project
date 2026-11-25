/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/Drone.X/noise.c
 *  What    : 60 s IMU noise test (RMS noise, jitter, drift) with PASS/FAIL.
 *  Version : v3.0   |   Date: 2026-11-25
 *  Status  : CURRENT. Built from v2.0 (which builds and flew on the tether). Host tests pass
 *            (78 checks, including a simulated flight with the real 4 ms loop). Not yet built
 *            in MPLAB or flown.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): noise_start(), noise_sample(),
 *    rms(), noise_report()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <math.h>
# include "noise.h"
# include "imu.h"
# include "control.h"
# include "uart.h"

// 60 s noise test (PuTTY key n). The board must sit completely still.
// The 4 ms interrupt calls noise_sample(); main() prints noise_report() at the end.
// Pass marks (PINOUT_AND_PLAN.md, Step 1):
//   loop period    3990 .. 4010 us
//   gyro noise     <= 0.10 deg/s RMS on each axis
//   angle jitter   <= +/-0.10 deg (roll and pitch)
//   angle drift    <= 0.20 deg in 60 s (roll and pitch)

#define NOISE_SAMPLES 15000     // 60 s x 250 loops per second

volatile int noise_on=0;
volatile long noise_n=0;

double sum_r, sq_r, sum_p, sq_p, sum_y, sq_y;   // sums of the rates, and of the rates squared
float roll_lo, roll_hi, pitch_lo, pitch_hi;     // lowest and highest angle seen
float roll0, pitch0, yaw0;                      // angles at the start
float roll1, pitch1, yaw1;                      // angles at the end
float per_lo, per_hi, busy_hi;                  // loop timing, us

// ---------------------------------------------------------------------------------------------
// void noise_start(void)
//   Starts the 60 s noise test (the 4 ms loop begins collecting samples).
// ---------------------------------------------------------------------------------------------
void noise_start(void)
{
    noise_n=0;
    noise_on=1;                 // the interrupt starts counting on its next loop
}

// ---------------------------------------------------------------------------------------------
// void noise_sample(void)
//   Runs every 4 ms during the noise test: adds this sample to the sums and min/max.
// ---------------------------------------------------------------------------------------------
void noise_sample(void)
{
    if (noise_on==0) return;

    if (noise_n==0){            // first sample: start every number from here
        sum_r=0; sq_r=0; sum_p=0; sq_p=0; sum_y=0; sq_y=0;
        roll_lo=roll;   roll_hi=roll;
        pitch_lo=pitch; pitch_hi=pitch;
        roll0=roll; pitch0=pitch; yaw0=yaw;
        per_lo=loop_period_us; per_hi=loop_period_us;
        busy_hi=loop_busy_us;
    }

    sum_r=sum_r+roll_rate;   sq_r=sq_r+roll_rate*roll_rate;
    sum_p=sum_p+pitch_rate;  sq_p=sq_p+pitch_rate*pitch_rate;
    sum_y=sum_y+yaw_rate;    sq_y=sq_y+yaw_rate*yaw_rate;

    if (roll<roll_lo) roll_lo=roll;
    if (roll>roll_hi) roll_hi=roll;
    if (pitch<pitch_lo) pitch_lo=pitch;
    if (pitch>pitch_hi) pitch_hi=pitch;

    if (loop_period_us<per_lo) per_lo=loop_period_us;
    if (loop_period_us>per_hi) per_hi=loop_period_us;
    if (loop_busy_us>busy_hi) busy_hi=loop_busy_us;

    roll1=roll; pitch1=pitch; yaw1=yaw;     // the last sample ends up here

    noise_n++;
    if (noise_n>=NOISE_SAMPLES) noise_on=0; // done: main() prints the report
}

// ---------------------------------------------------------------------------------------------
// float rms(double sum, double sq, long n)
//   Standard deviation from a sum, a sum of squares and a count (RMS noise of a rate).
// ---------------------------------------------------------------------------------------------
// RMS noise of a rate = its standard deviation = sqrt(average of squares - square of average)
float rms(double sum, double sq, long n)
{
    double mean=sum/n;
    double var=sq/n-mean*mean;
    if (var<0) var=0;
    return (float)sqrt(var);
}

// ---------------------------------------------------------------------------------------------
// void noise_report(void)
//   Prints the noise test results (RMS noise, jitter, drift) with PASS/FAIL for each.
// ---------------------------------------------------------------------------------------------
void noise_report(void)
{
    char s[140], t1[16], t2[16], t3[16];
    float nr, np, ny, jr, jp, dr, dp, dy;
    int ok, pass=1;

    nr=rms(sum_r, sq_r, noise_n);
    np=rms(sum_p, sq_p, noise_n);
    ny=rms(sum_y, sq_y, noise_n);
    jr=(roll_hi-roll_lo)/2;             // jitter = half the spread, shown as +/-
    jp=(pitch_hi-pitch_lo)/2;
    dr=roll1-roll0;                     // drift = end - start
    dp=pitch1-pitch0;
    dy=yaw1-yaw0;

    putsU2("");
    putsU2("======================= NOISE TEST (60 s, still) =======================");

    ok=(per_lo>=3990 && per_hi<=4010);
    sprintf(s, " loop period : %s to %s us        (want 3990..4010)     %s",
            fstr(t1, per_lo, 1), fstr(t2, per_hi, 1), ok ? "PASS" : "FAIL");
    putsU2(s);
    if (!ok) pass=0;

    sprintf(s, " loop busy   : %s us of 4000 (how long one loop takes)", fstr(t1, busy_hi, 1));
    putsU2(s);

    ok=(nr<=0.10f && np<=0.10f && ny<=0.10f);
    sprintf(s, " gyro noise  : roll %s  pitch %s  yaw %s deg/s RMS (want <= 0.100)  %s",
            fstr(t1, nr, 3), fstr(t2, np, 3), fstr(t3, ny, 3), ok ? "PASS" : "FAIL");
    putsU2(s);
    if (!ok) pass=0;

    ok=(jr<=0.10f && jp<=0.10f);
    sprintf(s, " angle jitter: roll %s  pitch %s deg (+/-)  (want <= 0.10)       %s",
            fstr(t1, jr, 2), fstr(t2, jp, 2), ok ? "PASS" : "FAIL");
    putsU2(s);
    if (!ok) pass=0;

    ok=(fabsf(dr)<=0.20f && fabsf(dp)<=0.20f);
    sprintf(s, " angle drift : roll %s  pitch %s deg in 60 s (want <= 0.20)       %s",
            fstr(t1, dr, 2), fstr(t2, dp, 2), ok ? "PASS" : "FAIL");
    putsU2(s);
    if (!ok) pass=0;

    sprintf(s, " yaw drift   : %s deg in 60 s (info only: yaw has no gravity reference, so it creeps)",
            fstr(t1, dy, 2));
    putsU2(s);

    if (pass) putsU2(" RESULT: PASS - the gyro data is clean enough for the PID.");
    else      putsU2(" RESULT: FAIL - see the FAIL lines. Was the table bumped? Run it again (n).");
    putsU2("=========================================================================");
}
