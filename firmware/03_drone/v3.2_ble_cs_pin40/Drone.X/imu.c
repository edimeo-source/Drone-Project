/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.2_ble_cs_pin40/Drone.X/imu.c
 *  What    : LSM6DSO IMU driver over SPI1 and the complementary filter (roll, pitch, yaw).
 *  Version : v3.2   |   Date: 2026-11-26
 *  Status  : CURRENT. v3.1 with the Bluefruit CS moved from pin 13 (RJ13) to pin 40 (RJ3);
 *            nothing else changed. Host tests pass (87 checks; the Bluetooth simulation checks
 *            CS on RJ3 for every byte). Not yet built in MPLAB.
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
 *      SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 40 (RJ3, moved from pin 13 in v3.2)
 *      IRQ and RST not connected
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initSPI(), spi_xfer(),
 *    imu_write(), imu_read(), imu_read1(), imu_who_am_i(), write_check(), initIMU(),
 *    imu_read_sample(), imu_calibrate(), imu_reset_angles(), imu_update()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <math.h>
# include "imu.h"
# include "uart.h"
# include "delay.h"

// LSM6DSO 6-axis IMU (3-axis gyroscope + 3-axis accelerometer) on SPI1.
// Wiring, IMU board -> J12:
//   3V3 -> pin 1     GND -> pin 6
//   SCL -> pin 23    (RD1  = SCK1, the clock)              \  each one is a splitter wire
//   SDA -> pin 37    (RD10 = SDO1, data PIC -> IMU)         >  shared with the Bluefruit
//   SDO -> pin 7     (RA14 = SDI1, data IMU -> PIC)        /   (this contact has been flaky before)
//   CS  -> pin 16    (RK1, low = talking to the IMU; the Bluefruit has its own CS on pin 40)
// Only one CS is ever low, and an unselected chip lets go of MISO, so the two can share it.

// ---- LSM6DSO registers (datasheet DS12140, section 9) ----
#define WHO_AM_I    0x0F    // always reads 0x6C on an LSM6DSO
#define CTRL1_XL    0x10    // accelerometer speed, range, filter
#define CTRL2_G     0x11    // gyro speed, range
#define CTRL3_C     0x12    // reset, BDU, address auto-increment
#define CTRL4_C     0x13    // I2C off, gyro filter on
#define CTRL6_C     0x15    // gyro filter cutoff
#define CTRL8_XL    0x17    // accelerometer filter cutoff
#define CTRL9_XL    0x18    // I3C off
#define OUT_TEMP_L  0x20    // 0x20..0x2D: temperature, gyro X Y Z, accel X Y Z (14 bytes)

// ---- the settings we write ----
#define CTRL1_XL_VAL 0x7A   // accel 833 Hz, +/-4 g, second low-pass filter (LPF2) on
#define CTRL2_G_VAL  0x74   // gyro 833 Hz, +/-500 deg/s
#define CTRL3_C_VAL  0x44   // BDU (both bytes from the same sample) + address auto-increment
#define CTRL4_C_VAL  0x06   // I2C off (SPI only) + gyro low-pass filter LPF1 on
#define CTRL6_C_VAL  0x05   // gyro LPF1 cutoff 49 Hz at 833 Hz (Table 60, FTYPE = 101)
#define CTRL8_XL_VAL 0x60   // accel LPF2 cutoff 833/45 = 18.5 Hz (Table 65, HPCF_XL = 011)
#define CTRL9_XL_VAL 0xE2   // I3C off

#define GYRO_DPS_PER_LSB 0.0175f    // +/-500 deg/s range: 17.5 millidegrees/s per count
#define ACC_G_PER_LSB    0.000122f  // +/-4 g range: 0.122 milli-g per count
#define RAD_TO_DEG       57.29578f
#define DT               0.004f     // the loop runs every 4 ms
#define ALPHA            0.98f      // complementary filter: 98% gyro, 2% gravity

#define CAL_SAMPLES 500     // 500 x 4 ms = 2 s
#define GYRO_STILL  2.0f    // deg/s: a gyro axis that moves more than this (max - min) = not still
#define ACC_STILL   0.05f   // g: same idea for the accelerometer

volatile float roll=0, pitch=0, yaw=0;
volatile float roll_rate=0, pitch_rate=0, yaw_rate=0;
volatile float ax=0, ay=0, az=0;
volatile float temp_c=0;
volatile int imu_error=0;

float gx=0, gy=0, gz=0;                 // gyro this sample, deg/s (offsets removed)
float gx_off=0, gy_off=0, gz_off=0;     // gyro offsets from calibration
float ax_off=0, ay_off=0;               // accel offsets from calibration (level reads 0)
int check_count=0;                      // counts loops between WHO_AM_I checks

// ---------------- SPI1 ----------------
// ---------------------------------------------------------------------------------------------
// void initSPI(void)
//   Sets up SPI1 (pin remaps, mode, clock speed) and the chip-select pin for this version's
//   wiring.
// ---------------------------------------------------------------------------------------------
void initSPI(void)
{
    TRISKbits.TRISK1=0;     // RK1 = CS (pin 16)
    LATKbits.LATK1=1;       // CS high = IMU not selected

    RPD10R=5;               // SDO1 -> RD10 (pin 37)
    SDI1R=13;               // SDI1 <- RA14 (pin 7)
    TRISDbits.TRISD10=0;
    TRISAbits.TRISA14=1;
    LATDbits.LATD1=1;       // SCK1 = RD1 (pin 23), clock idles high
    TRISDbits.TRISD1=0;

    SPI1CON=0;
    SPI1BRG=14;             // 60 MHz / (2 x 15) = 2 MHz (the IMU allows up to 10 MHz)
    SPI1CON=0x8060;         // ON, master, 8-bit, clock idles high (CKP = 1, CKE = 0) = SPI mode 3
}

// ---------------------------------------------------------------------------------------------
// int spi_xfer(int b)
//   One SPI1 byte out, one byte in, with a 50 us timeout that sets imu_error instead of
//   hanging.
// ---------------------------------------------------------------------------------------------
int spi_xfer(int b)         // send one byte, get one byte back at the same time
{
    unsigned int start=(unsigned int)_CP0_GET_COUNT();
    SPI1BUF=b;
    while (!SPI1STATbits.SPIRBF){
        if ((unsigned int)(_CP0_GET_COUNT()-start)>3000u){ // 50 us, not an endless ISR wait
            imu_error=1;
            return 0;
        }
    }
    return SPI1BUF;
}

// ---------------------------------------------------------------------------------------------
// void imu_write(int reg, int val)
//   Writes one LSM6DSO register over SPI (CS low, address with bit 7 = 0, value, CS high).
// ---------------------------------------------------------------------------------------------
void imu_write(int reg, int val)
{
    LATKbits.LATK1=0;       // CS low: start
    spi_xfer(reg);          // address, bit 7 = 0 means write
    spi_xfer(val);
    LATKbits.LATK1=1;       // CS high: done
}

// ---------------------------------------------------------------------------------------------
// void imu_read(int reg, unsigned char *buf, int n)
//   Reads n LSM6DSO registers in a row over SPI (bit 7 = 1 = read; the address
//   auto-increments).
// ---------------------------------------------------------------------------------------------
void imu_read(int reg, unsigned char *buf, int n)
{
    LATKbits.LATK1=0;
    spi_xfer(reg|0x80);     // address, bit 7 = 1 means read
    for (int i=0;i<n;i++){
        buf[i]=spi_xfer(0); // the address steps up by itself (auto-increment)
    }
    LATKbits.LATK1=1;
}

// ---------------------------------------------------------------------------------------------
// int imu_read1(int reg)
//   Reads one LSM6DSO register and returns its value.
// ---------------------------------------------------------------------------------------------
int imu_read1(int reg)
{
    unsigned char v;
    imu_read(reg, &v, 1);
    return v;
}

// ---------------------------------------------------------------------------------------------
// int imu_who_am_i(void)
//   Reads the WHO_AM_I register (an LSM6DSO always answers 0x6C).
// ---------------------------------------------------------------------------------------------
int imu_who_am_i(void)
{
    return imu_read1(WHO_AM_I);
}

// ---------------- settings ----------------
// ---------------------------------------------------------------------------------------------
// int write_check(int reg, int val, char *name)
//   Writes one IMU register, reads it back and prints OK or MISMATCH. 0 = OK, 1 = mismatch.
// ---------------------------------------------------------------------------------------------
// write one register, read it back, print OK or MISMATCH. Returns 0 if OK, 1 if not.
int write_check(int reg, int val, char *name)
{
    char s[80];
    int back;

    imu_write(reg, val);
    back=imu_read1(reg);
    if (back==val){
        sprintf(s, "  %-9s (0x%02X) <- 0x%02X  read back 0x%02X  OK", name, reg, val, back);
        putsU2(s);
        return 0;
    }
    sprintf(s, "  %-9s (0x%02X) <- 0x%02X  read back 0x%02X  MISMATCH", name, reg, val, back);
    putsU2(s);
    return 1;
}

// ---------------------------------------------------------------------------------------------
// int initIMU(void)
//   Resets the LSM6DSO and writes every setting, reading each one back. Returns how many did
//   not stick (0 = all good).
// ---------------------------------------------------------------------------------------------
int initIMU(void)
{
    int bad=0;

    imu_write(CTRL3_C, 0x01);           // software reset: every setting back to power-on values
    delay_ms(10);

    bad=bad+write_check(CTRL3_C,  CTRL3_C_VAL,  "CTRL3_C");
    bad=bad+write_check(CTRL4_C,  CTRL4_C_VAL,  "CTRL4_C");
    bad=bad+write_check(CTRL9_XL, CTRL9_XL_VAL, "CTRL9_XL");
    bad=bad+write_check(CTRL6_C,  CTRL6_C_VAL,  "CTRL6_C");
    bad=bad+write_check(CTRL8_XL, CTRL8_XL_VAL, "CTRL8_XL");
    bad=bad+write_check(CTRL1_XL, CTRL1_XL_VAL, "CTRL1_XL");
    bad=bad+write_check(CTRL2_G,  CTRL2_G_VAL,  "CTRL2_G");

    delay_ms(100);                      // let the sensors and filters settle
    return bad;
}

// ---------------- reading ----------------
// ---------------------------------------------------------------------------------------------
// void imu_read_sample(void)
//   Reads one 14-byte sample (temperature, gyro XYZ, accel XYZ) and converts it to deg C, deg/s
//   and g.
// ---------------------------------------------------------------------------------------------
// One sample = 14 bytes read in one go from 0x20: temperature, gyro X Y Z, accel X Y Z.
// Each value is 2 bytes, low byte first, signed (short = 16-bit signed).
void imu_read_sample(void)
{
    unsigned char b[14];
    short raw;

    imu_read(OUT_TEMP_L, b, 14);

    raw=(short)((b[1]<<8)|b[0]);    temp_c=25.0f+raw/256.0f;     // 256 counts per C, 0 = 25 C
    raw=(short)((b[3]<<8)|b[2]);    gx=raw*GYRO_DPS_PER_LSB-gx_off;
    raw=(short)((b[5]<<8)|b[4]);    gy=raw*GYRO_DPS_PER_LSB-gy_off;
    raw=(short)((b[7]<<8)|b[6]);    gz=raw*GYRO_DPS_PER_LSB-gz_off;
    raw=(short)((b[9]<<8)|b[8]);    ax=raw*ACC_G_PER_LSB-ax_off;
    raw=(short)((b[11]<<8)|b[10]);  ay=raw*ACC_G_PER_LSB-ay_off;
    raw=(short)((b[13]<<8)|b[12]);  az=raw*ACC_G_PER_LSB;
}

// ---------------- calibration ----------------
// ---------------------------------------------------------------------------------------------
// int imu_calibrate(void)
//   Averages the IMU while the drone sits still and level to find the gyro and accel offsets
//   (retries if it moved).
// ---------------------------------------------------------------------------------------------
// Average 2 s of readings while the board sits still and level:
//   gyro offsets   -> a still gyro reads 0
//   accel X, Y     -> "level" reads roll 0, pitch 0 (Z keeps its 1 g)
// If anything moved during the 2 s, it throws the numbers away and tries again (5 tries).
int imu_calibrate(void)
{
    char s[120], t1[16], t2[16], t3[16], t4[16], t5[16];
    float v[5], sum[5], lo[5], hi[5];
    float old[5];
    int i, k, tries;

    if (imu_who_am_i()!=0x6C){
        putsU2("  Calibration skipped: the IMU isn't answering. Check the SDO wire (pin 7).");
        imu_error=1;
        return 0;
    }
    imu_error=0;

    old[0]=gx_off; old[1]=gy_off; old[2]=gz_off; old[3]=ax_off; old[4]=ay_off;

    for (tries=1;tries<=5;tries++){
        gx_off=0; gy_off=0; gz_off=0; ax_off=0; ay_off=0;   // measure the raw values
        for (k=0;k<5;k++){
            sum[k]=0;
            lo[k]=1000000;
            hi[k]=-1000000;
        }

        for (i=0;i<CAL_SAMPLES;i++){
            imu_read_sample();
            v[0]=gx; v[1]=gy; v[2]=gz; v[3]=ax; v[4]=ay;
            for (k=0;k<5;k++){
                sum[k]=sum[k]+v[k];
                if (v[k]<lo[k]) lo[k]=v[k];
                if (v[k]>hi[k]) hi[k]=v[k];
            }
            delay_ms(4);
        }

        // still = every gyro axis moved less than GYRO_STILL and every accel axis less than ACC_STILL
        if (hi[0]-lo[0]<GYRO_STILL && hi[1]-lo[1]<GYRO_STILL && hi[2]-lo[2]<GYRO_STILL &&
            hi[3]-lo[3]<ACC_STILL && hi[4]-lo[4]<ACC_STILL){
            gx_off=sum[0]/CAL_SAMPLES;
            gy_off=sum[1]/CAL_SAMPLES;
            gz_off=sum[2]/CAL_SAMPLES;
            ax_off=sum[3]/CAL_SAMPLES;
            ay_off=sum[4]/CAL_SAMPLES;
            sprintf(s, "  Calibrated. Gyro offsets X %s  Y %s  Z %s deg/s, level offsets X %s  Y %s g",
                    fstr(t1, gx_off, 2), fstr(t2, gy_off, 2), fstr(t3, gz_off, 2),
                    fstr(t4, ax_off, 3), fstr(t5, ay_off, 3));
            putsU2(s);
            return 1;
        }
        sprintf(s, "  It moved (try %d of 5) - hold it still...", tries);
        putsU2(s);
    }

    gx_off=old[0]; gy_off=old[1]; gz_off=old[2]; ax_off=old[3]; ay_off=old[4];
    putsU2("  Never held still - kept the old offsets. Press c to try again.");
    return 0;
}

// ---------------------------------------------------------------------------------------------
// void imu_reset_angles(void)
//   Starts roll and pitch from the accelerometer (gravity) and sets yaw to 0.
// ---------------------------------------------------------------------------------------------
void imu_reset_angles(void)
{
    imu_read_sample();
    roll=atan2f(ay, az)*RAD_TO_DEG;
    pitch=atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;
    yaw=0;
}

// ---------------- every 4 ms (from the control interrupt) ----------------
// ---------------------------------------------------------------------------------------------
// void imu_update(void)
//   Runs every 4 ms: reads the IMU and updates roll, pitch, yaw with the complementary filter
//   (alpha 0.98).
// ---------------------------------------------------------------------------------------------
void imu_update(void)
{
    float roll_acc, pitch_acc;

    imu_read_sample();

    // rates in drone terms (X arrow = front, Z = up)
    roll_rate=gx;           // + = left side going up
    pitch_rate=-gy;         // the gyro's Y is + when the nose goes DOWN, so flip it
    yaw_rate=gz;            // + = nose turning left (counter-clockwise seen from above)

    // angles from gravity alone (only right when not accelerating, so they get just 2%)
    roll_acc=atan2f(ay, az)*RAD_TO_DEG;
    pitch_acc=atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;

    // complementary filter: 98% (last angle + rate x dt) for smooth and fast,
    //                        2% gravity so it can never drift away
    roll=ALPHA*(roll+roll_rate*DT)+(1.0f-ALPHA)*roll_acc;
    pitch=ALPHA*(pitch+pitch_rate*DT)+(1.0f-ALPHA)*pitch_acc;

    yaw=yaw+yaw_rate*DT;    // no gravity reference for yaw, so it drifts slowly (z zeros it)
    if (yaw>180.0f) yaw=yaw-360.0f;
    if (yaw<-180.0f) yaw=yaw+360.0f;

    // once a second, make sure the IMU still answers (the SDO wire on pin 7 has been flaky)
    check_count++;
    if (check_count>=250){
        check_count=0;
        if (imu_who_am_i()!=0x6C) imu_error=1;
    }
}
