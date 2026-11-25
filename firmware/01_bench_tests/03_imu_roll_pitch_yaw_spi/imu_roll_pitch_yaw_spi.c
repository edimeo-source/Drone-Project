/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/03_imu_roll_pitch_yaw_spi/imu_roll_pitch_yaw_spi.c
 *  What    : IMU roll / pitch / yaw over SPI (complementary filter)
 *  Version : bench test   |   Date: 2026-11-12
 *  Status  : Builds. Same SPI wiring problem as 02 at the time (CS on pin 3).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1, CS on pin 3 in this era)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 3 (RA3)
 *    ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC red 5 V wires . NOT connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initTimers(), delay_ms(),
 *    escs_off(), initU2(), putU2(), putsU2(), initSPI(), writeSPI(), SPIReadReg(),
 *    SPIWriteReg(), SPIReadAxis(), imu_init(), imu_read(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

/*
 * imu_roll_pitch_yaw_spi.c  -  Drone IMU over SPI: read accelerometer + gyro,
 *                              print Roll / Pitch / Yaw to PuTTY
 * Ethan DiMeo
 *
 * Board : PIC32MZ EF Starter Kit (PIC32MZ2048EFM144), XC32, MPLAB X
 * IMU   : SparkFun LSM6DSO on SPI1 (class SPI code), WHO_AM_I 0x0F = 0x6C
 * UART  : UART2 -> kit USB port (COM10) -> PuTTY at 115200 8-N-1, no flow control
 *
 * WIRING (J12 header)
 *   IMU 3V3 -> pin 1          IMU GND -> pin 6
 *   IMU SCL -> pin 23 (RD1  = SCK1, SPI clock)
 *   IMU SDA -> pin 37 (RD10 = SDO1, data INTO the IMU, MOSI)
 *   IMU SDO -> pin 7  (RA14 = SDI1, data OUT of the IMU, MISO)
 *   IMU CS  -> pin 3  (RA3, chip select, low = selected)
 *   Keep the CS wire away from the SCL (clock) wire.
 *   ESC signals stay on pins 11 and 12: this program holds them at 1 ms (motors off).
 *
 * Output (20 lines per second):
 *   AX AY AZ in g    GX GY GZ in deg/s    ROLL PITCH YAW in degrees    bad reads
 *
 * How the angles are made (complementary filter, 100 Hz):
 *   - Accelerometer gives roll/pitch from gravity: correct long-term, noisy short-term.
 *   - Gyro gives rotation rate: smooth short-term, drifts long-term.
 *   - angle = 0.98 * (angle + gyro*dt) + 0.02 * accel_angle  -> best of both.
 *   - YAW comes only from the gyro (gravity can't see heading), so it slowly drifts.
 *
 * Keep the seesaw flat and still for ~2 seconds after reset while the gyro calibrates.
 */
#include <p32xxxx.h>
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "config.bits"          /* SYSCLK 120 MHz, PBCLK 60 MHz */

/* ---------------- settings ---------------- */
#define IMU_WHO_AM_I_ID 0x6C

#define ACCEL_G_PER_LSB     0.000061f   /* +/-2 g range:     0.061 mg/LSB  */
#define GYRO_DPS_PER_LSB    0.0175f     /* +/-500 deg/s:     17.5 mdps/LSB */
#define ALPHA               0.98f       /* complementary filter: gyro trust */
#define LOOP_MS             10          /* 100 Hz filter loop */
#define PRINT_EVERY         5           /* print every 5th loop = 20 Hz */
#define TICKS_PER_SEC       234375.0f   /* 60 MHz PBCLK / 256 prescale */
#define RAD2DEG             57.2957795f

/* LSM6DSO registers */
#define REG_WHO_AM_I        0x0F
#define REG_CTRL1_XL        0x10        /* accel: data rate + full scale */
#define REG_CTRL2_G         0x11        /* gyro:  data rate + full scale */
#define REG_CTRL3_C         0x12        /* reset, block data update, auto-increment */
#define REG_CTRL4_C         0x13        /* I2C disable (SPI only) */
#define REG_OUTX_L_G        0x22        /* gyro XYZ then accel XYZ, low byte first */
#define SPI_READ            0x80        /* first SPI byte: bit 7 = 1 read, 0 write */

/* ---------------- timers ---------------- */
// ---------------------------------------------------------------------------------------------
// void initTimers(void)
//   Starts the free-running timers: Timer4 for delays and Timer3 as the clock used to measure
//   dt.
// ---------------------------------------------------------------------------------------------
/* Timer2 = ESC pulses, Timer4 = delays, Timer3 = free-running clock for dt */
void initTimers(void)
{
    T4CON = 0x8070; PR4 = 0xFFFF;     /* 1:256 */
    T3CON = 0x8070; PR3 = 0xFFFF;     /* 1:256, wraps every ~280 ms */
}

// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
void delay_ms(int n)
{
    for (int i = 0; i < n; i++) {
        TMR4 = 0;
        while (TMR4 < 234);
        WDTCONbits.WDTCLRKEY = 0x5743;  /* clear the watchdog (config.bits turns it on) */
    }
}

/* ---------------- ESCs held at 1 ms (motors off) ---------------- */
// ---------------------------------------------------------------------------------------------
// void escs_off(void)
//   Starts the ESC outputs and holds them at 1 ms so the motors stay off during the IMU test.
// ---------------------------------------------------------------------------------------------
void escs_off(void)
{
    TRISDbits.TRISD0 = 0;  RPD0R = 12;  /* pin 11 = OC1 */
    TRISFbits.TRISF2 = 0;  RPF2R = 11;  /* pin 12 = OC2 */
    T2CON = 0; TMR2 = 0; PR2 = 18749;   /* 20 ms at 60 MHz / 64 */
    OC1R = 938; OC1RS = 938;            /* 938 ticks = 1 ms */
    OC2R = 938; OC2RS = 938;
    OC1CON = 0x8006; OC2CON = 0x8006;
    T2CON = 0x8060;
}

/* ---------------- UART2 -> PuTTY ---------------- */
// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
void initU2(void)
{
    ANSELGbits.ANSG6 = 0;
    TRISGbits.TRISG6 = 1;
    U2RXR = 1;
    TRISBbits.TRISB14 = 0;
    RPB14R = 2;
    U2BRG = 129;                /* 60 MHz / (4 x 130) = 115200 */
    U2MODE = 0x8008;
    U2STA = 0x1400;
}

// ---------------------------------------------------------------------------------------------
// int putU2(int c)
//   Sends one character out UART2 to the PC (waits while the transmit buffer is full).
// ---------------------------------------------------------------------------------------------
int putU2(int c)
{
    while (U2STAbits.UTXBF);
    U2TXREG = c;
    return c;
}

// ---------------------------------------------------------------------------------------------
// void putsU2(char *s)
//   Sends a text line to the PC over UART2, followed by a new line.
// ---------------------------------------------------------------------------------------------
void putsU2(char *s)
{
    while (*s) putU2(*s++);
    putU2('\r');
    putU2('\n');
}

/* ---------------- SPI1 (class code) ---------------- */
// ---------------------------------------------------------------------------------------------
// void initSPI(void)
//   Sets up SPI1 (pin remaps, mode, clock speed) and the chip-select pin for this version's
//   wiring.
// ---------------------------------------------------------------------------------------------
void initSPI(void)
{
    TRISAbits.TRISA3 = 0;       /* pin 3 = IMU CS */
    LATAbits.LATA3 = 1;         /* CS high = not selected */
    TRISJbits.TRISJ13 = 0;      /* pin 13 = Bluefruit CS (shares SPI1) */
    LATJbits.LATJ13 = 1;        /* keep the Bluefruit off the bus */

    RPD10R = 5;                 /* RD10 = SDO1 (MOSI, pin 37) */
    SDI1R = 13;                 /* RA14 = SDI1 (MISO, pin 7) */
    TRISDbits.TRISD1 = 0;       /* RD1  = SCK1 (pin 23) */

    SPI1CON = 0;
    SPI1BRG = 119;              /* 60 MHz / (2 x 120) = 250 kHz (slow = fewer glitches) */
    SPI1CON = 0x8120;           /* on, master, 8-bit, mode 0 (class labs). Mode 3 = 0x8060 */
}

// ---------------------------------------------------------------------------------------------
// int writeSPI(int i)
//   Sends one byte over SPI1 and returns the byte that came back at the same time.
// ---------------------------------------------------------------------------------------------
int writeSPI(int i)
{
    SPI1BUF = i;
    while (!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}

// ---------------------------------------------------------------------------------------------
// int SPIReadReg(int reg)
//   Reads one LSM6DSO register over SPI.
// ---------------------------------------------------------------------------------------------
int SPIReadReg(int reg)
{
    int v;
    LATAbits.LATA3 = 0;
    writeSPI(SPI_READ | reg);
    v = writeSPI(0);
    LATAbits.LATA3 = 1;
    return v & 0xFF;
}

// ---------------------------------------------------------------------------------------------
// void SPIWriteReg(int reg, int val)
//   Writes one LSM6DSO register over SPI.
// ---------------------------------------------------------------------------------------------
void SPIWriteReg(int reg, int val)
{
    LATAbits.LATA3 = 0;
    writeSPI(reg);
    writeSPI(val);
    LATAbits.LATA3 = 1;
}

// ---------------------------------------------------------------------------------------------
// int16_t SPIReadAxis(int reg)
//   Reads one signed 16-bit axis (low byte + high byte) in its own short CS-low transfer.
// ---------------------------------------------------------------------------------------------
/* one axis = low byte + high byte, in its own short CS-low read */
int16_t SPIReadAxis(int reg)
{
    int lo, hi;
    LATAbits.LATA3 = 0;
    writeSPI(SPI_READ | reg);
    lo = writeSPI(0) & 0xFF;
    hi = writeSPI(0) & 0xFF;
    LATAbits.LATA3 = 1;
    return (int16_t)((hi << 8) | lo);
}

/* ---------------- IMU ---------------- */
// ---------------------------------------------------------------------------------------------
// int imu_init(void)
//   Checks WHO_AM_I, resets the LSM6DSO and writes its settings. 1 = IMU found and set up.
// ---------------------------------------------------------------------------------------------
int imu_init(void)
{
    int id = SPIReadReg(REG_WHO_AM_I);
    if (id != IMU_WHO_AM_I_ID) return 0;           /* wrong answer: wiring */

    SPIWriteReg(REG_CTRL3_C, 0x01);                /* software reset */
    delay_ms(20);
    SPIWriteReg(REG_CTRL4_C, 0x04);                /* I2C off: SPI only */
    SPIWriteReg(REG_CTRL3_C, 0x44);                /* BDU = 1 (no torn readings), IF_INC = 1 */
    SPIWriteReg(REG_CTRL1_XL, 0x40);               /* accel 104 Hz, +/-2 g */
    SPIWriteReg(REG_CTRL2_G,  0x44);               /* gyro  104 Hz, +/-500 deg/s */
    delay_ms(100);                                 /* let the first samples settle */
    return id;
}

unsigned long bad_reads = 0;    /* readings thrown away because they made no sense */

// ---------------------------------------------------------------------------------------------
// int imu_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
//   Reads all 6 axes (gyro + accel) and converts them to g and deg/s. 1 = the reading looks
//   valid.
// ---------------------------------------------------------------------------------------------
/* Reads accel (g) and gyro (deg/s, raw - not bias corrected). Returns 1 on success.
   A reading only counts if the total accel is 0.5 - 1.5 g (it is ~1 g = gravity);
   otherwise it is read again, up to 3 times. */
int imu_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    float a;
    for (int tries = 0; tries < 3; tries++) {
        *gx = SPIReadAxis(REG_OUTX_L_G + 0)  * GYRO_DPS_PER_LSB;
        *gy = SPIReadAxis(REG_OUTX_L_G + 2)  * GYRO_DPS_PER_LSB;
        *gz = SPIReadAxis(REG_OUTX_L_G + 4)  * GYRO_DPS_PER_LSB;
        *ax = SPIReadAxis(REG_OUTX_L_G + 6)  * ACCEL_G_PER_LSB;
        *ay = SPIReadAxis(REG_OUTX_L_G + 8)  * ACCEL_G_PER_LSB;
        *az = SPIReadAxis(REG_OUTX_L_G + 10) * ACCEL_G_PER_LSB;
        a = sqrtf((*ax) * (*ax) + (*ay) * (*ay) + (*az) * (*az));
        if (a > 0.5f && a < 1.5f) return 1;
        bad_reads++;
    }
    return 0;
}

/* ---------------- main ---------------- */
// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: calibrates the gyro, then prints roll, pitch and yaw from the complementary
//   filter.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char s[160];
    float ax, ay, az, gx, gy, gz;
    float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    float roll, pitch, yaw = 0.0f;
    float acc_roll, acc_pitch, dt;
    uint16_t t_now, t_last;
    int id, i, n, count = 0;

    initTimers();
    escs_off();
    initU2();
    initSPI();
    delay_ms(100);

    putsU2("");
    putsU2("=== Drone IMU (SPI): accel + gyro -> roll / pitch / yaw ===");

    /* --- find the IMU --- */
    while ((id = imu_init()) == 0) {
        sprintf(s, "No IMU answer (WHO_AM_I = 0x%02X, want 0x6C) - check CS pin 3, SCL 23, SDA 37, SDO 7. Retrying...",
                SPIReadReg(REG_WHO_AM_I));
        putsU2(s);
        delay_ms(1000);
    }
    sprintf(s, "IMU found over SPI, WHO_AM_I = 0x%02X", id);
    putsU2(s);

    /* --- gyro bias calibration: keep the seesaw still --- */
    putsU2("Calibrating gyro - keep the seesaw flat and still...");
    n = 0;
    for (i = 0; i < 500; i++) {
        if (imu_read(&ax, &ay, &az, &gx, &gy, &gz)) {
            gx_bias += gx; gy_bias += gy; gz_bias += gz;
            n++;
        }
        delay_ms(3);
    }
    if (n > 0) { gx_bias /= n; gy_bias /= n; gz_bias /= n; }
    sprintf(s, "Gyro bias (deg/s): X %.3f  Y %.3f  Z %.3f   (%d good of 500)", gx_bias, gy_bias, gz_bias, n);
    putsU2(s);

    /* --- start angles from gravity so the filter doesn't have to swing in --- */
    while (!imu_read(&ax, &ay, &az, &gx, &gy, &gz));
    roll  = atan2f(ay, az) * RAD2DEG;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

    putsU2("");
    putsU2("   AX(g)   AY(g)   AZ(g) |  GX(d/s)  GY(d/s)  GZ(d/s) |    ROLL   PITCH     YAW | bad");
    putsU2("-------------------------+----------------------------+-------------------------+----");

    t_last = TMR3;
    while (1) {
        if (!imu_read(&ax, &ay, &az, &gx, &gy, &gz)) {
            delay_ms(LOOP_MS);          /* 3 bad reads in a row: skip this loop */
            continue;
        }

        /* time since last loop (16-bit subtraction handles timer wrap) */
        t_now  = TMR3;
        dt     = (uint16_t)(t_now - t_last) / TICKS_PER_SEC;
        t_last = t_now;

        /* remove gyro bias */
        gx -= gx_bias;
        gy -= gy_bias;
        gz -= gz_bias;

        /* angles from gravity */
        acc_roll  = atan2f(ay, az) * RAD2DEG;
        acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

        /* complementary filter */
        roll  = ALPHA * (roll  + gx * dt) + (1.0f - ALPHA) * acc_roll;
        pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * acc_pitch;
        yaw  += gz * dt;                                  /* gyro only, drifts */
        if (yaw >  180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;

        if (++count >= PRINT_EVERY) {
            count = 0;
            sprintf(s, "%7.2f %7.2f %7.2f | %8.2f %8.2f %8.2f | %7.2f %7.2f %7.2f | %lu",
                    ax, ay, az, gx, gy, gz, roll, pitch, yaw, bad_reads);
            putsU2(s);
        }

        delay_ms(LOOP_MS);
    }
    return 0;
}
