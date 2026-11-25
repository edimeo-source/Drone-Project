/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/02_imu_spi_test/imu_spi_test.c
 *  What    : Basic IMU test over SPI (WHO_AM_I, accel, gyro, roll/pitch)
 *  Version : bench test   |   Date: 2026-11-12
 *  Status  : Builds. SPI reads were unreliable on this wiring (CS on pin 3); fixed later by
 *            moving CS to pin 16.
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): delay_ms(), initU2(), putU2(),
 *    putsU2(), escs_off(), initSPI(), setSPI(), writeSPI(), imu_read_reg(), imu_write_reg(),
 *    imu_read_bytes(), read_axis(), imu_read_all(), imu_setup(), main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <stdio.h>
# include <math.h>
# include "config.bits"     // SYSCLK 120 MHz, PBCLK 60 MHz

// =====================================================================
// BASIC IMU TEST over SPI - SparkFun LSM6DSO on SPI1 (class SPI code, Lab 4/5)
// Ethan DiMeo
//
// IMU header pin (SPI job)        -> J12 pin
//   3V3                           -> pin 1
//   GND                           -> pin 6
//   SCL  (SPC = SPI clock)        -> pin 23 (RD1  = SCK1)
//   SDA  (SDI = data INTO the IMU)-> pin 37 (RD10 = SDO1, MOSI)
//   SDO  (data OUT of the IMU)    -> pin 7  (RA14 = SDI1, MISO)
//   CS   (chip select, active low)-> pin 3  (RA3, same CS pin as the BME680 lab)
// The ADR jumper is cut (your professor's cut) so SDO is free to drive MISO.
//
// PuTTY: COM10, 115200 (close the PID tuner first)
// Prints WHO_AM_I (should be 0x6C), then 5 times a second:
//   accel (g), gyro (deg/s), roll and pitch (deg)
// The ESCs on pins 11 and 12 get a steady 1 ms (motors off).
// =====================================================================

#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12
#define CTRL4_C     0x13
#define OUTX_L_G    0x22        // 12 bytes: gyro X,Y,Z then accel X,Y,Z
#define SPI_READ    0x80        // first byte: bit 7 = 1 means read, 0 means write
#define ACCEL_SCALE 0.000122f   // g per count at +/-4 g
#define GYRO_SCALE  0.0175f     // deg/s per count at +/-500 dps
#define RAD_TO_DEG  57.29578f

// ---------------- delays: Timer3, 234 counts = 1 ms ----------------
// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
void delay_ms(int n)
{
    for (int i=0;i<n;i++){
        TMR3=0;
        while(TMR3<234);
        WDTCONbits.WDTCLRKEY=0x5743;    // clear the watchdog
    }
}

// ---------------- UART2 -> PuTTY ----------------
// ---------------------------------------------------------------------------------------------
// void initU2(void)
//   Sets up UART2 (RG6 = RX, RB14 = TX) at 115200 baud for PuTTY / the PC tuner.
// ---------------------------------------------------------------------------------------------
void initU2(void)
{
    ANSELGbits.ANSG6=0;
    TRISGbits.TRISG6=1;
    U2RXR=1;
    TRISBbits.TRISB14=0;
    RPB14R=2;
    U2BRG=129;              // 115200 at 60 MHz
    U2MODE=0x8008;
    U2STA=0x1400;
}

// ---------------------------------------------------------------------------------------------
// void putU2(int c)
//   Sends one character out UART2 to the PC (waits while the transmit buffer is full).
// ---------------------------------------------------------------------------------------------
void putU2(int c)
{
    while (U2STAbits.UTXBF);
    U2TXREG=c;
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

// ---------------- ESCs held at 1 ms (off) ----------------
// ---------------------------------------------------------------------------------------------
// void escs_off(void)
//   Starts the ESC outputs and holds them at 1 ms so the motors stay off during the IMU test.
// ---------------------------------------------------------------------------------------------
void escs_off(void)
{
    TRISDbits.TRISD0=0;  RPD0R=12;      // pin 11 = OC1
    TRISFbits.TRISF2=0;  RPF2R=11;      // pin 12 = OC2
    T2CON=0; TMR2=0; PR2=18749;         // 20 ms at 60 MHz / 64
    OC1R=938; OC1RS=938;                // 1 ms
    OC2R=938; OC2RS=938;
    OC1CON=0x8006; OC2CON=0x8006;
    T2CON=0x8060;
}

// ---------------- SPI1 (class code) ----------------
// ---------------------------------------------------------------------------------------------
// void initSPI(void)
//   Sets up SPI1 (pin remaps, mode, clock speed) and the chip-select pin for this version's
//   wiring.
// ---------------------------------------------------------------------------------------------
void initSPI(void)
{
    TRISAbits.TRISA3=0;     // pin 3 = IMU CS
    LATAbits.LATA3=1;       // CS high = IMU not selected
    TRISJbits.TRISJ13=0;    // pin 13 = Bluefruit CS (shares SPI1)
    LATJbits.LATJ13=1;      // keep the Bluefruit OFF the bus so it can't fight MISO

    RPD10R=5;               // RD10 = SDO1 (MOSI, pin 37)
    SDI1R=13;               // RA14 = SDI1 (MISO, pin 7)
    TRISDbits.TRISD1=0;     // RD1  = SCK1 (pin 23)

    SPI1CON=0x8120;         // SPI on, master, 8-bit, CKE=1 CKP=0 (mode 0), same as the labs
    SPI1BRG=29;             // 60 MHz / (2 x (29+1)) = 1 MHz
}

// ---------------------------------------------------------------------------------------------
// void setSPI(int con, int brg)
//   Restarts SPI1 with a given SPI1CON (mode) and SPI1BRG (speed) value.
// ---------------------------------------------------------------------------------------------
// change the SPI mode and speed (the module must be off while CON changes)
//   mode 0: SPI1CON = 0x8120 (CKE=1, CKP=0, clock idles low)   - the class labs
//   mode 3: SPI1CON = 0x8060 (CKE=0, CKP=1, clock idles high)  - ST datasheet drawing
//   speed:  60 MHz / (2 x (BRG+1))   BRG 29 = 1 MHz, BRG 119 = 250 kHz
void setSPI(int con, int brg)
{
    SPI1CON=0;
    SPI1BRG=brg;
    SPI1CON=con;
}

// ---------------------------------------------------------------------------------------------
// int writeSPI(int i)
//   Sends one byte over SPI1 and returns the byte that came back at the same time.
// ---------------------------------------------------------------------------------------------
int writeSPI(int i)         // send one byte, get one byte back
{
    SPI1BUF=i;
    while (!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}

// ---------------------------------------------------------------------------------------------
// int imu_read_reg(int reg)
//   Reads one LSM6DSO register over SPI (CS on pin 3 in this test).
// ---------------------------------------------------------------------------------------------
// LSM6DSO over SPI: CS low, send the register number, then the data, CS high
int imu_read_reg(int reg)
{
    int v;
    LATAbits.LATA3=0;
    writeSPI(SPI_READ | reg);
    v=writeSPI(0);          // dummy byte out, register value in
    LATAbits.LATA3=1;
    return v & 0xFF;
}

// ---------------------------------------------------------------------------------------------
// void imu_write_reg(int reg, int val)
//   Writes one LSM6DSO register over SPI (CS on pin 3 in this test).
// ---------------------------------------------------------------------------------------------
void imu_write_reg(int reg, int val)
{
    LATAbits.LATA3=0;
    writeSPI(reg);          // bit 7 = 0 -> write
    writeSPI(val);
    LATAbits.LATA3=1;
}

// ---------------------------------------------------------------------------------------------
// void imu_read_bytes(int reg, int b[], int n)
//   Reads n LSM6DSO registers in a row over SPI into b[].
// ---------------------------------------------------------------------------------------------
void imu_read_bytes(int reg, int b[], int n)   // n registers in a row (auto-increment)
{
    LATAbits.LATA3=0;
    writeSPI(SPI_READ | reg);
    for (int i=0; i<n; i++) b[i]=writeSPI(0) & 0xFF;
    LATAbits.LATA3=1;
}

// ---------------------------------------------------------------------------------------------
// int read_axis(int reg)
//   Reads one 16-bit axis (low byte + high byte) in its own short CS-low transfer.
// ---------------------------------------------------------------------------------------------
// One axis = 2 registers (low byte, high byte) in its own short CS-low read.
// Short reads mean a glitch can only spoil one axis, not everything after it.
int read_axis(int reg)
{
    int lo, hi;
    LATAbits.LATA3=0;
    writeSPI(SPI_READ | reg);
    lo=writeSPI(0) & 0xFF;
    hi=writeSPI(0) & 0xFF;
    LATAbits.LATA3=1;
    return (short)((hi<<8) | lo);   // (short) makes it signed
}

// ---------------------------------------------------------------------------------------------
// int imu_read_all(int r[6])
//   Reads gyro X,Y,Z and accel X,Y,Z as raw counts. 1 = the total acceleration is near 1 g
//   (sane reading).
// ---------------------------------------------------------------------------------------------
// r[0..2] = gyro X,Y,Z   r[3..5] = accel X,Y,Z   (raw counts)
// returns 1 if the reading makes sense: total accel between 0.5 g and 1.5 g
// (sitting still or moving slowly the total is always about 1 g = gravity)
int imu_read_all(int r[6])
{
    float a;
    for (int i=0; i<6; i++) r[i]=read_axis(OUTX_L_G+2*i);
    a=sqrtf((float)r[3]*r[3]+(float)r[4]*r[4]+(float)r[5]*r[5])*ACCEL_SCALE;
    return (a>0.5f && a<1.5f);
}

// ---------------------------------------------------------------------------------------------
// void imu_setup(void)
//   Writes the LSM6DSO settings: SPI only, block data update, 833 Hz, +/-4 g, +/-500 dps.
// ---------------------------------------------------------------------------------------------
void imu_setup(void)
{
    imu_write_reg(CTRL4_C, 0x04);   // I2C off: SPI only (stops the chip mistaking SPI for I2C)
    imu_write_reg(CTRL3_C, 0x44);   // BDU + address auto-increment
    imu_write_reg(CTRL1_XL, 0x78);  // accel 833 Hz, +/-4 g
    imu_write_reg(CTRL2_G, 0x74);   // gyro 833 Hz, +/-500 dps
    delay_ms(50);
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: checks WHO_AM_I, sets up the IMU, then prints accel, gyro, roll and pitch 5
//   times a second.
// ---------------------------------------------------------------------------------------------
// =====================================================================
int main(void)
{
    char s[120];
    int id, r[6], tries;
    unsigned long reads=0, bad=0;
    float ax, ay, az, gx, gy, gz, roll, pitch;

    T3CON=0x8070;               // Timer3 for delays
    escs_off();
    initU2();
    initSPI();
    delay_ms(100);

    putsU2("");
    putsU2("=== BASIC IMU TEST (SPI) ===");

    // try every SPI mode/speed: 20 WHO_AM_I reads + 20 full readings each.
    // A full reading counts as good when the total accel is about 1 g. KEEP THE SEESAW STILL.
    int con[4]  = {0x8120, 0x8060, 0x8120, 0x8060};
    int brg[4]  = {29,     29,     119,    119};
    char *name[4] = {"mode 0, 1 MHz  ", "mode 3, 1 MHz  ", "mode 0, 250 kHz", "mode 3, 250 kHz"};
    int best=0, best_ok=-1;
    putsU2("SPI self-test (keep the seesaw still):");
    for (int k=0; k<4; k++){
        int ok_id=0, ok_data=0;
        setSPI(con[k], brg[k]);
        imu_setup();
        for (int i=0; i<20; i++){
            if (imu_read_reg(WHO_AM_I)==0x6C) ok_id++;
            if (imu_read_all(r)) ok_data++;
            delay_ms(5);
        }
        sprintf(s, "  %s : WHO_AM_I %2d/20   readings %2d/20", name[k], ok_id, ok_data);
        putsU2(s);
        if (ok_id+ok_data>best_ok){
            best_ok=ok_id+ok_data;
            best=k;
        }
    }
    setSPI(con[best], brg[best]);
    imu_setup();
    sprintf(s, "Using %s", name[best]);
    putsU2(s);

    id=imu_read_reg(WHO_AM_I);
    sprintf(s, "WHO_AM_I = 0x%02X   (LSM6DSO should be 0x6C)", id);
    putsU2(s);
    if (id==0x00) putsU2("  0x00 -> MISO stuck low: check SDO -> pin 7, and 3V3/GND");
    if (id==0xFF) putsU2("  0xFF -> nothing answers: check CS -> pin 3, SCL -> pin 23, SDA -> pin 37");

    sprintf(s, "check: CTRL1_XL = 0x%02X (should be 0x78)", imu_read_reg(CTRL1_XL));
    putsU2(s);

    putsU2("   ax     ay     az  (g)  |   gx     gy     gz  (deg/s) |  roll   pitch | bad reads");
    while (1){
        // read; if it doesn't make sense, try again (up to 3 times)
        for (tries=0; tries<3; tries++){
            reads++;
            if (imu_read_all(r)) break;
            bad++;
        }
        gx=r[0]*GYRO_SCALE;
        gy=r[1]*GYRO_SCALE;
        gz=r[2]*GYRO_SCALE;
        ax=r[3]*ACCEL_SCALE;
        ay=r[4]*ACCEL_SCALE;
        az=r[5]*ACCEL_SCALE;
        roll =atan2f(ay, az)*RAD_TO_DEG;
        pitch=atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;
        sprintf(s, "%6.2f %6.2f %6.2f     | %6.1f %6.1f %6.1f       | %6.1f %6.1f | %lu/%lu",
                ax, ay, az, gx, gy, gz, roll, pitch, bad, reads);
        putsU2(s);
        delay_ms(200);
    }
    return 0;
}
