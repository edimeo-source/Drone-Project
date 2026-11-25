/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/imu.c
 *  What    : LSM6DSO IMU driver over SPI1 and the complementary filter (roll, pitch, yaw).
 *  Version : FC draft   |   Date: 2026-11-04
 *  Status  : Written as practice code before the hardware arrived. Never flown. Superseded by
 *            the SewSaw seesaw code (the IMU moved to SPI).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (I2C2)
 *      3V3 pin 1   GND pin 6   SDA pin 3 (RA3)   SCL pin 5 (RA2)
 *    ESC 1 front-left .. signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 front-right . signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC 3 back-left ... signal J12 pin 21 (RF4 -> OC3)   ground pin 25
 *    ESC 4 back-right .. signal J12 pin 19 (RF5 -> OC4)   ground pin 30
 *    ESC red 5 V wires . NOT connected (ESCs are powered by the battery/PDB)
 *    Bluefruit SPI Friend (shares SPI1 with the IMU)
 *      VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): initI2C(), I2C_wait_for_idle(),
 *    I2CStart(), I2CRestart(), I2CStop(), I2CAck(), I2CNack(), I2CWrite(), I2CRead(),
 *    I2CReadsinglebyte(), I2CReadbytes(), I2CWritebyte(), initIMU(), imu_read_raw(),
 *    imu_calibrate(), imu_update()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <math.h>
# include "imu.h"
# include "delay.h"

// SparkFun LSM6DSO 6DoF (Qwiic) on I2C2 (same pins as Lab 5)
//sda:  pin 3 (RA3)
//scl:  pin 5 (RA2)
//3.3V: pin 1   (never 5 V)
//gnd:  pin 6
//INT1 not connected

#define IMU_ADDR    0x6B        // SparkFun default (0x6A with the jumper cut)
#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12
#define OUTX_L_G    0x22        // first of 12 bytes: gyro X,Y,Z then accel X,Y,Z

#define DT          0.004f      // control loop time, seconds
#define ALPHA       0.98f       // complementary filter: 98% gyro, 2% accelerometer
#define GYRO_SCALE  0.0175f     // 17.5 mdps per count at +/-500 dps (datasheet Table 2)
#define RAD_TO_DEG  57.29578f

volatile float roll=0, pitch=0;
volatile float roll_rate=0, pitch_rate=0, yaw_rate=0;
volatile int imu_error=0;

float gx_bias=0, gy_bias=0, gz_bias=0;

// ================= I2C2 (class code from Lab 5) =================

// ---------------------------------------------------------------------------------------------
// void initI2C(void)
//   Sets up I2C2 (pins 3/5) at 100 kHz with the internal pull-ups on.
// ---------------------------------------------------------------------------------------------
void initI2C(void)
{
    CNPUAbits.CNPUA3=1;
    CNPUAbits.CNPUA2=1;
    I2C2BRG=67;          // CAPSTONE: 400 kHz: (1/(2 x 400 kHz) - 104 ns) x 60 MHz - 2 = 66.8 -> 67
                         // if the bus acts up, go back to 292 (100 kHz, Lab 5)
    I2C2CON=0x8000;
    delay_ms(1);
}

// ---------------------------------------------------------------------------------------------
// void I2C_wait_for_idle(void)
//   Waits until the I2C2 module is idle (gives up after a timeout).
// ---------------------------------------------------------------------------------------------
void I2C_wait_for_idle(void)
{
    while(I2C2CON & 0x1F);
    while(I2C2STATbits.TRSTAT);
}

// ---------------------------------------------------------------------------------------------
// void I2CStart(void)
//   Sends an I2C start condition.
// ---------------------------------------------------------------------------------------------
void I2CStart(void)
{
    I2C_wait_for_idle();
    I2C2CONbits.SEN=1;
    while (I2C2CONbits.SEN);
}

// ---------------------------------------------------------------------------------------------
// void I2CRestart(void)
//   Sends an I2C repeated-start condition.
// ---------------------------------------------------------------------------------------------
void I2CRestart(void){
    I2C_wait_for_idle();
    I2C2CONbits.RSEN=1;
    while (I2C2CONbits.RSEN);
}

// ---------------------------------------------------------------------------------------------
// void I2CStop(void)
//   Sends an I2C stop condition.
// ---------------------------------------------------------------------------------------------
void I2CStop(void)
{
    I2C_wait_for_idle();
    I2C2CONbits.PEN=1;
    while (I2C2CONbits.PEN);
}

// ---------------------------------------------------------------------------------------------
// void I2CAck(void)
//   Answers a received byte with ACK (more bytes wanted).
// ---------------------------------------------------------------------------------------------
void I2CAck(void)
{
    I2C_wait_for_idle();
    I2C2CONbits.ACKDT=0;
    I2C2CONbits.ACKEN=1;
    while (I2C2CONbits.ACKEN);
}

// ---------------------------------------------------------------------------------------------
// void I2CNack(void)
//   Answers a received byte with NACK (last byte).
// ---------------------------------------------------------------------------------------------
void I2CNack(void)
{
    I2C_wait_for_idle();
    I2C2CONbits.ACKDT=1;
    I2C2CONbits.ACKEN=1;
    while (I2C2CONbits.ACKEN);
}

// ---------------------------------------------------------------------------------------------
// void I2CWrite(int data)
//   Sends one byte on I2C2.
// ---------------------------------------------------------------------------------------------
void I2CWrite(int data){
    int t=0;
    I2C2TRN=data;
    while (I2C2STATbits.TBF);
    I2C_wait_for_idle();
    while (I2C2STATbits.ACKSTAT){   // CAPSTONE: give up instead of hanging if the IMU stops answering
        t++;
        if (t>10000){
            imu_error=1;
            break;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// int I2CRead(void)
//   Receives one byte on I2C2.
// ---------------------------------------------------------------------------------------------
int I2CRead(void){
    I2C2CONbits.RCEN=1;
    while (I2C2CONbits.RCEN);
    while (!I2C2STATbits.RBF);
    return I2C2RCV;
}

// ---------------------------------------------------------------------------------------------
// int I2CReadsinglebyte(int address, int reg)
//   Reads one register of the I2C device at address.
// ---------------------------------------------------------------------------------------------
int I2CReadsinglebyte(int address, int reg){
    int a;
    I2CStart();
    a=address<<1;
    I2CWrite(a);
    I2CWrite(reg);
    I2CRestart();
    a=(address<<1)+1;
    I2CWrite(a);
    a=I2CRead();
    I2CNack();
    I2CStop();
    return a;
}

// ---------------------------------------------------------------------------------------------
// void I2CReadbytes(int address, int reg, int b[], int n)
//   Reads n registers in a row from the I2C device at address into b[].
// ---------------------------------------------------------------------------------------------
// CAPSTONE: same as the class I2CReadmultibytes, but reads n bytes into b[]
void I2CReadbytes(int address, int reg, int b[], int n){
    int i, a;
    I2CStart();
    a=address<<1;
    I2CWrite(a);
    I2CWrite(reg);
    I2CRestart();
    a=(address<<1)+1;
    I2CWrite(a);
    for (i=0; i<n; i++)
    {
        b[i]=I2CRead();
        if (i<n-1){
            I2CAck();
        }
        else
        {
            I2CNack();
        }
    }
    I2CStop();
}

// ---------------------------------------------------------------------------------------------
// void I2CWritebyte(int address, int reg, int val)
//   Writes one value to register reg of the I2C device at address.
// ---------------------------------------------------------------------------------------------
void I2CWritebyte(int address, int reg, int val){
    I2CStart();
    I2CWrite(address<<1);
    I2CWrite(reg);
    I2CWrite(val);
    I2CStop();
}

// ================= LSM6DSO =================

// ---------------------------------------------------------------------------------------------
// int initIMU(void)
//   Starts I2C2, reads WHO_AM_I and writes the IMU settings. Returns WHO_AM_I (should be 0x6C).
// ---------------------------------------------------------------------------------------------
int initIMU(void)
{
    int id;
    initI2C();
    id=I2CReadsinglebyte(IMU_ADDR, WHO_AM_I);
    I2CWritebyte(IMU_ADDR, CTRL3_C, 0x44);    // BDU = 1 (high and low byte from the same sample), auto-increment address
    I2CWritebyte(IMU_ADDR, CTRL1_XL, 0x78);   // accel 833 Hz, +/-4 g
    I2CWritebyte(IMU_ADDR, CTRL2_G, 0x74);    // gyro 833 Hz, +/-500 dps
    delay_ms(100);                            // let the sensor settle
    return id;
}

// ---------------------------------------------------------------------------------------------
// void imu_read_raw(int r[6])
//   Reads gyro X,Y,Z and accel X,Y,Z as signed raw counts (I2C version).
// ---------------------------------------------------------------------------------------------
// read 12 bytes and put each low/high pair together as a signed 16-bit number
// r[0..2] = gyro X,Y,Z   r[3..5] = accel X,Y,Z
void imu_read_raw(int r[6])
{
    int b[12];
    I2CReadbytes(IMU_ADDR, OUTX_L_G, b, 12);
    for (int i=0; i<6; i++){
        r[i]=(short)((b[2*i+1]<<8) | b[2*i]);   // high byte, low byte; (short) makes it signed
    }
}

// ---------------------------------------------------------------------------------------------
// void imu_calibrate(void)
//   Averages the IMU while the drone sits still and level to find the gyro and accel offsets
//   (retries if it moved).
// ---------------------------------------------------------------------------------------------
// Average 500 gyro readings while the drone sits still. That average is the
// zero-rate offset (datasheet: up to +/-1 dps), subtracted from every reading after this.
void imu_calibrate(void)
{
    int r[6];
    float sx=0, sy=0, sz=0, ax=0, ay=0, az=0;
    for (int i=0; i<500; i++){
        imu_read_raw(r);
        sx=sx+r[0]*GYRO_SCALE;
        sy=sy+r[1]*GYRO_SCALE;
        sz=sz+r[2]*GYRO_SCALE;
        ax=ax+r[3];
        ay=ay+r[4];
        az=az+r[5];
        delay_ms(2);
    }
    gx_bias=sx/500;
    gy_bias=sy/500;
    gz_bias=sz/500;
    // start the filter at the accelerometer angle instead of 0
    roll =atan2f(ay, az)*RAD_TO_DEG;
    pitch=atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;
}

// ---------------------------------------------------------------------------------------------
// void imu_update(void)
//   Runs every 4 ms: reads the IMU and updates roll, pitch, yaw with the complementary filter
//   (alpha 0.98).
// ---------------------------------------------------------------------------------------------
// Called every 4 ms from the control interrupt
void imu_update(void)
{
    int r[6];
    float gx, gy, gz, ax, ay, az;
    float roll_acc, pitch_acc;

    imu_read_raw(r);

    // counts -> degrees per second, bias removed
    gx=r[0]*GYRO_SCALE-gx_bias;
    gy=r[1]*GYRO_SCALE-gy_bias;
    gz=r[2]*GYRO_SCALE-gz_bias;

    // accelerometer counts (atan2 only uses the ratio, so no scaling needed)
    ax=r[3];
    ay=r[4];
    az=r[5];

    // chip mounted X forward, Y left, Z up
    roll_rate = gx;         // + = left side going up
    pitch_rate=-gy;         // + = nose going up (nose up is a negative turn about +Y)
    yaw_rate  = gz;

    // tilt from gravity
    roll_acc =atan2f(ay, az)*RAD_TO_DEG;
    pitch_acc=atan2f(ax, sqrtf(ay*ay+az*az))*RAD_TO_DEG;

    // complementary filter
    roll =ALPHA*(roll +roll_rate *DT)+(1-ALPHA)*roll_acc;
    pitch=ALPHA*(pitch+pitch_rate*DT)+(1-ALPHA)*pitch_acc;
}
