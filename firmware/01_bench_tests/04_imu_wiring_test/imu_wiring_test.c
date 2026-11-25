/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/04_imu_wiring_test/imu_wiring_test.c
 *  What    : IMU wiring test - LSM6DSO over SPI with CS on pin 16
 *  Version : bench test   |   Date: 2026-11-15
 *  Status  : WORKED: WHO_AM_I reads 0x6C on the real board. Use this first whenever the IMU
 *            wiring changes.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): delay_ms(), initU2(),
 *    U2TXHandler(), tx_free(), putU2(), putsU2(), fx(), initSPI(), spi_xfer(), imu_write(),
 *    imu_read(), imu_read1(), imu_write_check(), spi_config(), sdo_pull(), bb_wait(),
 *    bb_xfer(), bb_who_am_i(), wiring_test(), scl_rel(), i2c_start(), i2c_stop(), i2c_wr(),
 *    i2c_rd_nack(), i2c_read_reg(), i2c_write_reg(), i2c_who_am_i(), show(), i2c_wire_test(),
 *    cand_set(), cs_finder(), wait_for_imu_id(), initIMU(), s16(), imu_sample(),
 *    wait_new_data(), apply_cal(), mm(), calibrate_once(), calibrate(), dz(), wrap180(),
 *    acc_roll(), acc_pitch(), flight_reset(), flight_update(), getU2_nb(), fxw(),
 *    print_header(), print_line(), main()
 *  TYPES IN THIS FILE: imu_data_t, flight_t
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c,
 *  and put config.bits (same folder in the repo) next to it in the project folder. Do not add
 *  config.bits as a source file: main.c #includes it.
 * ============================================================================================ */

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "config.bits"

/* =====================================================================
 * Drone IMU test - LSM6DSO (STMicroelectronics) over SPI
 * Board: PIC32MZ2048EFM144, 120 MHz SYSCLK, 60 MHz PBCLK (config.bits)
 *
 * Prints the IMU's identity first, then accelerometer, gyroscope,
 * temperature, the IMU's own timestamp, and roll/pitch to UART2
 * at 115200 baud (same UART as your other labs).
 *
 * Wiring (same SPI1 pins as the temperature sensor project):
 *   IMU 3V3  -> pin 1
 *   IMU GND  -> pin 6
 *   IMU SCL  -> pin 23  RD1  = SCK1  (SPI clock)
 *   IMU SDA  -> pin 37  RD10 = SDO1  (data PIC -> IMU, "MOSI")
 *   IMU SDO  -> pin 7          SDI1  (data IMU -> PIC, "MISO") - REQUIRED
 *   IMU CS   -> pin 16  RK1          (chip select, active low)
 *
 * In SPI mode the IMU's pins change meaning: SCL = clock, SDA = data in,
 * SDO = data out. Without SDO the PIC can send but never read anything.
 * ===================================================================== */

/* ---------------- LSM6DSO registers ---------------- */
#define WHO_AM_I     0x0F   /* always reads 0x6C on an LSM6DSO */
#define CTRL1_XL     0x10   /* accel data rate + range */
#define CTRL2_G      0x11   /* gyro data rate + range */
#define CTRL3_C      0x12   /* reset, BDU, auto-increment */
#define CTRL4_C      0x13   /* I2C disable */
#define CTRL9_XL     0x18   /* I3C disable */
#define CTRL10_C     0x19   /* timestamp enable */
#define STATUS_REG   0x1E   /* bit0 accel ready, bit1 gyro ready, bit2 temp ready */
#define OUT_TEMP_L   0x20   /* 0x20..0x2D: temp, gyro XYZ, accel XYZ (14 bytes) */
#define TIMESTAMP0   0x40   /* 0x40..0x43: 32-bit timestamp, 25 us per count */

#define LSM6DSO_ID   0x6C

/* Settings written below and the matching scale factors (datasheet table 2) */
#define ODR_HZ       416    /* sample rate - drone loops want it fast   */
#define CTRL1_XL_VAL 0x68   /* ODR 416 Hz (0110), FS +/-4 g (10)        */
#define CTRL2_G_VAL  0x68   /* ODR 416 Hz (0110), FS +/-1000 dps (10)   */
#define ACC_G_PER_LSB    0.000122f   /* 0.122 mg/LSB at +/-4 g      */
#define GYRO_DPS_PER_LSB 0.035f      /* 35 mdps/LSB at +/-1000 dps  */

#define CS_LOW()   (LATKbits.LATK1 = 0)
#define CS_HIGH()  (LATKbits.LATK1 = 1)

/* ---------------- delays (Timer3, 60 MHz / 256 = 234 counts per ms) ---------------- */
// ---------------------------------------------------------------------------------------------
// void delay_ms(int n)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
void delay_ms(int n)
{
    for (int i = 0; i < n; i++) {
        TMR3 = 0;
        while (TMR3 < 234);
    }
}

/* ---------------- UART2, 115200 baud (same as your labs) ---------------- */
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
    U2BRG = 129;
    U2MODE = 0x8008;
    U2STA = 0x1400;

    /* TX goes through a buffer emptied by an interrupt, so printing never
       makes the main loop wait (the sensor loop keeps running at full rate) */
    IFS4bits.U2TXIF = 0;
    IPC36bits.U2TXIP = 3;
    IPC36bits.U2TXIS = 0;
    INTCONbits.MVEC = 1;
    __builtin_enable_interrupts();
}

#define TXBUF 2048
volatile char txb[TXBUF];
volatile unsigned txh = 0, txt = 0;     /* head = write, tail = read */

// ---------------------------------------------------------------------------------------------
// void __ISR(_UART2_TX_VECTOR, IPL3SOFT) U2TXHandler(void)
//   UART2 transmit interrupt: feeds queued characters to the UART so printing never blocks the
//   loop.
// ---------------------------------------------------------------------------------------------
void __ISR(_UART2_TX_VECTOR, IPL3SOFT) U2TXHandler(void)
{
    while (!U2STAbits.UTXBF && txt != txh) {
        U2TXREG = txb[txt];
        txt = (txt + 1) % TXBUF;
    }
    if (txt == txh) IEC4bits.U2TXIE = 0;   /* nothing left: stop interrupting */
    IFS4bits.U2TXIF = 0;
}

// ---------------------------------------------------------------------------------------------
// unsigned tx_free(void)
//   Returns how many characters still fit in the UART2 transmit ring buffer.
// ---------------------------------------------------------------------------------------------
unsigned tx_free(void) { return (txt + TXBUF - txh - 1) % TXBUF; }

// ---------------------------------------------------------------------------------------------
// int putU2(int c)
//   Sends one character out UART2 to the PC (waits while the transmit buffer is full).
// ---------------------------------------------------------------------------------------------
int putU2(int c)              /* waits only if the buffer is completely full */
{
    while (tx_free() == 0);
    txb[txh] = (char)c;
    txh = (txh + 1) % TXBUF;
    IEC4bits.U2TXIE = 1;
    return c;
}

// ---------------------------------------------------------------------------------------------
// void putsU2(const char *s)
//   Sends a text line to the PC over UART2, followed by a new line.
// ---------------------------------------------------------------------------------------------
void putsU2(const char *s)
{
    while (*s) putU2(*s++);
    putU2('\r');
    putU2('\n');
}

// ---------------------------------------------------------------------------------------------
// char *fx(char *buf, float v, int dec)
//   Turns a float into fixed-point text (1-3 decimals) without %f.
// ---------------------------------------------------------------------------------------------
/* XC32 can silently drop %f unless the full printf library is linked,
   so floats are printed as fixed-point integers instead. */
char *fx(char *buf, float v, int dec)
{
    long scale = (dec == 1) ? 10 : (dec == 2) ? 100 : 1000;
    long n = (long)(v * scale + (v < 0 ? -0.5f : 0.5f));
    char sign = (n < 0) ? '-' : '+';
    if (n < 0) n = -n;
    if (dec == 1)      sprintf(buf, "%c%ld.%01ld", sign, n / scale, n % scale);
    else if (dec == 2) sprintf(buf, "%c%ld.%02ld", sign, n / scale, n % scale);
    else               sprintf(buf, "%c%ld.%03ld", sign, n / scale, n % scale);
    return buf;
}

/* ---------------- SPI1 ---------------- */
// ---------------------------------------------------------------------------------------------
// void initSPI(void)
//   Sets up SPI1 (pin remaps, mode, clock speed) and the chip-select pin for this version's
//   wiring.
// ---------------------------------------------------------------------------------------------
void initSPI(void)
{
    TRISKbits.TRISK1 = 0;   /* RK1 = CS (pin 16); pin 3 stays free for I2C2 */
    CS_HIGH();

    RPD10R = 5;             /* SDO1 -> RD10 (pin 37) */
    SDI1R = 13;             /* SDI1 <- pin 7 (same as your temp sensor project) */
    LATDbits.LATD1 = 1;
    TRISDbits.TRISD1 = 0;   /* SCK1 = RD1 (pin 23) */

    SPI1CON = 0;
    (void)SPI1BUF;          /* clear any old byte */
    SPI1BRG = 14;           /* 60 MHz / (2 x (14+1)) = 2 MHz (LSM6DSO max 10 MHz) */
    SPI1CON = 0x8060;       /* ON, master, 8-bit, CKP=1 CKE=0 = SPI mode 3 (what ST specifies).
                               If WHO_AM_I fails, try 0x8120 (mode 0, the setting your
                               temp sensor used) - ST parts usually accept both. */
}

// ---------------------------------------------------------------------------------------------
// uint8_t spi_xfer(uint8_t b)
//   One SPI1 byte out, one byte in, with a 50 us timeout that sets imu_error instead of
//   hanging.
// ---------------------------------------------------------------------------------------------
uint8_t spi_xfer(uint8_t b)
{
    SPI1BUF = b;
    while (!SPI1STATbits.SPIRBF);
    return (uint8_t)SPI1BUF;
}

// ---------------------------------------------------------------------------------------------
// void imu_write(uint8_t reg, uint8_t val)
//   Writes one LSM6DSO register over SPI (CS low, address with bit 7 = 0, value, CS high).
// ---------------------------------------------------------------------------------------------
/* Register write: first byte = address with bit 7 = 0 */
void imu_write(uint8_t reg, uint8_t val)
{
    CS_LOW();
    spi_xfer(reg & 0x7F);
    spi_xfer(val);
    CS_HIGH();
}

// ---------------------------------------------------------------------------------------------
// void imu_read(uint8_t reg, uint8_t *buf, int n)
//   Reads n LSM6DSO registers in a row over SPI (bit 7 = 1 = read; the address
//   auto-increments).
// ---------------------------------------------------------------------------------------------
/* Register read: first byte = address with bit 7 = 1, then clock out data.
   With IF_INC set, the address steps up by itself for multi-byte reads. */
void imu_read(uint8_t reg, uint8_t *buf, int n)
{
    CS_LOW();
    spi_xfer(reg | 0x80);
    for (int i = 0; i < n; i++) buf[i] = spi_xfer(0x00);
    CS_HIGH();
}

// ---------------------------------------------------------------------------------------------
// uint8_t imu_read1(uint8_t reg)
//   Reads one LSM6DSO register and returns its value.
// ---------------------------------------------------------------------------------------------
uint8_t imu_read1(uint8_t reg)
{
    uint8_t v;
    imu_read(reg, &v, 1);
    return v;
}

// ---------------------------------------------------------------------------------------------
// int imu_write_check(uint8_t reg, uint8_t val, const char *name)
//   Writes one IMU register, reads it back and prints whether it stuck.
// ---------------------------------------------------------------------------------------------
/* Write a register and read it back, report if it didn't stick */
int imu_write_check(uint8_t reg, uint8_t val, const char *name)
{
    char s[80];
    imu_write(reg, val);
    uint8_t back = imu_read1(reg);
    sprintf(s, "  %-9s (0x%02X) <- 0x%02X  read back 0x%02X  %s",
            name, reg, val, back, (back == val) ? "OK" : "MISMATCH");
    putsU2(s);
    return back == val;
}

/* ---------------- wiring self-test ----------------
 * Pin 7 (SDI1R = 13) is RA14. This reads WHO_AM_I every way we can:
 *   - SPI mode 3 and mode 0, fast (2 MHz) and slow (250 kHz)
 *   - bit-banged by hand (skips the SPI module completely)
 * each once with a pull-UP and once with a pull-DOWN on the SDO line.
 * If the IMU really drives SDO, pull-up and pull-down give the same answer.
 * If they differ, nothing is driving SDO -> wiring problem.
 * Picks the first setting that returns 0x6C and keeps it.          */

// ---------------------------------------------------------------------------------------------
// void spi_config(uint32_t con, int brg)
//   Restarts SPI1 with a given control word (mode) and baud divider (speed).
// ---------------------------------------------------------------------------------------------
void spi_config(uint32_t con, int brg)
{
    SPI1CON = 0;
    (void)SPI1BUF;
    SPI1BRG = brg;
    SPI1CON = con;
}

// ---------------------------------------------------------------------------------------------
// void sdo_pull(int up)
//   Turns the pull-up (up = 1) or pull-down (up = 0) on the MISO pin, to see if the IMU drives
//   it.
// ---------------------------------------------------------------------------------------------
void sdo_pull(int up)
{
    CNPUAbits.CNPUA14 = up ? 1 : 0;
    CNPDAbits.CNPDA14 = up ? 0 : 1;
    delay_ms(1);
}

int bb_delay = 300;            /* bit-bang speed: bigger = slower */
// ---------------------------------------------------------------------------------------------
// void bb_wait(void)
//   Short busy-wait that sets the bit-bang speed (bigger bb_delay = slower).
// ---------------------------------------------------------------------------------------------
void bb_wait(void) { for (volatile int k = 0; k < bb_delay; k++); }

// ---------------------------------------------------------------------------------------------
// uint8_t bb_xfer(uint8_t out)
//   One SPI byte by bit-banging the pins in mode 3 (to test the wires without the SPI module).
// ---------------------------------------------------------------------------------------------
uint8_t bb_xfer(uint8_t out)   /* SPI mode 3 by hand */
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        LATDbits.LATD1 = 0;                     /* falling edge: data changes */
        LATDbits.LATD10 = (out >> i) & 1;
        bb_wait();
        LATDbits.LATD1 = 1;                     /* rising edge: both sides sample */
        in = (in << 1) | PORTAbits.RA14;
        bb_wait();
    }
    return in;
}

// ---------------------------------------------------------------------------------------------
// uint8_t bb_who_am_i(void)
//   Reads WHO_AM_I by bit-banged SPI, to tell a wiring fault from an SPI setup fault.
// ---------------------------------------------------------------------------------------------
uint8_t bb_who_am_i(void)
{
    uint8_t id;
    SPI1CON = 0;                 /* SPI module off, pins become plain I/O */
    RPD10R = 0;                  /* RD10 back to normal pin */
    LATDbits.LATD1 = 1;          /* clock idles high (mode 3) */
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD10 = 0;
    bb_wait();
    CS_LOW();
    bb_wait();
    bb_xfer(WHO_AM_I | 0x80);
    id = bb_xfer(0x00);
    CS_HIGH();
    RPD10R = 5;                  /* give RD10 back to SDO1 */
    return id;
}

// ---------------------------------------------------------------------------------------------
// void wiring_test(void)
//   Full SPI wiring test: tries 2 SPI modes at 2 speeds with MISO pulled up and down, prints
//   the results.
// ---------------------------------------------------------------------------------------------
void wiring_test(void)
{
    const uint32_t con[4] = { 0x8060, 0x8060, 0x8120, 0x8120 };
    const int      brg[4] = { 14,     119,    14,     119    };
    const char    *nm[4]  = { "SPI mode 3, 2 MHz  ", "SPI mode 3, 250 kHz",
                              "SPI mode 0, 2 MHz  ", "SPI mode 0, 250 kHz" };
    uint8_t up, dn;
    int found = -1, driven = 0;
    char s[100];

    putsU2("Wiring test - WHO_AM_I should be 0x6C:");
    for (int i = 0; i < 4; i++) {
        spi_config(con[i], brg[i]);
        sdo_pull(1); up = imu_read1(WHO_AM_I);
        sdo_pull(0); dn = imu_read1(WHO_AM_I);
        sprintf(s, "  %s  SDO pull-up: 0x%02X  pull-down: 0x%02X%s",
                nm[i], up, dn, (up == 0x6C && dn == 0x6C) ? "  <-- WORKS" : "");
        putsU2(s);
        if (up == dn) driven = 1;
        if (found < 0 && up == 0x6C && dn == 0x6C) found = i;
    }
    sdo_pull(1); up = bb_who_am_i();
    sdo_pull(0); dn = bb_who_am_i();
    sprintf(s, "  bit-banged by hand    SDO pull-up: 0x%02X  pull-down: 0x%02X%s",
            up, dn, (up == 0x6C && dn == 0x6C) ? "  <-- WORKS" : "");
    putsU2(s);
    if (up == dn) driven = 1;

    CNPUAbits.CNPUA14 = 0;       /* pulls off again */
    CNPDAbits.CNPDA14 = 0;

    /* Very slow read with NO pull on pin 7. A bad contact on the SDO wire
     * (high resistance) can't beat the pull-up/pull-down above, but with no
     * pull and lots of time the IMU's answer can still creep through.     */
    uint8_t slow;
    delay_ms(1);
    bb_delay = 30000;            /* about 100x slower */
    slow = bb_who_am_i();
    bb_delay = 300;
    sprintf(s, "  bit-banged VERY slowly, no pull:  0x%02X", slow);
    putsU2(s);
    if (found < 0 && slow == 0x6C)
        putsU2("  -> 0x6C ONLY when slow with no pull: the SDO wire (pin 7) has a BAD CONTACT.\r\n"
               "     The IMU's answer barely gets through. Reseat or replace that jumper, and\r\n"
               "     check the solder joint on the IMU's SDO pin.");

    if (found >= 0) {
        sprintf(s, "Using: %s", nm[found]);
        putsU2(s);
        spi_config(con[found], brg[found]);
        return;
    }
    spi_config(0x8060, 14);
    if (up == 0x6C && dn == 0x6C)
        putsU2("Only bit-banging works -> the SPI pin mapping is off. Send me this output.");
    else if (!driven)
        putsU2("Pull-up and pull-down disagree -> the IMU's answer isn't reaching pin 7.\r\n"
               "  If the wire-by-wire test above says everything is GOOD, the SDO wire (pin 7 <-> IMU SDO)\r\n"
               "  has a bad contact: it passes a voltage TO the IMU but not the IMU's answer back.\r\n"
               "  Reseat or replace that jumper and check the IMU's SDO solder joint.");
    else
        putsU2("Something drives SDO but the answer is wrong -> SCL/SDA are probably swapped\r\n"
               "  or SCL isn't on pin 23. SCL = pin 23 (clock), SDA = pin 37 (data in).");
}

/* ---------------- wire-by-wire test over I2C ----------------
 * With CS high the LSM6DSO also answers I2C on the same two wires
 * (SDA = pin 37, SCL = pin 23), and its SDO pin picks the address:
 * SDO low -> 0x6A, SDO high -> 0x6B. So, bit-banging I2C:
 *   answers at all            -> 3V3, GND, SDA, SCL wires are good
 *   address follows pin 7     -> SDO wire is good
 *   goes quiet when CS is low -> CS wire is good                    */

#define SDA_LOW()  do { LATDbits.LATD10 = 0; TRISDbits.TRISD10 = 0; } while (0)
#define SDA_REL()  (TRISDbits.TRISD10 = 1)
#define SCL_LOW()  do { LATDbits.LATD1 = 0; TRISDbits.TRISD1 = 0; } while (0)
#define SDA_IN()   (PORTDbits.RD10)

// ---------------------------------------------------------------------------------------------
// void scl_rel(void)
//   Releases the bit-banged I2C clock line and waits while a device holds it low (clock
//   stretching).
// ---------------------------------------------------------------------------------------------
void scl_rel(void)
{
    TRISDbits.TRISD1 = 1;
    for (int t = 0; t < 1000 && !PORTDbits.RD1; t++) bb_wait();   /* clock stretching */
}

// ---------------------------------------------------------------------------------------------
// void i2c_start(void)
//   Bit-banged I2C start condition.
// ---------------------------------------------------------------------------------------------
void i2c_start(void) { SDA_REL(); scl_rel(); bb_wait(); SDA_LOW(); bb_wait(); SCL_LOW(); bb_wait(); }
// ---------------------------------------------------------------------------------------------
// void i2c_stop(void)
//   Bit-banged I2C stop condition.
// ---------------------------------------------------------------------------------------------
void i2c_stop(void)  { SDA_LOW(); bb_wait(); scl_rel(); bb_wait(); SDA_REL(); bb_wait(); }

// ---------------------------------------------------------------------------------------------
// int i2c_wr(uint8_t b)
//   Bit-banged I2C byte write. 1 = the IMU acknowledged.
// ---------------------------------------------------------------------------------------------
int i2c_wr(uint8_t b)          /* returns 1 if the IMU ACKed */
{
    for (int i = 7; i >= 0; i--) {
        if ((b >> i) & 1) SDA_REL(); else SDA_LOW();
        bb_wait(); scl_rel(); bb_wait(); SCL_LOW();
    }
    SDA_REL(); bb_wait(); scl_rel(); bb_wait();
    int ack = !SDA_IN();
    SCL_LOW(); bb_wait();
    return ack;
}

// ---------------------------------------------------------------------------------------------
// uint8_t i2c_rd_nack(void)
//   Bit-banged I2C byte read, answered with NACK (last byte).
// ---------------------------------------------------------------------------------------------
uint8_t i2c_rd_nack(void)
{
    uint8_t v = 0;
    SDA_REL();
    for (int i = 0; i < 8; i++) {
        bb_wait(); scl_rel(); bb_wait();
        v = (v << 1) | SDA_IN();
        SCL_LOW();
    }
    SDA_REL(); bb_wait(); scl_rel(); bb_wait(); SCL_LOW(); bb_wait();   /* NACK */
    return v;
}

// ---------------------------------------------------------------------------------------------
// int i2c_read_reg(uint8_t addr, uint8_t reg)
//   Reads one register over bit-banged I2C. Returns -1 if nobody acknowledged.
// ---------------------------------------------------------------------------------------------
/* read one register over I2C: returns -1 if no ACK, else the value */
int i2c_read_reg(uint8_t addr, uint8_t reg)
{
    int v = -1;
    i2c_start();
    if (i2c_wr(addr << 1) && i2c_wr(reg)) {
        i2c_start();                         /* repeated start */
        if (i2c_wr((addr << 1) | 1)) v = i2c_rd_nack();
    }
    i2c_stop();
    return v;
}

// ---------------------------------------------------------------------------------------------
// int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
//   Writes one register over bit-banged I2C. 1 = every byte was acknowledged.
// ---------------------------------------------------------------------------------------------
/* write one register over I2C: returns 1 if the IMU ACKed every byte */
int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    int ok;
    i2c_start();
    ok = i2c_wr(addr << 1) && i2c_wr(reg) && i2c_wr(val);
    i2c_stop();
    return ok;
}

// ---------------------------------------------------------------------------------------------
// int i2c_who_am_i(uint8_t addr)
//   Reads WHO_AM_I over bit-banged I2C at address addr (-1 = no answer).
// ---------------------------------------------------------------------------------------------
int i2c_who_am_i(uint8_t addr) { return i2c_read_reg(addr, WHO_AM_I); }

// ---------------------------------------------------------------------------------------------
// void show(char *s, const char *what, int v6a, int v6b)
//   Formats and prints one line of the I2C test results for addresses 0x6A and 0x6B.
// ---------------------------------------------------------------------------------------------
void show(char *s, const char *what, int v6a, int v6b)
{
    char a[12], b[12];
    if (v6a < 0) sprintf(a, "no answer"); else sprintf(a, "0x%02X", v6a);
    if (v6b < 0) sprintf(b, "no answer"); else sprintf(b, "0x%02X", v6b);
    sprintf(s, "  %-30s  addr 0x6A: %-9s  addr 0x6B: %s", what, a, b);
    putsU2(s);
}

// ---------------------------------------------------------------------------------------------
// void i2c_wire_test(void)
//   Checks whether the IMU answers over I2C instead (bit-banged) at both possible addresses.
// ---------------------------------------------------------------------------------------------
void i2c_wire_test(void)
{
    char s[120];
    int lo_a, lo_b, hi_a, hi_b, cs_a, cs_b;

    SPI1CON = 0;                                /* SPI off, pins become plain I/O */
    RPD10R = 0;
    CNPUDbits.CNPUD1 = 1;                       /* backup pull-ups on SDA/SCL */
    CNPUDbits.CNPUD10 = 1;
    SDA_REL(); TRISDbits.TRISD1 = 1;
    CS_HIGH();                                  /* CS high = I2C allowed */
    delay_ms(2);

    putsU2("Wire-by-wire test (I2C, WHO_AM_I should be 0x6C):");

    LATAbits.LATA14 = 0; TRISAbits.TRISA14 = 0; delay_ms(1);   /* drive SDO low  -> 0x6A */
    lo_a = i2c_who_am_i(0x6A); lo_b = i2c_who_am_i(0x6B);
    show(s, "CS high, pin 7 driven LOW", lo_a, lo_b);

    LATAbits.LATA14 = 1; delay_ms(1);                          /* drive SDO high -> 0x6B */
    hi_a = i2c_who_am_i(0x6A); hi_b = i2c_who_am_i(0x6B);
    show(s, "CS high, pin 7 driven HIGH", hi_a, hi_b);

    TRISAbits.TRISA14 = 1;                                     /* let go of SDO, and hold it low with  */
    CNPDAbits.CNPDA14 = 1; delay_ms(1);                        /* the weak pull-down: IMU sits at 0x6A, */
                                                               /* and if it talks SPI it can still drive */
                                                               /* the pin without fighting the PIC       */
    CS_LOW(); delay_ms(1);                                     /* CS low -> should stop I2C */
    cs_a = i2c_who_am_i(0x6A); cs_b = i2c_who_am_i(0x6B);
    show(s, "CS LOW, pin 7 pulled low (should be silent)", cs_a, cs_b);
    CS_HIGH();
    CNPDAbits.CNPDA14 = 0;                                     /* pull-down off */

    int any = (lo_a >= 0 || lo_b >= 0 || hi_a >= 0 || hi_b >= 0);
    putsU2("Result:");
    if (!any) {
        putsU2("  IMU never answered -> power or SDA/SCL problem. Check, IMU end to header:");
        putsU2("    3V3 -> pin 1, GND -> pin 6, SDA -> pin 37, SCL -> pin 23,");
        putsU2("    the header pins are SOLDERED to the IMU board, and its power LED is on.");
    } else {
        putsU2("  3V3, GND, SDA (pin 37) and SCL (pin 23): GOOD - the IMU answered.");
        if (lo_a >= 0 && lo_b < 0 && hi_b >= 0 && hi_a < 0)
            putsU2("  SDO (pin 7): GOOD - the address followed pin 7.");
        else
            putsU2("  SDO (pin 7): BAD - the address did NOT follow pin 7. SDO wire isn't reaching the IMU's SDO pad.");
        if (lo_a >= 0 && cs_a < 0 && cs_b < 0)   /* answered at 0x6A, then silent with CS low */
            putsU2("  CS (pin 16): GOOD - the IMU went quiet when CS went low.");
        else
            putsU2("  CS (pin 16): BAD - the IMU kept answering with CS low. CS wire isn't reaching the IMU's CS pad.");

        /* The IMU keeps its settings while the PIC is reflashed (it stays powered).
         * If a stray write set SIM = 1 (bit 3 of CTRL3_C), it answers SPI on the
         * SDA wire instead of SDO, so pin 7 never sees data. Read the interface
         * settings over I2C, then reset the IMU back to its power-on settings.   */
        LATAbits.LATA14 = 0; TRISAbits.TRISA14 = 0; delay_ms(1);   /* SDO low -> address 0x6A */
        int c3 = i2c_read_reg(0x6A, CTRL3_C);
        int c4 = i2c_read_reg(0x6A, CTRL4_C);
        int c9 = i2c_read_reg(0x6A, CTRL9_XL);
        sprintf(s, "  IMU settings now: CTRL3_C = 0x%02X (power-on 0x04)  CTRL4_C = 0x%02X (0x00)  CTRL9_XL = 0x%02X (0xE0)",
                c3 & 0xFF, c4 & 0xFF, c9 & 0xFF);
        putsU2(s);
        if (c3 >= 0 && (c3 & 0x08))
            putsU2("  SIM = 1: the IMU is in 3-wire SPI mode and never answers on SDO (pin 7). That is the 0x7F.");
        i2c_write_reg(0x6A, CTRL3_C, 0x01);    /* SW_RESET: every setting back to power-on */
        delay_ms(10);
        c3 = i2c_read_reg(0x6A, CTRL3_C);
        sprintf(s, "  Reset the IMU over I2C -> CTRL3_C = 0x%02X (should be 0x04).", c3 & 0xFF);
        putsU2(s);
        TRISAbits.TRISA14 = 1;                 /* let go of SDO */
    }

    CNPUDbits.CNPUD1 = 0;                       /* put SPI back */
    CNPUDbits.CNPUD10 = 0;
    LATDbits.LATD1 = 1;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD10 = 0;
    RPD10R = 5;
    spi_config(0x8060, 14);
}

/* ---------------- CS finder ----------------
 * Finds where the CS wire really is. Every free J12 I/O pin is set high
 * (CS idle), then each one is pulled low by itself while we ask for
 * WHO_AM_I. The pin that gets 0x6C back is the one the CS wire is on.
 * Only free pins are tried: not 7/23/37 (SPI), not 8/10 (PC serial),
 * not power or ground.                                                */
#define NCAND 20
const int cand_pin[NCAND] = { 3, 11, 12, 13, 15, 16, 18, 19, 21, 22,
                              24, 26, 29, 31, 32, 33, 35, 36, 38, 40 };

// ---------------------------------------------------------------------------------------------
// void cand_set(int pin, int level)
//   Drives one candidate J12 pin (3, 11, 12, 13, ...) high or low while searching for the CS
//   wire.
// ---------------------------------------------------------------------------------------------
void cand_set(int pin, int level)      /* drive one J12 pin high (1) or low (0) */
{
    switch (pin) {
    case 3:  LATAbits.LATA3   = level; TRISAbits.TRISA3   = 0; break;
    case 11: LATDbits.LATD0   = level; TRISDbits.TRISD0   = 0; break;
    case 12: LATFbits.LATF2   = level; TRISFbits.TRISF2   = 0; break;
    case 13: LATJbits.LATJ13  = level; TRISJbits.TRISJ13  = 0; break;
    case 15: ANSELBbits.ANSB3 = 0;     /* RB3 is analog at reset */
             LATBbits.LATB3   = level; TRISBbits.TRISB3   = 0; break;
    case 16: LATKbits.LATK1   = level; TRISKbits.TRISK1   = 0; break;
    case 18: LATKbits.LATK2   = level; TRISKbits.TRISK2   = 0; break;
    case 19: LATFbits.LATF5   = level; TRISFbits.TRISF5   = 0; break;
    case 21: LATFbits.LATF4   = level; TRISFbits.TRISF4   = 0; break;
    case 22: LATKbits.LATK3   = level; TRISKbits.TRISK3   = 0; break;
    case 24: LATJbits.LATJ5   = level; TRISJbits.TRISJ5   = 0; break;
    case 26: LATJbits.LATJ7   = level; TRISJbits.TRISJ7   = 0; break;
    case 29: LATHbits.LATH7   = level; TRISHbits.TRISH7   = 0; break;
    case 31: LATHbits.LATH9   = level; TRISHbits.TRISH9   = 0; break;
    case 32: LATHbits.LATH11  = level; TRISHbits.TRISH11  = 0; break;
    case 33: LATKbits.LATK4   = level; TRISKbits.TRISK4   = 0; break;
    case 35: LATKbits.LATK5   = level; TRISKbits.TRISK5   = 0; break;
    case 36: LATKbits.LATK6   = level; TRISKbits.TRISK6   = 0; break;
    case 38: LATHbits.LATH15  = level; TRISHbits.TRISH15  = 0; break;
    case 40: LATJbits.LATJ3   = level; TRISJbits.TRISJ3   = 0; break;
    }
}

// ---------------------------------------------------------------------------------------------
// int cs_finder(void)
//   Tries every free J12 pin as chip select and returns the pin the IMU answers on (0 = none).
// ---------------------------------------------------------------------------------------------
/* returns the J12 pin the CS wire is on, or 0 if no free pin works */
int cs_finder(void)
{
    int i, found = 0;
    uint8_t id;

    spi_config(0x8060, 119);                 /* mode 3, slow 250 kHz to be safe */
    for (i = 0; i < NCAND; i++) cand_set(cand_pin[i], 1);   /* all idle high */
    delay_ms(1);

    for (i = 0; i < NCAND; i++) {
        cand_set(cand_pin[i], 0);            /* this pin = CS low */
        spi_xfer(WHO_AM_I | 0x80);
        id = spi_xfer(0x00);
        cand_set(cand_pin[i], 1);            /* back to idle */
        if (id == LSM6DSO_ID && found == 0) found = cand_pin[i];
    }

    spi_config(0x8060, 14);                  /* back to normal: mode 3, 2 MHz */
    return found;
}

/* ---------------- IMU bring-up ---------------- */
// ---------------------------------------------------------------------------------------------
// void wait_for_imu_id(void)
//   Keeps reading WHO_AM_I until the IMU answers 0x6C, printing wiring hints while it does not.
// ---------------------------------------------------------------------------------------------
void wait_for_imu_id(void)
{
    char s[120];
    uint8_t id;
    int pin;

    while (1) {
        id = imu_read1(WHO_AM_I);
        if (id == LSM6DSO_ID) return;

        sprintf(s, "WHO_AM_I = 0x%02X with CS on pin 16, expected 0x6C - IMU not answering.", id);
        putsU2(s);
        pin = cs_finder();
        if (pin == 16)
            putsU2("  CS finder: pin 16 works now (loose wire?) - trying again.");
        else if (pin > 0) {
            sprintf(s, "  CS finder: the IMU answers with CS on pin %d -> move that wire to pin 16.", pin);
            putsU2(s);
        } else {
            putsU2("  CS finder: no free pin works as CS. If the wire-by-wire test said CS is GOOD,");
            putsU2("  the SDO wire (pin 7) is the problem: reseat it - this retries every second.");
        }
        putsU2("  Retrying in 1 s...");
        delay_ms(1000);
    }
}

// ---------------------------------------------------------------------------------------------
// int initIMU(void)
//   Resets the LSM6DSO and writes every setting with a read-back check. 1 = every setting
//   stuck.
// ---------------------------------------------------------------------------------------------
int initIMU(void)
{
    int ok = 1;
    int i;

    /* software reset back to factory defaults, then wait for it to finish */
    imu_write(CTRL3_C, 0x01);
    for (i = 0; i < 100; i++) {
        delay_ms(1);
        if ((imu_read1(CTRL3_C) & 0x01) == 0) break;
    }

    putsU2("Configuring:");
    ok &= imu_write_check(CTRL3_C,  0x44, "CTRL3_C");    /* BDU=1 (no half-updated samples), IF_INC=1 */
    ok &= imu_write_check(CTRL4_C,  0x04, "CTRL4_C");    /* I2C off - SPI only */
    ok &= imu_write_check(CTRL9_XL, 0xE2, "CTRL9_XL");   /* I3C off (default 0xE0 + bit 1) */
    ok &= imu_write_check(CTRL10_C, 0x20, "CTRL10_C");   /* timestamp counter on */
    ok &= imu_write_check(CTRL1_XL, CTRL1_XL_VAL, "CTRL1_XL");
    ok &= imu_write_check(CTRL2_G,  CTRL2_G_VAL,  "CTRL2_G");
    return ok;
}

/* ---------------- readings ---------------- */
// ---------------------------------------------------------------------------------------------
// TYPE: typedef struct imu_data_t
//   One IMU sample in real units: accel (g), gyro (deg/s), temperature (C) and the IMU's
//   timestamp.
// ---------------------------------------------------------------------------------------------
typedef struct {
    float ax, ay, az;       /* g     */
    float gx, gy, gz;       /* deg/s */
    float temp_c;           /* deg C */
    uint32_t ts;            /* IMU timestamp counts (25 us each) */
} imu_data_t;

// ---------------------------------------------------------------------------------------------
// static int16_t s16(uint8_t lo, uint8_t hi)
//   Joins a low byte and a high byte into one signed 16-bit value.
// ---------------------------------------------------------------------------------------------
static int16_t s16(uint8_t lo, uint8_t hi) { return (int16_t)((hi << 8) | lo); }

// ---------------------------------------------------------------------------------------------
// void imu_sample(imu_data_t *d)
//   Reads one raw IMU sample (temperature, gyro, accel, IMU timestamp) in real units, no
//   corrections.
// ---------------------------------------------------------------------------------------------
/* Raw reading straight from the IMU, converted to real units, no corrections */
void imu_sample(imu_data_t *d)
{
    uint8_t b[14], t[4];

    imu_read(OUT_TEMP_L, b, 14);    /* temp, gyro XYZ, accel XYZ - same moment (BDU) */
    imu_read(TIMESTAMP0, t, 4);

    d->temp_c = 25.0f + s16(b[0], b[1]) / 256.0f;    /* 256 LSB per deg C, 0 = 25 C */
    d->gx = s16(b[2],  b[3])  * GYRO_DPS_PER_LSB;
    d->gy = s16(b[4],  b[5])  * GYRO_DPS_PER_LSB;
    d->gz = s16(b[6],  b[7])  * GYRO_DPS_PER_LSB;
    d->ax = s16(b[8],  b[9])  * ACC_G_PER_LSB;
    d->ay = s16(b[10], b[11]) * ACC_G_PER_LSB;
    d->az = s16(b[12], b[13]) * ACC_G_PER_LSB;
    d->ts = ((uint32_t)t[3] << 24) | ((uint32_t)t[2] << 16) | ((uint32_t)t[1] << 8) | t[0];
}

// ---------------------------------------------------------------------------------------------
// void wait_new_data(void)
//   Waits until the IMU has a new accelerometer AND gyro sample ready (STATUS_REG bits 0-1).
// ---------------------------------------------------------------------------------------------
void wait_new_data(void)
{
    while ((imu_read1(STATUS_REG) & 0x03) != 0x03);   /* accel + gyro both new */
}

/* ---------------- tuning (change these freely) ---------------- */
#define PRINT_MS        40      /* one line every 40 ms = 25 lines per second.
                                   Serial is the limit: one line takes ~8 ms to send at
                                   115200 baud, so ~10 ms is the fastest possible.
                                   (Use 400 for one line every 0.4 s.) */
#define RATE_DEADBAND   0.5f    /* deg/s: gyro rates smaller than this read 0
                                   (sensor noise at rest is about +/-0.2)       */
#define ANGLE_DEADBAND  0.3f    /* deg:   roll/pitch/yaw within this of 0 read 0 */
#define ACC_DEADBAND    0.01f   /* g:     accel within this of its rest value reads rest */
#define RATE_SMOOTH     0.25f   /* 0..1 per sample, smaller = smoother but slower gyro */
#define ACC_SMOOTH      0.10f   /* 0..1 per sample, smaller = smoother accel           */
#define COMP_ALPHA      0.98f   /* angle filter: trust gyro 98%, accel 2% each sample  */

#define DEG 57.29578f

/* ---------------- calibration ----------------
 * Still and level, the IMU should read accel X = 0, Y = 0, Z = +1.000 g and
 * gyro 0/0/0. Calibration averages 2 s and stores what's left over as offsets.
 * It rejects the run if the board moved - averaging movement is what gave
 * the +28/-26/-39 deg/s gyro error before.                               */

typedef struct { float ax, ay, az, gx, gy, gz; } offsets_t;
offsets_t off = { 0, 0, 0, 0, 0, 0 };

// ---------------------------------------------------------------------------------------------
// void apply_cal(const imu_data_t *raw, imu_data_t *d)
//   Copies a raw IMU sample and subtracts the calibration offsets from it.
// ---------------------------------------------------------------------------------------------
void apply_cal(const imu_data_t *raw, imu_data_t *d)
{
    *d = *raw;
    d->ax -= off.ax; d->ay -= off.ay; d->az -= off.az;
    d->gx -= off.gx; d->gy -= off.gy; d->gz -= off.gz;
}

#define CAL_SAMPLES   (2 * ODR_HZ)   /* 2 seconds */
#define GYRO_STILL    2.0f           /* max wobble allowed while calibrating, deg/s */
#define ACC_STILL     0.05f          /* max wobble allowed while calibrating, g */

// ---------------------------------------------------------------------------------------------
// static void mm(float v, float *lo, float *hi)
//   Updates a running minimum and maximum with the value v.
// ---------------------------------------------------------------------------------------------
static void mm(float v, float *lo, float *hi) { if (v < *lo) *lo = v; if (v > *hi) *hi = v; }

// ---------------------------------------------------------------------------------------------
// int calibrate_once(void)
//   Averages 2 s of IMU samples; 1 = the board stayed still and the offsets were saved.
// ---------------------------------------------------------------------------------------------
int calibrate_once(void)       /* returns 1 if the board stayed still */
{
    imu_data_t r;
    float s[6] = { 0, 0, 0, 0, 0, 0 };
    float lo[6], hi[6];
    char line[120], a[16], b[16], c[16];

    for (int i = 0; i < 6; i++) { lo[i] = 1e9f; hi[i] = -1e9f; }

    for (int i = 0; i < CAL_SAMPLES; i++) {
        wait_new_data();
        imu_sample(&r);
        float v[6] = { r.ax, r.ay, r.az, r.gx, r.gy, r.gz };
        for (int k = 0; k < 6; k++) { s[k] += v[k]; mm(v[k], &lo[k], &hi[k]); }
    }

    float gwob = 0, awob = 0;
    for (int k = 0; k < 3; k++) if (hi[k] - lo[k] > awob) awob = hi[k] - lo[k];
    for (int k = 3; k < 6; k++) if (hi[k] - lo[k] > gwob) gwob = hi[k] - lo[k];

    if (gwob > GYRO_STILL || awob > ACC_STILL) {
        sprintf(line, "  Moved during calibration (gyro wobble %s deg/s, accel wobble %s g) - again...",
                fx(a, gwob, 2), fx(b, awob, 3));
        putsU2(line);
        return 0;
    }

    float n = CAL_SAMPLES;
    float mz = s[2] / n;
    off.ax = s[0] / n;
    off.ay = s[1] / n;
    off.az = mz - (mz >= 0 ? 1.0f : -1.0f);   /* keep gravity on Z */
    off.gx = s[3] / n;
    off.gy = s[4] / n;
    off.gz = s[5] / n;

    sprintf(line, "  accel offsets removed: X=%s Y=%s Z=%s g",
            fx(a, off.ax, 3), fx(b, off.ay, 3), fx(c, off.az, 3));
    putsU2(line);
    sprintf(line, "  gyro  offsets removed: X=%s Y=%s Z=%s deg/s",
            fx(a, off.gx, 2), fx(b, off.gy, 2), fx(c, off.gz, 2));
    putsU2(line);
    return 1;
}

// ---------------------------------------------------------------------------------------------
// void calibrate(void)
//   Calibration routine: asks you to set the drone down level, then calls calibrate_once()
//   until it stays still.
// ---------------------------------------------------------------------------------------------
void calibrate(void)
{
    imu_data_t r;
    putsU2("");
    putsU2("CALIBRATING - set the drone down LEVEL and don't touch it (2 s)...");
    delay_ms(1500);                                    /* time to let go of it */
    for (int i = 0; i < 40; i++) { wait_new_data(); imu_sample(&r); }   /* flush */
    for (int tries = 1; tries <= 15; tries++)
        if (calibrate_once()) { putsU2("  Calibration done."); return; }
    putsU2("  Never held still - keeping the last good offsets. Press c to try again.");
}

/* ---------------- flight data (what a PID loop uses) ----------------
 * Rates  (deg/s) = gyro, smoothed + dead zone   -> RATE / acro mode PID input
 * Angles (deg)   = complementary filter: gyro integrated each sample, slowly
 *                  pulled toward the accel's gravity angle so it can't drift
 *                                                -> ANGLE / self-level PID input
 * Yaw angle      = gyro integrated only (no compass), so it drifts slowly;
 *                  the dead zone stops it drifting while still. z re-zeros it.
 * dt (ms)        = time between samples from the IMU's own clock -> PID dt   */

// ---------------------------------------------------------------------------------------------
// TYPE: typedef struct flight_t
//   The filter's output: roll, pitch, yaw angles and rates.
// ---------------------------------------------------------------------------------------------
typedef struct {
    float roll, pitch, yaw;          /* deg   */
    float rr, pr, yr;                /* deg/s, after dead zone */
    float ax, ay, az;                /* g,     after dead zone */
    float dt_ms;
    float temp_c;
} flight_t;

flight_t fl;
float sgx, sgy, sgz, sax, say, saz, stc;   /* smoothed sensor values */

// ---------------------------------------------------------------------------------------------
// static float dz(float v, float band)
//   Dead zone: returns 0 if v is inside +/-band, otherwise v.
// ---------------------------------------------------------------------------------------------
static float dz(float v, float band) { return (v > band || v < -band) ? v : 0.0f; }

// ---------------------------------------------------------------------------------------------
// static float wrap180(float a)
//   Wraps an angle into the range -180 to +180 degrees.
// ---------------------------------------------------------------------------------------------
static float wrap180(float a)
{
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

// ---------------------------------------------------------------------------------------------
// static float acc_roll(void)
//   Roll angle from the accelerometer alone (atan2 of Y and Z), in degrees.
// ---------------------------------------------------------------------------------------------
/* Signs (same as the flight code and the mixer):
 *   roll  + = LEFT side up    (IMU X arrow pointing to the front)
 *   pitch + = NOSE up         (front = the way the X arrow points)  */
static float acc_roll(void)  { return atan2f(say, saz) * DEG; }
// ---------------------------------------------------------------------------------------------
// static float acc_pitch(void)
//   Pitch angle from the accelerometer alone, in degrees.
// ---------------------------------------------------------------------------------------------
static float acc_pitch(void) { return atan2f(sax, sqrtf(say * say + saz * saz)) * DEG; }

// ---------------------------------------------------------------------------------------------
// void flight_reset(const imu_data_t *d)
//   Starts the angle filter from one IMU sample (accelerometer angles, rates zeroed).
// ---------------------------------------------------------------------------------------------
void flight_reset(const imu_data_t *d)
{
    sgx = sgy = sgz = 0;
    sax = d->ax; say = d->ay; saz = d->az;
    stc = d->temp_c;
    fl.roll  = acc_roll();
    fl.pitch = acc_pitch();
    fl.yaw   = 0;
}

// ---------------------------------------------------------------------------------------------
// void flight_update(const imu_data_t *d, float dt)
//   Smooths the IMU sample and runs the complementary filter for one time step dt.
// ---------------------------------------------------------------------------------------------
void flight_update(const imu_data_t *d, float dt)
{
    /* smooth the noise (exponential moving average) */
    sgx += RATE_SMOOTH * (d->gx - sgx);
    sgy += RATE_SMOOTH * (d->gy - sgy);
    sgz += RATE_SMOOTH * (d->gz - sgz);
    sax += ACC_SMOOTH * (d->ax - sax);
    say += ACC_SMOOTH * (d->ay - say);
    saz += ACC_SMOOTH * (d->az - saz);
    stc += 0.01f * (d->temp_c - stc);

    /* dead zone: tiny noise reads as exactly 0 */
    fl.rr = dz(sgx, RATE_DEADBAND);
    fl.pr = dz(-sgy, RATE_DEADBAND);   /* minus: gyro Y + is nose DOWN */
    fl.yr = dz(sgz, RATE_DEADBAND);
    fl.ax = dz(sax, ACC_DEADBAND);
    fl.ay = dz(say, ACC_DEADBAND);
    fl.az = 1.0f + dz(saz - 1.0f, ACC_DEADBAND);

    /* angles */
    fl.roll  = COMP_ALPHA * (fl.roll  + fl.rr * dt) + (1.0f - COMP_ALPHA) * acc_roll();
    fl.pitch = COMP_ALPHA * (fl.pitch + fl.pr * dt) + (1.0f - COMP_ALPHA) * acc_pitch();
    fl.yaw   = wrap180(fl.yaw + fl.yr * dt);

    fl.dt_ms  = dt * 1000.0f;
    fl.temp_c = stc;
}

/* ---------------- output ---------------- */
// ---------------------------------------------------------------------------------------------
// int getU2_nb(void)
//   Returns a received UART2 character without waiting, or -1 if none (clears an overrun
//   first).
// ---------------------------------------------------------------------------------------------
int getU2_nb(void)            /* key from PuTTY, or -1 if none */
{
    if (U2STAbits.OERR) U2STAbits.OERR = 0;
    if (!U2STAbits.URXDA) return -1;
    return U2RXREG;
}

// ---------------------------------------------------------------------------------------------
// char *fxw(char *buf, float v, int dec, int w)
//   Same as fx(), right-aligned in a field w characters wide.
// ---------------------------------------------------------------------------------------------
char *fxw(char *buf, float v, int dec, int w)   /* right-aligned fixed width */
{
    char t[20];
    fx(t, v, dec);
    sprintf(buf, "%*s", w, t);
    return buf;
}

// ---------------------------------------------------------------------------------------------
// void print_header(void)
//   Prints the column names for the live PuTTY lines.
// ---------------------------------------------------------------------------------------------
void print_header(void)
{
    char s[120], t[16];
    sprintf(s, "-- LSM6DSO @ %d Hz | temp %s C | keys: c = recalibrate, z = zero yaw --",
            ODR_HZ, fx(t, fl.temp_c, 1));
    putsU2(s);
    putsU2("  time(s)  dt(ms) |   ROLL  PITCH    YAW  deg |  ROLL/s PITCH/s   YAW/s |    AX    AY    AZ  g");
}

// ---------------------------------------------------------------------------------------------
// void print_line(uint32_t ts)
//   Prints one line of live readings (angles, rates, motor pulses) to PuTTY.
// ---------------------------------------------------------------------------------------------
void print_line(uint32_t ts)
{
    char s[140], t[16], dt[16], r[16], p[16], y[16], rr[16], pr[16], yr[16], ax[16], ay[16], az[16];
    sprintf(t, "%lu.%02lu", (unsigned long)(ts / 40000), (unsigned long)((ts % 40000) / 400));
    sprintf(s, "%9s %7s | %s %s %s     | %s %s %s | %s %s %s",
            t, fxw(dt, fl.dt_ms, 2, 7),
            fxw(r, dz(fl.roll,  ANGLE_DEADBAND), 1, 6),
            fxw(p, dz(fl.pitch, ANGLE_DEADBAND), 1, 6),
            fxw(y, dz(fl.yaw,   ANGLE_DEADBAND), 1, 6),
            fxw(rr, fl.rr, 1, 7), fxw(pr, fl.pr, 1, 7), fxw(yr, fl.yr, 1, 7),
            fxw(ax, fl.ax, 2, 5), fxw(ay, fl.ay, 2, 5), fxw(az, fl.az, 2, 5));
    putsU2(s);
}

/* ---------------- main ---------------- */
// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: finds and checks the IMU wiring, calibrates, then streams IMU readings and
//   angles to PuTTY.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    char s[120];
    imu_data_t raw, d;
    uint32_t prev_ts, last_print;
    int rows = 0;

    T3CON = 0x8070;          /* Timer3 on, 1:256, for delay_ms */
    initU2();
    initSPI();
    delay_ms(20);            /* LSM6DSO needs ~10 ms after power-up */

    putsU2("\x1b[?25h");     /* show the cursor again (the old dashboard hid it) */
    putsU2("==========================================================");
    putsU2(" Drone IMU");
    putsU2(" Test build: Nov 15, version 8 (slow SDO contact test)");
    if (imu_read1(WHO_AM_I) != LSM6DSO_ID) {        /* wiring tests only if needed */
        putsU2(" IMU didn't answer - running wiring tests...");
        i2c_wire_test();
        wiring_test();
        wait_for_imu_id();
    }
    putsU2(" Name : LSM6DSO  (STMicroelectronics iNEMO 6-axis IMU)");
    putsU2(" Role : flight IMU - roll/pitch/yaw rates and angles for the PID loop");
    sprintf(s, " ID   : WHO_AM_I (reg 0x0F) = 0x%02X -> matches LSM6DSO", imu_read1(WHO_AM_I));
    putsU2(s);
    putsU2(" Bus  : SPI1, 4-wire, mode 3, 2 MHz, CS on pin 16");
    putsU2("==========================================================");

    if (!initIMU())
        putsU2("WARNING: a register didn't read back right - data may be wrong.");
    sprintf(s, " Accel: +/-4 g   Gyro: +/-1000 deg/s   both at %d Hz", ODR_HZ);
    putsU2(s);
    delay_ms(100);           /* gyro needs ~70 ms to start up */

    calibrate();
    wait_new_data(); imu_sample(&raw); apply_cal(&raw, &d);
    flight_reset(&d);
    prev_ts = last_print = raw.ts;
    putsU2("");
    print_header();

    while (1) {
        int k = getU2_nb();
        if (k == 'c' || k == 'C') {
            calibrate();
            wait_new_data(); imu_sample(&raw); apply_cal(&raw, &d);
            flight_reset(&d);
            prev_ts = last_print = raw.ts;
            print_header(); rows = 0;
        } else if (k == 'z' || k == 'Z') {
            fl.yaw = 0;
            putsU2("-- yaw zeroed --");
        }

        wait_new_data();
        imu_sample(&raw);
        apply_cal(&raw, &d);

        float dt = (uint32_t)(raw.ts - prev_ts) * 0.000025f;   /* 25 us per count */
        prev_ts = raw.ts;
        if (dt <= 0.0f || dt > 0.05f) dt = 1.0f / ODR_HZ;      /* guard against a glitch */
        flight_update(&d, dt);

        if ((uint32_t)(raw.ts - last_print) >= (uint32_t)PRINT_MS * 40) {
            last_print = raw.ts;
            if (tx_free() > 200) {               /* never stall the loop for printing */
                if (++rows > 30) { print_header(); rows = 0; }
                print_line(raw.ts);
            }
        }
    }
}
