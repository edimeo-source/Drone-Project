/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/00_flight_controller_draft/main.c
 *  What    : Program entry: start-up, then the main loop (PuTTY keys, tuner/phone commands,
 *            status).
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): main()
 * ============================================================================================ */

# include <p32xxxx.h>
# include <xc.h>
# include <sys/attribs.h>
# include <stdio.h>
# include <string.h>
# include "config.bits"     // same config file as Lab 6 (SYSCLK 120 MHz, PBCLK 60 MHz)
# include "delay.h"
# include "uart.h"
# include "bluetooth.h"
# include "imu.h"
# include "motors.h"
# include "control.h"

// ELC 275 capstone - self-leveling flying platform - practice code
// Ethan DiMeo
//
// main() sets everything up, then only handles the phone.
// The flying (IMU -> filter -> PID -> motors) all happens in the Timer4
// interrupt in control.c, every 4 ms.
//
// Bluefruit Connect app -> Controller -> Control Pad:
//   1 = arm          2 = KILL (motors off)
//   up = throttle +20 us     down = throttle -20 us
//   right = Kp + 0.25        left = Kp - 0.25
//   3 = Kd + 0.05            4 = Kd - 0.05
// SW1 on the kit = KILL too.
// PROPS OFF for the first tests.

#define THROTTLE_STEP   20.0f
#define THROTTLE_TOP    1850.0f     // leave room above for the PID corrections

// ---------------------------------------------------------------------------------------------
// void main (void)
//   Program entry: sets up delays, UART, Bluetooth, IMU, motors and the 4 ms loop, then handles
//   the phone's Control Pad buttons.
// ---------------------------------------------------------------------------------------------
void main (void)
{
    char s[100];
    int *d;
    int id;
    unsigned long last=0;

    initDelay();            // Timer3 first, everything else uses delays
    initMotors();           // right away, so the ESCs see a steady 1000 us
    initU2();
    initSPI();

    ANSELBbits.ANSB12=0;    // SW1 (RB12) as a digital input with a pull-up
    TRISBbits.TRISB12=1;
    CNPUBbits.CNPUB12=1;

    putsU2("Flight controller start");

    id=initIMU();
    sprintf(s, "WHO_AM_I = 0x%X (LSM6DSO should be 0x6C)", id);
    putsU2(s);
    strcat(s, "\r\n");
    ble_send(s);

    putsU2("Calibrating gyro, keep the drone still");
    imu_calibrate();
    sprintf(s, "Start angle roll %+.1f pitch %+.1f", roll, pitch);
    putsU2(s);

    initControl();          // the 4 ms control interrupt starts here

    while (1){
        // hardware kill switch
        if (PORTBbits.RB12==0){
            armed=0;
            throttle_us=PULSE_OFF;
        }

        // phone commands: ! B button '1'(pressed)
        d=u_receive();
        if (d[0]=='!' && d[1]=='B' && d[3]=='1'){
            if (d[2]=='1'){                                 // ARM, only if close to level
                if (roll<10 && roll>-10 && pitch<10 && pitch>-10 && imu_error==0){
                    throttle_us=PULSE_IDLE;
                    armed=1;
                    ble_send("ARMED\r\n");
                }
                else ble_send("not level, not armed\r\n");
            }
            if (d[2]=='2'){                                 // KILL
                armed=0;
                throttle_us=PULSE_OFF;
                ble_send("KILL\r\n");
            }
            if (d[2]=='5' && armed==1){                     // up arrow
                throttle_us=throttle_us+THROTTLE_STEP;
                if (throttle_us>THROTTLE_TOP) throttle_us=THROTTLE_TOP;
            }
            if (d[2]=='6' && armed==1){                     // down arrow
                throttle_us=throttle_us-THROTTLE_STEP;
                if (throttle_us<PULSE_IDLE) throttle_us=PULSE_IDLE;
            }
            if (d[2]=='8') Kp=Kp+0.25f;                     // right arrow
            if (d[2]=='7' && Kp>=0.25f) Kp=Kp-0.25f;        // left arrow
            if (d[2]=='3') Kd=Kd+0.05f;
            if (d[2]=='4' && Kd>=0.05f) Kd=Kd-0.05f;
        }

        // telemetry every 50 loops = 200 ms
        if (loop_count-last>=50){
            last=loop_count;
            sprintf(s, "R%+.1f P%+.1f T%d Kp%.2f Kd%.2f", roll, pitch, (int)throttle_us, Kp, Kd);
            putsU2(s);
            strcat(s, "\r\n");
            ble_send(s);
        }
        if (imu_error==1) putsU2("IMU stopped answering - motors off");

        delay_ms(20);
    }
}
