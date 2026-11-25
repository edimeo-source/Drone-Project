/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/Drone.X/pid.c
 *  What    : PID math (one step, with anti-windup).
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
 *  FUNCTIONS IN THIS FILE (each has a title block above it): pid()
 * ============================================================================================ */

# include "pid.h"

// PID the way Kevin Lynch writes it (Embedded Computing and Mechatronics with
// the PIC32, Ch. 23; Modern Robotics 11.4):
//
//   e    = r - s              error = reference - sensed
//   eint = eint + e*dt        running integral of the error
//   edot = rdot - sdot        rate of the error (here from the gyro, not by subtracting)
//   u    = Kp*e + Ki*eint + Kd*edot
//
// It runs once every dt = 4 ms (the timer interrupt), so dt is a constant.
// eint lives in the caller, one per axis, passed in by pointer.
// Anti-windup: eint is clamped to +/-eintmax. The output u is clamped to +/-umax.
// This is the same math as the PID tuner's demo seesaw.

#define DT 0.004f

// ---------------------------------------------------------------------------------------------
// float pid(float e, float edot, float Kp, float Ki, float Kd, float *eint, float eintmax,
//     float umax)
//   One PID step: u = Kp*e + Ki*integral(e) + Kd*edot, with anti-windup and an output limit.
// ---------------------------------------------------------------------------------------------
float pid(float e, float edot, float Kp, float Ki, float Kd,
          float *eint, float eintmax, float umax)
{
    float u;

    *eint=*eint+e*DT;                   // integrate the error
    if (*eint> eintmax) *eint= eintmax; // anti-windup
    if (*eint<-eintmax) *eint=-eintmax;

    u=Kp*e+Ki*(*eint)+Kd*edot;          // the PID law

    if (u> umax) u= umax;               // saturate the output
    if (u<-umax) u=-umax;
    return u;
}
