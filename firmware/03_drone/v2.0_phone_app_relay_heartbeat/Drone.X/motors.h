/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/Drone.X/motors.h
 *  What    : Pulse limits (1000/2000 us) and the motor functions.
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
 *  FUNCTIONS DECLARED HERE
 *    initMotors()  -  Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC
 *        pulses (all start at 1000 us = stop).
 *    set_motor()  -  Sets the ESC pulse for motor n (1-4); forced to 'stop' unless the
 *        drone is armed and running.
 * ============================================================================================ */

#ifndef MOTORS_H
#define MOTORS_H

// ESC pulses on Output Compare 1-4, all on Timer2 (50 Hz, the rate your ESCs already ran at).
// Looking down on the drone. TRUE FRONT = the end opposite the battery pads, IMU X arrow = front,
// white arms = left, red arms = right:
//
//              FRONT
//   ESC 1 (FL, white)   ESC 2 (FR, red)     ESC 1 = J12 pin 11 (RD0 = OC1), ground pin 14
//              \         /                  ESC 2 = J12 pin 12 (RF2 = OC2), ground pin 20
//               [ PIC ]                     ESC 3 = J12 pin 21 (RF4 = OC3), ground pin 25
//              /         \                  ESC 4 = J12 pin 19 (RF5 = OC4), ground pin 30
//   ESC 3 (RL, white)   ESC 4 (RR, red)
//              BACK (battery pads)
//
// Motor-power relay: RK4 = J12 pin 33 -> relay module CH1 (VREF jumpered to DC-, so CH1 is
// active-HIGH). Same as the tested main_bluefruit_relay_test.c:
//   RK4 HIGH = relay on  = battery reaches the ESCs (only after ARM)
//   RK4 LOW  = relay off = motor power CUT (power-up, reset, every stop)

#define PULSE_OFF  1000.0f  // us, motor stopped (also what an ESC needs to arm)
#define PULSE_IDLE 1100.0f  // us, slowest a motor goes while the PID runs (keeps the props turning)
#define PULSE_MAX  2000.0f  // us, full throttle

void initMotors(void);              // all 4 outputs start at 1000 us so the ESCs arm
void set_motor(int n, float us);    // n = 1..4 (ESC number), us is kept inside 1000..2000

#endif
