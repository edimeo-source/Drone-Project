/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/Drone.X/motors.h
 *  What    : Pulse limits (1000/2000 us), the per-motor trim #defines and the motor functions.
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
 *  FUNCTIONS DECLARED HERE
 *    initMotors()  -  Sets up Timer2 (50 Hz) and the output-compare pins that send the ESC
 *        pulses (all start at 1000 us = stop).
 *    set_motor()  -  Sets ESC n (1-4): forced to stop unless armed and running, then the
 *        motor's trim, then held inside 1000-2000 us.
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

// PER-MOTOR TRIMS from 01_bench_tests/11_motor_calibration_test (it prints these 8 lines).
// Paste its numbers here so every motor pushes the same for the same pulse:
//   pulse sent = 1000 + TRIM_GAIN x (pulse - 1000) + TRIM_OFFSET      (only above 1000 us)
// 1.0 / 0.0 = no trim (the motors are used as they are).
#ifndef TRIM_GAIN_FL                // (lets the PC tests try other trims)
#define TRIM_GAIN_FL   1.0000f
#define TRIM_OFFSET_FL 0.0f
#define TRIM_GAIN_FR   1.0000f
#define TRIM_OFFSET_FR 0.0f
#define TRIM_GAIN_RL   1.0000f
#define TRIM_OFFSET_RL 0.0f
#define TRIM_GAIN_RR   1.0000f
#define TRIM_OFFSET_RR 0.0f
#endif

#define PULSE_OFF  1000.0f  // us, motor stopped (also what an ESC needs to arm)
#define PULSE_IDLE 1100.0f  // us, slowest a motor goes while the PID runs (keeps the props turning)
#define PULSE_MAX  2000.0f  // us, full throttle

void initMotors(void);              // all 4 outputs start at 1000 us so the ESCs arm
void set_motor(int n, float us);    // n = 1..4 (ESC number), trims added, kept inside 1000..2000 us

#endif
