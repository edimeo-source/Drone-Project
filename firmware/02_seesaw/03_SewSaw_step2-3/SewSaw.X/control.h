/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/03_SewSaw_step2-3/SewSaw.X/control.h
 *  What    : Shared flight state (armed, setpoint, gains, motor pulses) and the control-loop
 *            functions.
 *  Version : SewSaw v2   |   Date: 2026-11-16
 *  Status  : Written and handed over; the seesaw stage was marked done Nov 18 (no separate
 *            hardware log).
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC 1 (motor A) ... signal J12 pin 11 (RD0 -> OC1)   ground pin 14
 *    ESC 2 (motor B) ... signal J12 pin 12 (RF2 -> OC2)   ground pin 20
 *    ESC red 5 V wires . NOT connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    initControl()  -  Sets up Timer4 to interrupt every 4 ms; that interrupt runs the
 *        whole flight loop.
 *    motors_off()  -  KILL: every ESC to 1000 us, latch the kill, and (with a relay) open
 *        the motor-power relay.
 *    control_pause()  -  Motors off and the 4 ms loop stopped (used while calibrating or
 *        printing reports).
 *    control_resume()  -  Clears the PID and restarts the 4 ms loop.
 * ============================================================================================ */

#ifndef CONTROL_H
#define CONTROL_H

// set by main() and the tuner, used by the 4 ms interrupt
extern volatile float Kp, Ki, Kd;
extern volatile float setpoint;             // angle the seesaw should hold, degrees (0 = level)
extern volatile int motor_mode;             // 0 = OFF (1000 us), 1 = PID, 2 = manual. Starts at 0.
extern volatile float base_a, base_b;       // PID base pulse of motor A and B, us
extern volatile float man_a, man_b;         // manual mode pulse of motor A and B, us
extern volatile float angle_offset;         // Z (zero angle) moves this

// made by the interrupt every 4 ms
extern volatile float angle;                // seesaw angle = roll - angle_offset, degrees
extern volatile float u;                    // PID output, us
extern volatile float p_term, i_term, d_term;
extern volatile float motor_a, motor_b;     // the pulses really going to the ESCs, us
extern volatile float ramp_us;              // soft start limit, climbs 1000 -> 2000 us in 1 s
extern volatile int off_reason;             // why the interrupt turned the motors off (main prints it)
extern volatile unsigned long loop_count;   // +1 every 4 ms
extern volatile float loop_period_us;       // time between the last two loops (should be 4000)
extern volatile float loop_busy_us;         // how long one loop takes

// off_reason values
#define OFF_SW1   1         // SW1 kill button
#define OFF_IMU   2         // the IMU stopped answering
#define OFF_TIP   3         // angle past the tip limit

void initControl(void);     // starts the 4 ms loop
void motors_off(void);      // motor_mode = 0 and both ESCs to 1000 us right now
void control_pause(void);   // motors off, then stop the loop (calibration reads the IMU itself)
void control_resume(void);

#endif
