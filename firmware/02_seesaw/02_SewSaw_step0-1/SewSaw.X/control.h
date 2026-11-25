/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/02_SewSaw_step0-1/SewSaw.X/control.h
 *  What    : Shared flight state (armed, setpoint, gains, motor pulses) and the control-loop
 *            functions.
 *  Version : SewSaw v1   |   Date: 2026-11-16
 *  Status  : WORKED: step 0 (IMU check) passed and step 1 ran on the board Nov 16.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    IMU LSM6DSO (SPI1 mode 3, CS on pin 16)
 *      3V3 pin 1   GND pin 6   SCL/SCK pin 23 (RD1)   SDA/MOSI pin 37 (RD10)
 *      SDO/MISO pin 7 (RA14)   CS pin 16 (RK1)
 *    ESC pins 11/12 are reserved for the seesaw motors but not driven yet (motors.c arrives
 *      in step 2-3)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    initControl()  -  Sets up Timer4 to interrupt every 4 ms; that interrupt runs the
 *        whole flight loop.
 *    control_pause()  -  Motors off and the 4 ms loop stopped (used while calibrating or
 *        printing reports).
 *    control_resume()  -  Clears the PID and restarts the 4 ms loop.
 * ============================================================================================ */

#ifndef CONTROL_H
#define CONTROL_H

// set by main() and the tuner, used by the 4 ms interrupt
extern volatile float Kp, Ki, Kd;
extern volatile float setpoint;             // angle the seesaw should hold, degrees (0 = level)
extern volatile int pid_on;                 // 0 = KILL: motor commands held at 1000 us

// made by the interrupt every 4 ms
extern volatile float u;                    // PID output, us
extern volatile float p_term, i_term, d_term;
extern volatile float motor_a, motor_b;     // seesaw motor commands, us (no motors connected yet)
extern volatile unsigned long loop_count;   // +1 every 4 ms
extern volatile float loop_period_us;       // time between the last two loops (should be 4000)
extern volatile float loop_busy_us;         // how long one loop takes

void initControl(void);     // starts the 4 ms loop
void control_pause(void);   // stop the loop (calibration reads the IMU itself)
void control_resume(void);

#endif
