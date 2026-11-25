/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/02_seesaw/02_SewSaw_step0-1/SewSaw.X/tuner.h
 *  What    : Tuner functions and the shared step-test state.
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
 *    tuner_command()  -  Runs one command line from the PID tuner (K gains, S setpoint, R
 *        step test, G, M, B, X, Z, ...).
 *    tuner_send_live()  -  Sends one live data line (D,...) to the tuner graph.
 *    tuner_send_run()  -  Sends the recorded step test to the PC, one line per call, so
 *        commands still work meanwhile.
 *    tuner_setpoint_step()  -  Runs every 4 ms: holds the start angle, then makes the
 *        setpoint step for a step test.
 *    tuner_record()  -  Runs every 4 ms during a step test: stores angle, setpoint, motor
 *        pulses and P/I/D terms.
 * ============================================================================================ */

#ifndef TUNER_H
#define TUNER_H

extern volatile int display_on;     // 1 = PuTTY data lines
extern volatile int live_on;        // 1 = D lines for the PID tuner
extern volatile int run_state;      // step test: 0 none, 1 holding, 2 recording, 3 done (send it)

void tuner_command(char *line);     // one line from the tuner, like "K,2.5,0,0.4"
void tuner_send_live(void);         // one D line (main, every 8 ms while live)
void tuner_send_run(void);          // the recorded step test (main, when run_state = 3)
void tuner_setpoint_step(void);     // from the 4 ms interrupt, before the PID
void tuner_record(void);            // from the 4 ms interrupt, after the PID

#endif
