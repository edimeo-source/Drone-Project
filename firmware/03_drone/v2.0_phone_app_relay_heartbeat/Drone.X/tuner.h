/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/Drone.X/tuner.h
 *  What    : Tuner functions and the shared step-test state.
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
 *    tuner_command()  -  Runs one command line from the PID tuner (K gains, S setpoint, R
 *        step test, G, M, B, X, Z, ...).
 *    tuner_set_ble_output()  -  Chooses where tuner replies go: 1 = phone (Bluetooth), 0 =
 *        PC (USB).
 *    tuner_reply()  -  Sends a reply to whichever link sent the command (phone or PC).
 *    tuner_help()  -  Sends the list of tuner commands.
 *    tuner_send_live()  -  Sends one live data line (D,...) to the tuner graph.
 *    tuner_send_run()  -  Sends the recorded step test to the PC, one line per call, so
 *        commands still work meanwhile.
 *    tuner_send_run_phone()  -  Sends the recorded step test to the phone, only when the
 *        Bluetooth queue has room.
 *    tuner_run_is_phone()  -  1 if the step test in progress was started from the phone.
 *    round_i()  -  Rounds a float to the nearest int.
 *    tuner_setpoint_step()  -  Runs every 4 ms: holds the start angle, then makes the
 *        setpoint step for a step test.
 *    tuner_record()  -  Runs every 4 ms during a step test: stores angle, setpoint, motor
 *        pulses and P/I/D terms.
 * ============================================================================================ */

#ifndef TUNER_H
#define TUNER_H

extern volatile int display_on;     // 1 = PuTTY data lines
extern volatile int live_on;        // 1 = D lines for the PID tuner (never while recording)
extern volatile int run_state;      // step test: 0 = none, 1 = holding the start angle, 2 = recording
extern volatile int data_complete;  // professor's flag: 1 = the recording is full, main() sends it
extern volatile int run_stopped;    // 1 = the motors went off during a step test, main() ends it

void tuner_command(char *line);     // one line from the tuner, like "K,2.5,0,0.4"
void tuner_set_ble_output(int enabled);     // 1 = replies go to the phone (set around a phone command)
void tuner_reply(const char *text);         // reply to whichever link sent the command
void tuner_help(void);              // list the tuner commands
void tuner_send_live(void);         // one D line (main, every 8 ms while live and NOT recording)
void tuner_send_run(void);          // main: send the recording to the PC, ONE line per call (while data_complete = 1)
void tuner_send_run_phone(void);    // main: send the recording to the phone, 8 samples per line
int tuner_run_is_phone(void);       // 1 = the last step test came from the phone
int round_i(float v);               // nearest whole number (for the short phone lines)
void tuner_setpoint_step(void);     // from the 4 ms interrupt, before the PID
void tuner_record(void);            // from the 4 ms interrupt, after the PID: data[i] = ...

#endif
