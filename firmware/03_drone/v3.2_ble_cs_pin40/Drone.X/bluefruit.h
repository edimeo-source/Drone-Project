/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.2_ble_cs_pin40/Drone.X/bluefruit.h
 *  What    : Bluetooth functions and shared link state.
 *  Version : v3.2   |   Date: 2026-11-26
 *  Status  : CURRENT. v3.1 with the Bluefruit CS moved from pin 13 (RJ13) to pin 40 (RJ3);
 *            nothing else changed. Host tests pass (87 checks; the Bluetooth simulation checks
 *            CS on RJ3 for every byte). Not yet built in MPLAB.
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
 *      SCK pin 23   MOSI pin 37   MISO pin 7   CS pin 40 (RJ3, moved from pin 13 in v3.2)
 *      IRQ and RST not connected
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    ble_service()  -  Called from main() every pass: checks the link, reads phone text,
 *        sends queued text (retries every 1 s if the module failed).
 *    ble_get_line()  -  Copies the next complete command the phone sent into line. 1 = got
 *        one.
 *    ble_print()  -  Queues a text line to send to the phone (does nothing if no phone is
 *        connected).
 *    ble_room()  -  Returns how many characters still fit in the phone send buffer.
 * ============================================================================================ */

#ifndef BLUEFRUIT_H
#define BLUEFRUIT_H

// Adafruit Bluefruit LE SPI Friend on SPI1, shared with the IMU.
// Wiring (Nov 23): SCK -> J12 pin 23, MOSI -> pin 37, MISO -> pin 7 (each one is
// a splitter wire shared with the IMU), CS -> pin 40 (RJ3, v3.2; was pin 13), VIN -> pin 2 (+5 V),
// GND -> any kit ground. IRQ, RST and DFU are not connected.

extern volatile int ble_ok;         // 1 = the module answers AT commands
extern volatile int ble_connected;  // 1 = a phone is connected to it

void ble_service(void);             // main loop: link check, read the phone, send queued text
int ble_get_line(char *line);       // 1 = a full command from the phone was copied into line (48 chars)
int ble_print(const char *text);    // queue one line for the phone ('\n' added). 0 = no room
int ble_room(void);                 // free bytes in the send queue

#endif
