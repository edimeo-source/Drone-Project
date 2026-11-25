/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v1.1_ble_review_fixes/Drone.X/bluefruit.h
 *  What    : Bluetooth functions and shared link state.
 *  Version : v1.1   |   Date: 2026-11-20
 *  Status  : Host tests pass. Not built with XC32.
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
 *    Bluefruit SPI Friend (shares SPI1 with the IMU)
 *      VIN pin 1 (3.3 V, shared with the IMU)   GND pin 6   SCK pin 23   MOSI pin 37   MISO
 *      pin 7
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    initBluetooth()  -  Starts the Bluefruit driver: CS pin high, queues cleared, link
 *        marked down, drone disarmed.
 *    bluetooth_service()  -  Called from main() every pass: does one small step of
 *        Bluetooth work (never blocks).
 *    bluetooth_link_ok()  -  1 if the Bluefruit is working, a phone is connected, and the
 *        link was checked recently.
 *    bluetooth_send_text()  -  Queues a status line or reply for the phone (waits in the
 *        8-line queue until there is room).
 *    bluetooth_send_live()  -  Queues a live telemetry line only if nothing else is
 *        waiting; otherwise drops it.
 * ============================================================================================ */

#ifndef BLUEFRUIT_H
#define BLUEFRUIT_H

// Bluefruit LE SPI Friend on the shared SPI1 (pins 23/37/7), CS = RJ13 (J12 pin 13).
// Powered from J12 pin 1 (3.3 V) and pin 6 (GND) through the sensor hub. IRQ/RST not connected.

extern volatile int ble_ready;
extern volatile int ble_connected;
extern volatile int ble_arm_ready;
extern volatile unsigned int ble_last_link;
extern volatile int ble_notice;

void initBluetooth(void);
void bluetooth_service(void);       /* call frequently from main; one short SDEP step */
int bluetooth_link_ok(void);
int bluetooth_send_text(const char *text); /* status line or reply: queued (8 lines), sent in order */
int bluetooth_send_live(const char *text); /* live telemetry: only if nothing is queued; dropped when the module is full */

#endif
