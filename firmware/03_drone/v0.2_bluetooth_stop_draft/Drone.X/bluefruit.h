/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v0.2_bluetooth_stop_draft/Drone.X/bluefruit.h
 *  What    : Bluetooth functions and shared link state.
 *  Version : v0.2 draft   |   Date: 2026-11-20
 *  Status  : Draft source only (no MPLAB project files). Merged into v1.0.
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
 *      VIN pin 2 (+5 V)   GND pin 9   SCK pin 23   MOSI pin 37   MISO pin 7
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    PC serial (PuTTY / PID tuner) . UART2 over the J11 USB port, 115200 baud
 *      pins 8 (RG6) and 10 (RB14) are the PC link: never wire them, never press SW3
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS DECLARED HERE
 *    initBluetooth()  -  Starts the Bluefruit driver: CS pin high, queues cleared, link
 *        marked down, drone disarmed.
 *    bluetooth_service()  -  Called from main() every pass: does one small step of
 *        Bluetooth work (never blocks).
 *    bluetooth_watchdog()  -  Stops the motors if the Bluetooth link is lost (runs even if
 *        main is stuck printing).
 *    bluetooth_link_ok()  -  1 if the Bluefruit is working, a phone is connected, and the
 *        link was checked recently.
 * ============================================================================================ */

#ifndef BLUEFRUIT_H
#define BLUEFRUIT_H

// J12_PINOUT.html: shared SPI1 on 23/37/7, CS on 13. IRQ and RST stay unconnected.
extern volatile int ble_ready;
extern volatile int ble_connected;
extern volatile int ble_arm_ready;     // old RX text has been drained; a fresh ARM; may be accepted
extern volatile unsigned int ble_last_link;
extern volatile int ble_notice;       // main prints notices, never the control interrupt

void initBluetooth(void);
void bluetooth_service(void);         // main, UART waits and delays: one short SPI packet at a time
void bluetooth_watchdog(void);        // 4 ms interrupt: stops on stale connection checks
int bluetooth_link_ok(void);

#endif
