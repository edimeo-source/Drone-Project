/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v2.0_phone_app_relay_heartbeat/Drone.X/control.h
 *  What    : Shared flight state (armed, setpoint, gains, motor pulses) and the control-loop
 *            functions.
 *  Version : v2.0   |   Date: 2026-11-23
 *  Status  : CURRENT. Builds in MPLAB X with XC32 and was flown on the tethered bench (Nov 24).
 *            Host tests pass. The copy on the laptop has TIP_LIMIT 60 (this repo copy: 75).
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
 *    initControl()  -  Sets up Timer4 to interrupt every 4 ms; that interrupt runs the
 *        whole flight loop.
 *    pid_reset()  -  Clears the PID's integral so an old error does not carry into the next
 *        run.
 *    motors_off()  -  KILL: every ESC to 1000 us, latch the kill, and (with a relay) open
 *        the motor-power relay.
 *    control_arm()  -  ARM: safety checks, 1000 us to every ESC, then the relay closes so
 *        the ESCs power up seeing 'stop'.
 *    control_can_start()  -  1 if a run may start now: armed, IMU OK, ESC start-up time
 *        over, heartbeat fresh, SW1 not pressed.
 *    control_heartbeat()  -  Records a heartbeat from the link that armed the drone (a
 *        stale heartbeat stops the motors).
 *    control_pause()  -  Motors off and the 4 ms loop stopped (used while calibrating or
 *        printing reports).
 *    control_resume()  -  Clears the PID and restarts the 4 ms loop.
 *    calibrate_now()  -  Recalibrates the IMU with the motors off (PuTTY key c or SW2),
 *        then resumes the loop.
 * ============================================================================================ */

#ifndef CONTROL_H
#define CONTROL_H

// set by main() and the tuner, used by the 4 ms interrupt
extern volatile float Kp, Ki, Kd;
extern volatile float setpoint;             // pitch the drone should hold, degrees (0 = level)
extern volatile int motor_mode;             // 0 = OFF (1000 us), 1 = PID, 2 = manual. Starts at 0.
extern volatile int kill_latched;           // 1 = disarmed: relay open, no motor may run until ARM
extern volatile int control_ready;
extern volatile int control_link_source;    // who armed: 0 = nobody, 1 = phone (Bluetooth), 2 = PC (USB)
extern volatile float base_front, base_back;    // PID base pulse of the front pair and back pair, us
extern volatile float man_front, man_back;      // manual mode pulse of the front pair and back pair, us
extern volatile float angle_offset;         // Z (zero angle) moves this
extern volatile float eint;                 // the PID's running integral (pid_reset() clears it)

// made by the interrupt every 4 ms
extern volatile float angle;                // pitch - angle_offset, degrees (+ = nose up)
extern volatile float u;                    // PID output, us
extern volatile float p_term, i_term, d_term;
extern volatile float motor_front, motor_back;  // pulses really going to the ESCs, us
extern volatile float ramp_us;              // soft start limit, climbs 1000 -> 2000 us in 1 s
extern volatile int off_reason;             // why the interrupt turned the motors off (main prints it)
extern volatile unsigned long loop_count;   // +1 every 4 ms
extern volatile float loop_period_us;       // time between the last two loops (should be 4000)
extern volatile float loop_busy_us;         // how long one loop takes

// off_reason values
#define OFF_SW1    1
#define OFF_IMU    2
#define OFF_TIP    3
#define OFF_REMOTE 4
#define OFF_BLE    5    // heartbeat lost, phone disconnected, or the Bluefruit stopped answering

void initControl(void);     // starts the 4 ms loop
void pid_reset(void);       // integral back to 0 (no windup carried into the next run)
void motors_off(void);
int control_arm(int source);         // ARM from the phone (1) or the PC (2). 1 = armed, 0 = refused
int control_can_start(void);        // 1 = armed, ESCs started, heartbeat fresh: motors may run
void control_heartbeat(int source); // an "H" arrived from the phone (1) or the PC (2)
void control_pause(void);   // motors off, then stop the loop (calibration reads the IMU itself)
void control_resume(void);
void calibrate_now(void);   // motors off, 2 s calibration (drone still and level), loop back on

#endif
