/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/Drone.X/control.h
 *  What    : Shared flight state (axis, throttle, roll and pitch gains and targets, RC, PC, the
 *            4 pulses) and the control-loop functions.
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
 *    initControl()  -  Sets up Timer4 to interrupt every 4 ms; that interrupt runs the
 *        whole flight loop.
 *    pid_reset()  -  Clears both PIDs' integrals so an old error does not carry into the
 *        next run.
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
 *        clears the roll and pitch zero offsets, then resumes the loop.
 *    graph_angle()  -  The angle the tuner graphs: roll in Roll-only mode, pitch otherwise.
 *    graph_setpoint()  -  The target the tuner graphs: the roll target in Roll-only mode,
 *        the pitch target otherwise.
 *    ramp_done()  -  1 once the soft start has brought the throttle all the way up to T.
 * ============================================================================================ */

#ifndef CONTROL_H
#define CONTROL_H

// Two PID controllers (professor's plan):
//   PID 1 = ROLL  -> roll compensation  RC (us)
//   PID 2 = PITCH -> pitch compensation PC (us)
// Every motor gets the desired throttle T plus its share of RC and PC:
//   ESC 1 front-left  = T + PC + RC        ESC 2 front-right = T + PC - RC
//   ESC 3 rear-left   = T - PC + RC        ESC 4 rear-right  = T - PC - RC
// Tune ONE axis at a time (axis_mode 1 or 2: the other compensation is 0, like the seesaw),
// then run both together (axis_mode 3): then all 4 pulses are different.

// set by main() and the tuner, used by the 4 ms interrupt
extern volatile float Kp_roll, Ki_roll, Kd_roll;        // PID 1: roll gains
extern volatile float Kp_pitch, Ki_pitch, Kd_pitch;     // PID 2: pitch gains
extern volatile float sp_roll, sp_pitch;    // target angles, degrees (0 = level)
extern volatile int axis_mode;              // 1 = ROLL only, 2 = PITCH only, 3 = BOTH
extern volatile int motor_mode;             // 0 = OFF (1000 us), 1 = PID, 2 = manual. Starts at 0.
extern volatile int kill_latched;           // 1 = disarmed: relay open, no motor may run until ARM
extern volatile int control_ready;
extern volatile int control_link_source;    // who armed: 0 = nobody, 1 = phone (Bluetooth), 2 = PC (USB)
extern volatile float throttle;             // desired throttle T, us (same for all 4 motors)
extern volatile float ramp_rate;            // soft start: how fast the throttle may change, us per second
extern volatile float man_us[5];            // manual mode pulse for ESC 1-4 (index 1..4), us
extern volatile float roll_offset, pitch_offset;    // Z (zero angle) moves these
extern volatile float eint_roll, eint_pitch;        // the two PIDs' running integrals

// made by the interrupt every 4 ms
extern volatile float ang_roll, ang_pitch;  // roll - roll_offset, pitch - pitch_offset, degrees
extern volatile float roll_comp, pitch_comp;        // RC and PC: the two PID outputs, us
extern volatile float p_term, i_term, d_term;       // P, I, D of the graphed axis (roll in ROLL mode, else pitch)
extern volatile float thr_now;              // the throttle after the soft start (climbs to T at ramp_rate)
extern volatile float motor_us[5];          // pulses really going to ESC 1-4 (index 1..4), us
extern volatile int off_reason;             // why the interrupt turned the motors off (main prints it)
extern volatile unsigned long loop_count;   // +1 every 4 ms
extern volatile float loop_period_us;       // time between the last two loops (should be 4000)
extern volatile float loop_busy_us;         // how long one loop takes

#define TIP_LIMIT 60.0f     // degrees, + and -: past this the motors go off (ARM also needs to be inside it)

// off_reason values
#define OFF_SW1    1
#define OFF_IMU    2
#define OFF_TIP    3
#define OFF_REMOTE 4
#define OFF_BLE    5    // heartbeat lost, phone disconnected, or the Bluefruit stopped answering

void initControl(void);     // starts the 4 ms loop
void pid_reset(void);       // both integrals back to 0 (no windup carried into the next run)
void motors_off(void);
int control_arm(int source);         // ARM from the phone (1) or the PC (2). 1 = armed, 0 = refused
int control_can_start(void);        // 1 = armed, ESCs started, heartbeat fresh: motors may run
void control_heartbeat(int source); // an "H" arrived from the phone (1) or the PC (2)
void control_pause(void);   // motors off, then stop the loop (calibration reads the IMU itself)
void control_resume(void);
void calibrate_now(void);   // motors off, 2 s calibration (drone still and level), loop back on
float graph_angle(void);    // angle of the graphed axis (roll in ROLL mode, else pitch)
float graph_setpoint(void); // its setpoint
int ramp_done(void);        // 1 = the soft start has reached the throttle T

#endif
