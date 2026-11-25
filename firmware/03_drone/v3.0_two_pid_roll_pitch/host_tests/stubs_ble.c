/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/host_tests/stubs_ble.c
 *  What    : Host test: empty Bluetooth functions for the tests that do not use Bluetooth.
 *  Version : v3.0   |   Date: 2026-11-25
 *  Status  : CURRENT. Built from v2.0 (which builds and flew on the tether). Host tests pass
 *            (78 checks, including a simulated flight with the real 4 ms loop). Not yet built
 *            in MPLAB or flown.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): ble_service(), ble_get_line(),
 *    ble_print(), ble_room()
 * ============================================================================================ */

// The firmware tests that don't need Bluetooth link this instead of bluefruit.c
volatile int ble_ok=0, ble_connected=0;
// ---------------------------------------------------------------------------------------------
// void ble_service(void)
//   Called from main() every pass: checks the link, reads phone text, sends queued text
//   (retries every 1 s if the module failed).
// ---------------------------------------------------------------------------------------------
void ble_service(void) {}
// ---------------------------------------------------------------------------------------------
// int ble_get_line(char *line)
//   Copies the next complete command the phone sent into line. 1 = got one.
// ---------------------------------------------------------------------------------------------
int ble_get_line(char *line) { (void)line; return 0; }
// ---------------------------------------------------------------------------------------------
// int ble_print(const char *text)
//   Queues a text line to send to the phone (does nothing if no phone is connected).
// ---------------------------------------------------------------------------------------------
int ble_print(const char *text) { (void)text; return 0; }
// ---------------------------------------------------------------------------------------------
// int ble_room(void)
//   Returns how many characters still fit in the phone send buffer.
// ---------------------------------------------------------------------------------------------
int ble_room(void) { return 1024; }
