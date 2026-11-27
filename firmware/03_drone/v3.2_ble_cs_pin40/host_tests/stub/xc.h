/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.2_ble_cs_pin40/host_tests/stub/xc.h
 *  What    : Host test support: stand-in for <xc.h> so the firmware compiles with gcc.
 *  Version : v3.2   |   Date: 2026-11-26
 *  Status  : CURRENT. v3.1 with the Bluefruit CS moved from pin 13 (RJ13) to pin 40 (RJ3);
 *            nothing else changed. Host tests pass (87 checks; the Bluetooth simulation checks
 *            CS on RJ3 for every byte). Not yet built in MPLAB.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ============================================================================================ */

#include "p32xxxx.h"
