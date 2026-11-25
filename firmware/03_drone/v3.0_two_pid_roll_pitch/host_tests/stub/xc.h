/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/host_tests/stub/xc.h
 *  What    : Host test support: stand-in for <xc.h> so the firmware compiles with gcc.
 *  Version : v3.0   |   Date: 2026-11-25
 *  Status  : CURRENT. Built from v2.0 (which builds and flew on the tether). Host tests pass
 *            (78 checks, including a simulated flight with the real 4 ms loop). Not yet built
 *            in MPLAB or flown.
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ============================================================================================ */

#include "p32xxxx.h"
