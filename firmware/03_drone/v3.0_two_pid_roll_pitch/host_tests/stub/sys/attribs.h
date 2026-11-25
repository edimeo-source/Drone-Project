/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/03_drone/v3.0_two_pid_roll_pitch/host_tests/stub/sys/attribs.h
 *  What    : Host test support: stand-in for <sys/attribs.h> (__ISR becomes nothing).
 *  Version : v3.0   |   Date: 2026-11-25
 *  Status  : Builds in MPLAB X with XC32 (built on the laptop Nov 25). The drone spinning
 *            counterclockwise on the tether led to v3.1. Host tests pass (78 checks, including
 *            a simulated flight with the real 4 ms loop).
 *  Runs on : a PC (gcc), not the PIC32. Start it with host_tests/run_tests.sh
 * ============================================================================================ */

