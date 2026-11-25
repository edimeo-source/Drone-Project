#!/bin/sh
# Host (PC) tests of the firmware logic with stand-in PIC32 registers. Needs gcc.
# Not a substitute for an XC32 build or bench testing.
cd "$(dirname "$0")"
S=../Drone.X; ST=stub
set -e
gcc -w -I$ST -I$S -c $S/uart.c -DputsU2=real_putsU2 -o uart_real.o
gcc -w -I$ST -I$S fw_test.c $S/control.c $S/motors.c $S/bluefruit.c $S/tuner.c $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c -lm -o fw_test
gcc -w -I$ST -I$S v12_test.c $S/control.c $S/motors.c $S/bluefruit.c $S/tuner.c $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c -lm -o v12_test
gcc -w -Dstatic= -I$ST -I$S -c $S/bluefruit.c -o bluefruit_open.o
gcc -w -I$ST -I$S ble_test.c bluefruit_open.o $S/control.c $S/motors.c $S/tuner.c $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c -lm -o ble_test
echo "--- firmware logic ---"; ./fw_test
echo "--- Bluetooth send path ---"; ./ble_test
echo "--- 75 deg cutoff, end of a recorded run ---"; ./v12_test
rm -f *.o fw_test ble_test v12_test
