#!/bin/sh
# Host (PC) tests of the drone firmware with stand-in PIC32 registers and a simulated
# Bluefruit. Needs gcc (Linux / macOS / WSL). Not a substitute for the XC32 build or the bench test.
cd "$(dirname "$0")"
S=../Drone.X; ST=stub
set -e
gcc -w -I$ST -I$S -c $S/uart.c -DputsU2=real_putsU2 -o uart_real.o
gcc -w -Dstatic= -I$ST -I$S -c $S/tuner.c -o tuner_open.o
gcc -w -I$ST -I$S sim_ble.c $S/control.c $S/motors.c tuner_open.o $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c -lm -o sim_ble
for t in fw_test v12_test; do
  gcc -w -I$ST -I$S $t.c $S/control.c $S/motors.c stubs_ble.c $S/tuner.c $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c -lm -o $t
done
echo "--- Bluetooth driver vs a simulated Bluefruit ---"; ./sim_ble
echo "--- arming, relay, heartbeat, motor output, PC data ---"; ./fw_test
echo "--- 75 deg cutoff, end of a recorded run ---"; ./v12_test
rm -f *.o sim_ble fw_test v12_test
