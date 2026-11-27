#!/bin/sh
# Host (PC) tests of the drone firmware with stand-in PIC32 registers and a simulated
# Bluefruit. Needs gcc (Linux / macOS / WSL). Not a substitute for the XC32 build or the bench test.
cd "$(dirname "$0")"
S=../Drone.X; ST=stub
set -e
export TUNER_DUMP="${TUNER_DUMP:-$PWD/fw_out}"; mkdir -p "$TUNER_DUMP"   # real firmware output, read by the PC tuner v7 and phone app 3.0 tests
gcc -w -I$ST -I$S -c $S/uart.c -DputsU2=real_putsU2 -o uart_real.o
gcc -w -Dstatic= -I$ST -I$S -c $S/tuner.c -o tuner_open.o
FW="$S/control.c $S/motors.c $S/tuner.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c"
gcc -w -I$ST -I$S sim_ble.c $S/control.c $S/motors.c tuner_open.o $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c -lm -o sim_ble
for t in fw_test v12_test mix_test; do
  gcc -w -I$ST -I$S $t.c $FW $S/imu.c stubs_ble.c -lm -o $t
done
gcc -w -I$ST -I$S fly_test.c $FW stubs_ble.c -lm -o fly_test          # fly_test has its own IMU (a physics model)
TRIMS="-DTRIM_GAIN_FL=1.05f -DTRIM_OFFSET_FL=-3.0f -DTRIM_GAIN_FR=1.0f -DTRIM_OFFSET_FR=0.0f -DTRIM_GAIN_RL=1.0f -DTRIM_OFFSET_RL=0.0f -DTRIM_GAIN_RR=0.97f -DTRIM_OFFSET_RR=0.0f"
gcc -w -I$ST -I$S $TRIMS trim_test.c $S/control.c $S/motors.c $S/tuner.c $S/imu.c $S/noise.c $S/pid.c $S/delay.c uart_real.o $ST/defs.c stubs_ble.c -lm -o trim_test
echo "--- Bluetooth driver vs a simulated Bluefruit ---"; ./sim_ble
echo "--- arming, relay, heartbeat, motor output, PC data ---"; ./fw_test
echo "--- 60 deg cutoff, end of a recorded run ---"; ./v12_test
echo "--- two PIDs, 4-motor mixer, one axis at a time, commands ---"; ./mix_test
echo "--- per-motor trims ---"; ./trim_test
echo "--- FLYING a simulated drone with the real 4 ms loop ---"; ./fly_test
rm -f *.o sim_ble fw_test v12_test mix_test trim_test fly_test
echo "(firmware output for the tuner / app tests: $TUNER_DUMP)"
