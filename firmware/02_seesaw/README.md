# Seesaw rig (2 motors, one axis)

The PID was tuned first on a seesaw: one arm with a motor at each end, pivoting in the middle.

| Folder | What |
|---|---|
| [`01_seesaw_pid_single_file`](01_seesaw_pid_single_file) | Everything in one file, IMU on I2C (Nov 12). |
| [`02_SewSaw_step0-1`](02_SewSaw_step0-1) | `SewSaw.X`: the multi-file project. IMU on SPI1 with CS on pin 16, complementary filter, PuTTY display, PC tuner link (Nov 16, worked on the board). |
| [`03_SewSaw_step2-3`](03_SewSaw_step2-3) | Adds `motors.c`: the PID drives both seesaw motors (Nov 16). |
