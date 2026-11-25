IMU SLOT ADAPTER (Nov 18, 2026) - puts the LSM6DSO square in the middle of the F450 bottom plate
====================================================================================================
The IMU's 4 holes are on a 20.32 mm square; the plate's outer slots are not, so screwing the IMU
straight into the slots turns it about 10 degrees. This 3 g adapter keys into the 3 center slots
(it can only sit square), and the IMU screws onto it.

PRINT  print/imu_adapter_normal.stl first (flat side down, keys up, no supports, 0.2 mm, 100% infill).
       Drops in with a little play?  print imu_adapter_snug.stl.   Won't go in?  imu_adapter_loose.stl.
       (outer keys 20.10 / 20.20 / 19.90 mm apart against slot walls measured 20.3 mm apart)

FIT    keys DOWN into the 3 center slots, FRONT arrow toward the red (front) arms.
       2x M2.5 x 8 screws down through the adapter and the long middle slot, nuts under the plate
       (use M2 if the middle slot turns out narrower than 2.6 mm).
       IMU on the 4 posts with 4x M2.5 x 6 self-tapping screws (posts have 2.2 mm pilot holes;
       for M3 screws drill the pilots to 2.5 mm).
       Turn the IMU so the little X arrow printed on the board points at the red arms (front)
       and the Y arrow points to the drone's LEFT. The firmware's pitch depends on this.

MEASURE (if it doesn't fit, change these in src/imu_adapter.py and run it again)
       OUTER_EDGES = 20.3   outer edge of one outer slot to the outer edge of the other (your caliper)
       SLOT_W      = 3.0    slot width
       CENTER_LEN  = 38.0   middle slot length
