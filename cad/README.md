# 3D-printed parts, every version

Every part folder has the same layout:

| Folder | What is in it |
|---|---|
| `print/` | STL files to slice. |
| `step/` | STEP files to open in Fusion 360. |
| `renders/` | Pictures. |
| `src/` | The CadQuery (Python) source. Every size is a parameter. |
| `README.txt` | The original notes, with print settings and fit checks. |

The printer is a Creality K1 SE with Hyper PLA and a 0.4 mm nozzle.

**Print now:**

- tray v4.1 (`06_box_v4_1_tray_CURRENT/print/tray_v4_1.stl`);
- collar v3.1 (`04_box_v3_tray_and_collar/print/collar_v3_1.stl`);
- sensor deck v3.1 (`sensor_deck/v3_1_CURRENT/print/`, pick the snug / normal / loose fit).

Frame axes in the models: +X = true right, +Y = true front, z = 0 = the underside of the bottom plate.
Coordinates differ in the early case v0, so see its README.

| Folder | Part | Date | Status |
|---|---|---|---|
| [`01_case_v0`](01_case_v0) | Electronics case v0 + top-plate IMU mount | 2026-11-18 | Superseded (sat on the arm screws; replaced by box rev 1) |
| [`02_box_rev1`](02_box_rev1) | Electronics box rev 1 | 2026-11-18 | Superseded - sits on the arm screw heads, do not print |
| [`03_box_v2`](03_box_v2) | Electronics box v2 | 2026-11-18 | Superseded by v3 |
| [`04_box_v3_tray_and_collar`](04_box_v3_tray_and_collar) | Box v3: tray + collar (collar v3.1) | 2026-11-19 | Tray v3 superseded by v4/v4.1. COLLAR v3.1 is still the collar to print |
| [`05_box_v4_tray`](05_box_v4_tray) | Box v4 tray (relay bay, 45 deg roof) | 2026-11-24 | Superseded by v4.1 (battery leads) |
| [`06_box_v4_1_tray_CURRENT`](06_box_v4_1_tray_CURRENT) | Box v4.1 tray (relay in back lobe, bullet-lead slot) | 2026-11-24 | CURRENT tray - print with collar v3.1 |
| [`imu_adapter/v1`](imu_adapter/v1) | IMU slot adapter v1 | 2026-11-18 | Superseded by v2 |
| [`imu_adapter/v2`](imu_adapter/v2) | IMU slot adapter v2 | 2026-11-18 | Superseded by the sensor deck |
| [`sensor_deck/v3`](sensor_deck/v3) | Sensor deck v3 (IMU + Bluefruit + hub strip) | 2026-11-18 | Superseded by v3.1 |
| [`sensor_deck/v3_1_CURRENT`](sensor_deck/v3_1_CURRENT) | Sensor deck v3.1 | 2026-11-22 | CURRENT |
