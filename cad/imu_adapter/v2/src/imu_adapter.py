# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/imu_adapter/v2/src/imu_adapter.py
#  Part    : imu_adapter
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: box(), make_adapter(), for_print(), make_imu(), make_plate_patch(), vol(),
#    main()
# ==============================================================================================

# ELC275 drone - slim IMU adapter that keys into the 3 center slots of the F450 bottom plate
# Ethan DiMeo
#
# Why: the IMU's 4 holes are on a 20.32 mm square, the plate's outer slots are NOT,
# so screwing the IMU straight into the slots turns it ~10 deg (crooked).
# This adapter keys into the slots (so it can only sit square), and the IMU screws onto it.
#
# v2: NO POSTS. The top is one flat face, so the part prints top-face-DOWN with the keys
#     pointing up - nothing hangs in the air, no supports. The IMU sits flat on the top.
#     Through-cut reliefs under the 4 edges of the IMU clear the solder joints of its pins.
#
# Run:  python imu_adapter.py OUTPUT_DIR
# Frame: +X = front (red arms), +Y = left, z = 0 = TOP surface of the bottom plate
# (the adapter sits ON the bottom plate, between the plates, IMU on top). Sizes in mm.

import sys, os, math
import cadquery as cq

# ============================== PARAMETERS ==================================
# Bottom plate, center slots (Ethan's caliper: outer edge to outer edge of the 2 outer slots = 20.3)
OUTER_EDGES   = 20.3     # outer edge of the left outer slot to outer edge of the right one
SLOT_W        = 3.0      # width of each slot          (MEASURE: check this one)
CENTER_LEN    = 38.0     # long middle slot length     (photo + F450 model agree)
OUTER_LEN     = 28.0     # the two outer slots' length (F450 model)
PLATE_T       = 1.6

# Fit of the outer keys against the outer slot walls (the part that keeps it square)
KEY_CLEAR     = 0.10     # gap per side between key and slot wall. 0.10 = snug for PLA on a K1 SE
KEY_W         = 1.8      # key width (into the slot)
KEY_L         = 16.0     # outer key length
KEY_H         = 1.4      # key height (plate is 1.6, keys never poke out the bottom)
MID_KEY_W     = 1.6      # middle key width (loose, it only helps)
MID_KEY_L     = 19.0     # stops short of the edge reliefs

# Adapter + IMU
BASE_L, BASE_W, BASE_T = 44.0, 26.0, 4.0   # flat plate; the IMU sits right on top
IMU_HOLE_SP   = 20.32    # SparkFun LSM6DSO Qwiic: 4 holes on a 20.32 mm square (0.8")
IMU_SIZE      = 25.4     # board is 1.0" x 1.0"
PILOT_D       = 2.2      # M2.5 self-tapping into PLA (drill to 2.5 for M3)
PILOT_DEPTH   = 3.6      # blind, 0.4 mm floor left -> M2.5 x 4 or x 5 screws
RELIEF_IN     = 10.2     # edge reliefs start this far from center (pin pads are ~11.4 out)
RELIEF_HALF   = 7.4      # and run +/-7.4 mm along each edge (between the corner holes)
SCREW_X       = 17.0     # 2 screws down through the middle slot, outside the IMU
SCREW_D       = 2.8      # clearance for M2.5 (also fits M2)
# ============================================================================

OUTER_C = OUTER_EDGES / 2 - SLOT_W / 2          # outer slot centerline (y), = 8.65 for 20.3 / 3.0


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def make_adapter(key_clear=KEY_CLEAR):
    a = box(-BASE_L / 2, BASE_L / 2, -BASE_W / 2, BASE_W / 2, 0, BASE_T).edges("|Z").fillet(4.0)

    # outer keys: their OUTER faces sit against the outer slot walls (OUTER_EDGES apart)
    y_out = OUTER_EDGES / 2 - key_clear
    for s in (1, -1):
        k = box(-KEY_L / 2, KEY_L / 2, s * y_out, s * (y_out - KEY_W), -KEY_H, 0)
        k = k.edges("<Z").chamfer(0.4)                 # lead-in so it drops in easily
        a = a.union(k)
    # middle key in the long middle slot
    m = box(-MID_KEY_L / 2, MID_KEY_L / 2, -MID_KEY_W / 2, MID_KEY_W / 2, -KEY_H, 0).edges("<Z").chamfer(0.3)
    a = a.union(m)

    # IMU: 4 blind pilot holes from the top (self-tapping screws)
    h = IMU_HOLE_SP / 2
    for sx in (1, -1):
        for sy in (1, -1):
            a = a.cut(cq.Workplane("XY").circle(PILOT_D / 2).extrude(PILOT_DEPTH + 1)
                      .translate((sx * h, sy * h, BASE_T - PILOT_DEPTH)))

    # edge reliefs, cut all the way through (clears pin solder joints, no bridges to print)
    for s in (1, -1):
        a = a.cut(box(s * RELIEF_IN, s * (IMU_SIZE / 2 + 0.3), -RELIEF_HALF, RELIEF_HALF, -1, BASE_T + 1))
        a = a.cut(box(-RELIEF_HALF, RELIEF_HALF, s * RELIEF_IN, s * (BASE_W / 2 + 1), -1, BASE_T + 1))

    # 2 screws through the middle slot (outside the IMU), nut under the bottom plate
    for sx in (1, -1):
        a = a.cut(cq.Workplane("XY").circle(SCREW_D / 2).extrude(10).translate((sx * SCREW_X, 0, -5)))

    # FRONT arrow engraved on top (points +X), front end, beside the screw
    arrow = (cq.Workplane("XY").polyline([(-2.5, -1.2), (0.8, -1.2), (0.8, -2.6), (3.6, 0), (0.8, 2.6), (0.8, 1.2), (-2.5, 1.2)])
             .close().extrude(0.6).translate((16.8, 7.2, BASE_T - 0.6)))
    return a.cut(arrow)


def for_print(shape):
    # flip it over: top face on the bed (z = 0), keys pointing up
    return shape.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, BASE_T))


def make_imu():
    return box(-IMU_SIZE / 2, IMU_SIZE / 2, -IMU_SIZE / 2, IMU_SIZE / 2, BASE_T, BASE_T + 1.6)


def make_plate_patch():
    # the middle of the bottom plate with the 3 slots, to check the fit
    p = box(-30, 30, -20, 20, -PLATE_T, 0)
    p = p.cut(box(-CENTER_LEN / 2, CENTER_LEN / 2, -SLOT_W / 2, SLOT_W / 2, -5, 5).edges("|Z").fillet(SLOT_W / 2 - 0.01))
    for s in (1, -1):
        p = p.cut(box(-OUTER_LEN / 2, OUTER_LEN / 2, s * (OUTER_C - SLOT_W / 2), s * (OUTER_C + SLOT_W / 2), -5, 5)
                  .edges("|Z").fillet(SLOT_W / 2 - 0.01))
    return p


def vol(s):
    v = s.val() if s.vals() else None
    return v.Volume() if v else 0.0


def main(out):
    os.makedirs(out, exist_ok=True)
    plate = make_plate_patch()
    report = [f"outer slot centerline y = +/-{OUTER_C:.2f} mm, slot width {SLOT_W}, outer edges {OUTER_EDGES} apart"]
    for name, clr in (("snug", 0.05), ("normal", KEY_CLEAR), ("loose", 0.20)):
        a = make_adapter(clr)
        cq.exporters.export(for_print(a), f"{out}/imu_adapter_{name}.stl", tolerance=0.02, angularTolerance=0.05)
        if name == "normal":
            cq.exporters.export(a, f"{out}/imu_adapter.step")
            report.append(f"normal: key/plate overlap {vol(a.intersect(plate)):.4f} mm3 (0 = keys fit in the slots)")
            report.append(f"normal: adapter/IMU overlap {vol(a.intersect(make_imu())):.4f} mm3 (0 = IMU sits on top)")
            report.append(f"normal: volume {vol(a)/1000:.2f} cm3 = about {vol(a)/1000*1.24:.1f} g PLA solid")
            bb = a.val().BoundingBox()
            report.append(f"normal: size {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, IMU sits {BASE_T:.1f} mm above the plate")
        report.append(f"{name}: outer key faces {OUTER_EDGES - 2*clr:.2f} mm apart (slot walls {OUTER_EDGES} apart), "
                      f"turn play about +/-{math.degrees(clr*2/KEY_L):.2f} deg")
    cq.exporters.export(plate, f"{out}/plate_center_patch.step")
    asm = cq.Assembly(name="imu_adapter_check")
    asm.add(plate, name="bottom_plate_center", color=cq.Color(0.15, 0.15, 0.15))
    asm.add(make_adapter(), name="imu_adapter", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(make_imu(), name="LSM6DSO_board", color=cq.Color(0.8, 0.1, 0.1))
    asm.save(f"{out}/imu_adapter_assembly.step")
    open(f"{out}/imu_adapter_fit.txt", "w").write("\n".join(report) + "\n")
    print("\n".join(report))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
