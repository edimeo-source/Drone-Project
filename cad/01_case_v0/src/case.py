# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/01_case_v0/src/case.py
#  Part    : 01_case_v0
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: box(), make_case(), make_imu_mount(), make_plate(), make_parts_for_preview(),
#    main()
# ==============================================================================================

# ELC275 drone - electronics case + IMU mount (CadQuery 2.x)
# Ethan DiMeo
#
# Run:   python case.py OUTPUT_DIR
# Makes: case.step/.stl, imu_mount.step/.stl, assembly.step (with the F450 bottom plate,
#        PIC32 kit, Bluefruit, battery, IMU as plain blocks), and fit_check.txt
#
# Frame (same as the earlier CAD work): origin = frame center, +X = FRONT (red arms),
# +Y = left, +Z = up, z = 0 = UNDERSIDE of the F450 bottom plate. All sizes in mm.
#
# Every number you might want to change is in the PARAMETERS block. Measure the
# "MEASURE FIRST" ones on the real parts before printing, change them here, run again.

import sys, json, os
import cadquery as cq

# ============================== PARAMETERS ==================================
# F450 bottom plate (sliced from an open F450 model, matches the photos; +/-1 mm)
PLATE_T      = 1.6
SLOT_X       = 70.1      # tab slot centers at x = +/-70.1
SLOT_Y       = 17.0      #                    y = +/-17.0
SLOT_LEN     = 29.8      # slot length along X
SLOT_W       = 4.0       # slot width along Y
TAB_HALF_W   = 26.0      # case end plates stay inside the plate tab (tab is +/-28.5 wide)

# PIC32MZ EF Starter Kit  (MEASURE FIRST: size; J12 is on the UNDERSIDE, guide sec. 1.2/2)
KIT_L, KIT_W, KIT_T = 71.5, 51.0, 1.6
KIT_CLEAR    = 0.8       # gap around the board in its pocket
KIT_X0       = -46.0     # rear (USB) edge of the board; leaves a 14 mm bay for USB plugs
KIT_LIFT     = 30.0      # board underside this high above the floor: room for J12 jumpers hanging down
TOP_CLEAR    = 14.0      # board top to the frame plate (unplug the Ethernet daughter board)

# Bluefruit LE SPI Friend (taped onto a shallow pad, no screw holes needed)
BLE_L, BLE_W = 25.4, 22.9
BLE_X        = 43.0      # center of its pad (front of the case)

# Battery: Zeee 3S 2200 "shorty" 75 x 34 x 26.5  or regular 102 x 34 x 23 (both fit)
BAT_L, BAT_W, BAT_H = 75.0, 34.0, 26.5
STRAP_W      = 20.0      # velcro strap width
STRAP_X      = 28.0      # strap ears at x = +/-28

# Case walls
WALL, FLOOR  = 2.0, 2.4
IN_X0, IN_X1 = -60.0, 60.0          # inside length along X
IN_HALF_W    = 28.0                 # inside half-width (56 mm inside)
EAR_X1       = 86.0                 # end plates reach out to x = +/-86
EAR_T        = 4.0                  # end plate thickness
KEY_X0, KEY_X1 = 63.0, 73.0         # key that drops into each slot (inner part of the slot)
KEY_W, KEY_H = 3.0, 1.4             # 0.5 mm clearance each side, shorter than the 1.6 plate
BOLT_X       = 79.0                 # M3 bolt through the outer part of each slot
BOLT_D       = 3.4
NUT_AF, NUT_DEPTH = 5.7, 2.6        # M3 nut trap under the end plate

# IMU mount (top plate, center).  SparkFun LSM6DSO Qwiic: 25.4 square, 4 x d3.05 holes on 20.32 square
IMU_B        = 25.4
IMU_HOLE_SP  = 20.32
IMU_POST_H   = 8.0                  # room for header pins under the IMU board
MOUNT_L, MOUNT_W, MOUNT_T = 50.0, 44.0, 3.0
TOP_PLATE_Z  = 38.4                 # top plate underside (earlier CAD), plate is 1.7 thick
# ============================================================================

H_IN   = KIT_LIFT + KIT_T + TOP_CLEAR          # inside height
Z_FLOOR_TOP = -H_IN
Z_BOTTOM    = Z_FLOOR_TOP - FLOOR


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(
        ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def make_case():
    # 1. outer shell minus the inside = an open-top tray (the frame plate is its lid)
    out_x0, out_x1 = IN_X0 - WALL, IN_X1 + WALL
    hw = IN_HALF_W + WALL
    c = box(out_x0, out_x1, -hw, hw, Z_BOTTOM, 0).edges("|Z").fillet(3.0)
    c = c.cut(box(IN_X0, IN_X1, -IN_HALF_W, IN_HALF_W, Z_FLOOR_TOP, 1).edges("|Z").fillet(1.5))

    # 2. end plates ("ears") that sit flat against the plate tabs, with keys + bolt holes
    for sx in (1, -1):
        x0, x1 = sorted((sx * (IN_X1), sx * EAR_X1))
        ear = box(x0, x1, -TAB_HALF_W, TAB_HALF_W, -EAR_T, 0).edges("|Z").fillet(2.0)
        c = c.union(ear)
        # 45 deg gussets under each end plate (print without supports, stiffen the ear)
        for gy in (-TAB_HALF_W + 1.5, 0.0, TAB_HALF_W - 1.5):
            d = EAR_X1 - IN_X1 - 2
            g = (cq.Workplane("XZ").polyline([(0, 0), (d, 0), (0, -d)]).close()
                 .extrude(1.5, both=True))
            g = g.translate((IN_X1 + 1, gy, -EAR_T)) if sx > 0 else \
                g.mirror("YZ").translate((-(IN_X1 + 1), gy, -EAR_T))
            c = c.union(g)
        for sy in (1, -1):
            # key into the slot (inner end of the slot)
            kx0, kx1 = sorted((sx * KEY_X0, sx * KEY_X1))
            c = c.union(box(kx0, kx1, sy * SLOT_Y - KEY_W / 2, sy * SLOT_Y + KEY_W / 2, 0, KEY_H))
            # bolt hole + hex nut trap from below
            c = c.cut(cq.Workplane("XY").circle(BOLT_D / 2).extrude(40)
                      .translate((sx * BOLT_X, sy * SLOT_Y, -20)))
            c = c.cut(cq.Workplane("XY").polygon(6, NUT_AF / 0.866).extrude(NUT_DEPTH)
                      .translate((sx * BOLT_X, sy * SLOT_Y, -EAR_T)))

    # 3. PIC32 kit: 4 corner towers. Board corners rest on a ledge at KIT_LIFT,
    #    a 3 mm lip above the ledge stops it sliding. J12 jumpers hang below the board.
    kx0, kx1 = KIT_X0 - KIT_CLEAR, KIT_X0 + KIT_L + KIT_CLEAR
    ky = KIT_W / 2 + KIT_CLEAR
    z_led = Z_FLOOR_TOP + KIT_LIFT
    T = 7.0                                   # tower footprint
    for (cx, cy) in ((kx0, ky), (kx1, ky), (kx0, -ky), (kx1, -ky)):
        tx0, tx1 = (cx - 2, cx + T - 2) if cx == kx0 else (cx - T + 2, cx + 2)
        ty0, ty1 = (cy - T + 2, cy + 2) if cy > 0 else (cy - 2, cy + T - 2)
        tower = box(tx0, tx1, ty0, ty1, Z_FLOOR_TOP, z_led + 3.0)
        # cut the board's corner pocket out of the lip (board sits at z_led..z_led+1.6)
        pocket = box(kx0, kx1, -ky, ky, z_led, z_led + 10)
        c = c.union(tower.cut(pocket))
    # zip-tie anchors on the long walls, over the middle of the board (holds it down)
    zt = z_led + KIT_T + 2.0
    for sy in (1, -1):
        for x in (-14.0, -6.0):
            c = c.cut(box(x - 1.6, x + 1.6, sy * IN_HALF_W - 3, sy * IN_HALF_W + 3, zt - 1.0, zt + 1.0))

    # 4. Bluefruit pad: 2 mm lip around a taped spot, raised 3 mm off the floor
    bx0, bx1 = BLE_X - BLE_L / 2 - 0.8, BLE_X + BLE_L / 2 + 0.8
    by = BLE_W / 2 + 0.8
    pad = box(bx0 - 1.6, bx1 + 1.6, -by - 1.6, by + 1.6, Z_FLOOR_TOP, Z_FLOOR_TOP + 5.0)
    pad = pad.cut(box(bx0, bx1, -by, by, Z_FLOOR_TOP + 3.0, Z_FLOOR_TOP + 6.0))
    c = c.union(pad)

    # 5. rear window: USB plugs (J3 debug, J11 UART on top, J4 power on the underside)
    zb = z_led
    c = c.cut(box(IN_X0 - WALL - 1, IN_X0 + 1, -21, 21, zb - 9.0, zb + KIT_T + 11.0))
    # side windows: route the J12 jumper bundle / see the LEDs and switches
    for sy in (1, -1):
        c = c.cut(box(-36, 14, sy * (IN_HALF_W - 1), sy * (IN_HALF_W + WALL + 1),
                      Z_FLOOR_TOP + 8, Z_FLOOR_TOP + 24))

    # 6. battery strap ears at floor level on the long walls (strap goes around the battery)
    for sx in (1, -1):
        for sy in (1, -1):
            ex = sx * STRAP_X
            ear = box(ex - STRAP_W / 2 - 4, ex + STRAP_W / 2 + 4,
                      min(sy * (IN_HALF_W + WALL - 0.5), sy * (IN_HALF_W + WALL + 8)),
                      max(sy * (IN_HALF_W + WALL - 0.5), sy * (IN_HALF_W + WALL + 8)),
                      Z_BOTTOM, Z_BOTTOM + FLOOR)
            ear = ear.edges("|Z").fillet(1.5)
            slot = box(ex - STRAP_W / 2 - 1, ex + STRAP_W / 2 + 1,
                       sy * (IN_HALF_W + WALL + 3.2) - 1.75, sy * (IN_HALF_W + WALL + 3.2) + 1.75,
                       Z_BOTTOM - 1, Z_BOTTOM + FLOOR + 1)
            c = c.union(ear).cut(slot)
    # battery locating rails under the floor (keep it centered), 2 mm tall
    for sy in (1, -1):
        y0 = sy * (BAT_W / 2 + 0.8)
        c = c.union(box(-50, 50, min(y0, y0 + sy * 2.4), max(y0, y0 + sy * 2.4), Z_BOTTOM - 2.0, Z_BOTTOM))
    # FRONT arrow engraved in the floor (inside)
    arrow = (cq.Workplane("XY").polyline([(-6, -4), (2, -4), (2, -8), (10, 0), (2, 8), (2, 4), (-6, 4)])
             .close().extrude(0.8).translate((8, 0, Z_FLOOR_TOP - 0.8)))
    c = c.cut(arrow)
    return c


def make_imu_mount():
    # flat plate taped to the top plate's center (foam tape), 4 posts for the IMU board,
    # a wire hole under the IMU, and a FRONT arrow on a tab.
    z0 = TOP_PLATE_Z + 1.7
    m = box(-MOUNT_L / 2, MOUNT_L / 2, -MOUNT_W / 2, MOUNT_W / 2, z0, z0 + MOUNT_T).edges("|Z").fillet(4)
    tab = (cq.Workplane("XY").polyline([(MOUNT_L / 2 - 2, -10), (MOUNT_L / 2 + 10, 0), (MOUNT_L / 2 - 2, 10)])
           .close().extrude(MOUNT_T).translate((0, 0, z0)))
    m = m.union(tab)
    h = IMU_HOLE_SP / 2
    for sx in (1, -1):
        for sy in (1, -1):
            post = (cq.Workplane("XY").circle(3.0).extrude(IMU_POST_H)
                    .translate((sx * h, sy * h, z0 + MOUNT_T)))
            m = m.union(post)
            m = m.cut(cq.Workplane("XY").circle(1.25).extrude(IMU_POST_H + MOUNT_T + 1)
                      .translate((sx * h, sy * h, z0)))      # 2.5 mm pilot: M3 screw cuts its own thread
    m = m.cut(box(-14, 0, -9, 9, z0 - 1, z0 + MOUNT_T + 1).edges("|Z").fillet(3))   # wire hole (rear half)
    # FRONT arrow engraved on top
    arrow = (cq.Workplane("XY").polyline([(-4, -2.5), (1, -2.5), (1, -5), (6, 0), (1, 5), (1, 2.5), (-4, 2.5)])
             .close().extrude(0.8).translate((MOUNT_L / 2 + 1, 0, z0 + MOUNT_T - 0.8)))
    return m.cut(arrow)


def make_plate():
    d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "f450_bottom_plate.json")))
    p = cq.Workplane("XY").polyline(d["exterior"][:-1]).close().extrude(PLATE_T)
    for ring in d["interiors"]:
        if len(ring) > 3:
            try:
                p = p.cut(cq.Workplane("XY").polyline(ring[:-1]).close().extrude(PLATE_T + 2).translate((0, 0, -1)))
            except Exception:
                pass
    return p


def make_parts_for_preview():
    z_led = Z_FLOOR_TOP + KIT_LIFT
    kit = box(KIT_X0, KIT_X0 + KIT_L, -KIT_W / 2, KIT_W / 2, z_led, z_led + KIT_T)
    j12 = box(KIT_X0 + 10, KIT_X0 + 10 + 50.8, KIT_W / 2 - 6, KIT_W / 2 - 1, z_led - 8.5, z_led)   # header under the board
    ble = box(BLE_X - BLE_L / 2, BLE_X + BLE_L / 2, -BLE_W / 2, BLE_W / 2, Z_FLOOR_TOP + 3, Z_FLOOR_TOP + 4.6)
    bat = box(-BAT_L / 2, BAT_L / 2, -BAT_W / 2, BAT_W / 2, Z_BOTTOM - BAT_H, Z_BOTTOM)
    imu_z = TOP_PLATE_Z + 1.7 + MOUNT_T + IMU_POST_H
    imu = box(-IMU_B / 2, IMU_B / 2, -IMU_B / 2, IMU_B / 2, imu_z, imu_z + 1.6)
    top = (box(-54, 54, -54, 54, TOP_PLATE_Z, TOP_PLATE_Z + 1.7)
           .cut(cq.Workplane("XY").circle(12).extrude(5).translate((0, 0, TOP_PLATE_Z - 1))))
    return kit.union(j12), ble, bat, imu, top


def main(out):
    os.makedirs(out, exist_ok=True)
    case = make_case()
    mount = make_imu_mount()
    plate = make_plate()
    kit, ble, bat, imu, top = make_parts_for_preview()

    # export the printed parts in print orientation too (case: floor down = as modeled)
    cq.exporters.export(case, f"{out}/case.step")
    cq.exporters.export(case, f"{out}/case.stl", tolerance=0.05, angularTolerance=0.1)
    cq.exporters.export(mount, f"{out}/imu_mount.step")
    cq.exporters.export(mount, f"{out}/imu_mount.stl", tolerance=0.05, angularTolerance=0.1)

    asm = cq.Assembly(name="ELC275_drone_case")
    asm.add(plate, name="F450_bottom_plate", color=cq.Color(0.15, 0.15, 0.15))
    asm.add(top, name="F450_top_plate_simplified", color=cq.Color(0.15, 0.15, 0.15, 0.6))
    asm.add(case, name="case_printed", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(mount, name="imu_mount_printed", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(kit, name="PIC32MZ_EF_starter_kit", color=cq.Color(0.1, 0.5, 0.2))
    asm.add(ble, name="Bluefruit_SPI_Friend", color=cq.Color(0.1, 0.2, 0.7))
    asm.add(bat, name="Battery_3S_2200", color=cq.Color(0.8, 0.6, 0.1))
    asm.add(imu, name="LSM6DSO_IMU", color=cq.Color(0.8, 0.1, 0.1))
    asm.save(f"{out}/assembly.step")

    # ---- fit checks ----
    lines = []
    def vol(s): return s.val().Volume()
    def inter(a, b):
        try: return a.intersect(b).val().Volume()
        except Exception: return 0.0
    lines.append(f"inside height {H_IN:.1f} mm, case bottom at z = {Z_BOTTOM:.1f}, battery bottom at z = {Z_BOTTOM - BAT_H - 2:.1f}")
    lines.append(f"case volume {vol(case)/1000:.1f} cm3 -> about {vol(case)/1000*1.27*0.6:.0f} g PETG at ~60% fill")
    lines.append(f"imu mount volume {vol(mount)/1000:.1f} cm3 -> about {vol(mount)/1000*1.27:.0f} g PETG")
    for name, part in (("case vs plate", case), ("PIC32 vs case", kit), ("Bluefruit vs case", ble),
                       ("battery vs case", bat), ("IMU vs mount", imu)):
        a, b = (part, plate) if name == "case vs plate" else (part, case if name != "IMU vs mount" else mount)
        lines.append(f"overlap {name}: {inter(a, b):.3f} mm3")
    # keys must be fully inside slots (key volume that is NOT in a slot = clash with plate)
    keys_only = case.intersect(box(-100, 100, -40, 40, 0, KEY_H))
    lines.append(f"key volume above z=0: {vol(keys_only):.1f} mm3; key/plate clash: {inter(keys_only, plate):.3f} mm3")
    open(f"{out}/fit_check.txt", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
