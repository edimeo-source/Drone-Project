# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/03_box_v2/src/box_v2.py
#  Part    : 03_box_v2
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: box(), cyl(), make_box(), make_plate(), make_arm_screw_heads(), make_parts(),
#    main()
# ==============================================================================================

# ELC275 drone - electronics box V2 (CadQuery 2.x)
# Ethan DiMeo
#
# Holds: PIC32MZ EF Starter Kit + the power / Bluetooth-kill board, battery strapped underneath.
# The Bluefruit and the IMU are NOT in the box any more: they sit on the sensor deck v3 in the
# middle of the bottom plate. Everything that leaves the box (sensor cable, ESC signal leads,
# power leads to the PDB) goes UP through the bottom plate's big front / rear center openings.
#
# V2 vs rev 1:
#   - no Bluefruit slot, no side windows, no floor cutout, no PWM slots in the walls
#   - 6 mm longer and 4 mm wider inside, for wire slack (no cut-and-solder, only plug-in leads)
#   - pockets in the top edge for the 4 arm screw heads under the plate at (+/-46.5, +/-32)
#     (rev 1's top edge sat right on them)
#   - power bay at the front sized for the 45 x 58 proto board, XT60 window moved up above it,
#     4 vent slots next to the 7805
#
# Run:   python box_v2.py OUTPUT_DIR
# Frame: origin = frame center, +X = FRONT (red arms), +Y = left, +Z = up,
#        z = 0 = UNDERSIDE of the F450 bottom plate (the box hangs under it). Sizes in mm.

import sys, json, os
import cadquery as cq

# ============================== PARAMETERS ==================================
PLATE_T      = 1.6
SLOT_X, SLOT_Y = 70.1, 17.0          # tab slot centers
TAB_HALF_W   = 26.0
ARM_SCREWS   = [(sx * 46.5, sy * 32.0) for sx in (1, -1) for sy in (1, -1)]   # arm screw heads under the plate
ARM_HEAD_D, ARM_HEAD_H = 7.0, 3.0    # pocket for each head (M2.5/M3 head + room)

IN_X0, IN_X1 = -63.0, 63.0           # inside length 126
IN_HW        = 34.0                  # inside half-width (68 inside)
WALL, FLOOR  = 2.0, 2.4

# PIC32MZ EF Starter Kit (MEASURE FIRST). J12 on the UNDERSIDE (front edge): board sits 30 mm up.
KIT_L, KIT_W, KIT_T = 71.5, 51.0, 1.6
KIT_X0       = -59.0     # rear (USB) edge, 4 mm off the rear wall
KIT_LIFT     = 30.0
TOP_CLEAR    = 14.0      # board top to the plate: room for wires running over the kit
HOLE_FROM_USB_EDGE = 19.2
HOLE_FROM_J5_EDGE  = 25.8
SIDE_CLEAR   = 1.0
POST_PILOT   = 2.2       # M2.5 x 8 self-tapping into the center post
J12_IN       = 6.0

# Power / kill board (Ethan's proto board with the 7805): flat on the floor of the front bay
PWR_L, PWR_W, PWR_H = 45.0, 58.0, 22.0    # along X, along Y, tallest part (MEASURE FIRST)
PWR_X0       = 15.5                       # rear edge of the board
ZIP_X        = (24.0, 52.0)               # 2 zip ties across the board (along Y) ...
ZIP_Y        = 31.0                       # ... down through floor slots at y = +/-31, under the floor
XT60_W, XT60_H, XT60_Y = 16.6, 9.0, -12.0 # front-wall window, ABOVE the power board
XT60_Z0      = -21.0
VENT_Y       = (8.0, 14.0, 20.0, 26.0)    # vent slots in the front wall next to the 7805
VENT_W, VENT_Z = 3.0, (-42.0, -26.0)

# frame mounting (unchanged): keys + M3 bolts in the 4 tab slots
EAR_X1, EAR_T = 86.0, 4.0
KEY_X0, KEY_X1, KEY_W, KEY_H = 63.0, 73.0, 3.0, 1.4
BOLT_X, BOLT_D = 79.0, 3.4
NUT_AF, NUT_DEPTH = 5.7, 2.6

BAT_L, BAT_W, BAT_H = 75.0, 34.0, 26.5
STRAP_W, STRAP_X = 20.0, 28.0
# ============================================================================

H_IN        = KIT_LIFT + KIT_T + TOP_CLEAR
Z_FLOOR_TOP = -H_IN
Z_BOTTOM    = Z_FLOOR_TOP - FLOOR
Z_KIT       = Z_FLOOR_TOP + KIT_LIFT
KIT_X1      = KIT_X0 + KIT_L
HOLE_X      = KIT_X0 + HOLE_FROM_USB_EDGE
HOLE_Y      = -KIT_W / 2 + HOLE_FROM_J5_EDGE
OUT_HW      = IN_HW + WALL


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl(d, z0, z1, x, y):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def make_box():
    # 1. open-top tray; the frame plate is the lid
    b = box(IN_X0 - WALL, IN_X1 + WALL, -OUT_HW, OUT_HW, Z_BOTTOM, 0).edges("|Z").fillet(3.0)
    b = b.cut(box(IN_X0, IN_X1, -IN_HW, IN_HW, Z_FLOOR_TOP, 1).edges("|Z").fillet(1.5))

    # 2. end plates on the plate tabs: key in each tab slot + M3 bolt + nut trap, 45 deg gussets
    for sx in (1, -1):
        x0, x1 = sorted((sx * IN_X1, sx * EAR_X1))
        b = b.union(box(x0, x1, -TAB_HALF_W, TAB_HALF_W, -EAR_T, 0).edges("|Z").fillet(2.0))
        gys = (-TAB_HALF_W + 1.5, 0.0, TAB_HALF_W - 1.5) if sx > 0 else \
              (-TAB_HALF_W + 1.5, -11.0, 10.2, TAB_HALF_W - 1.5)          # rear: clear of the USB plugs
        for gy in gys:
            d = EAR_X1 - IN_X1 - 2
            g = cq.Workplane("XZ").polyline([(0, 0), (d, 0), (0, -d)]).close().extrude(1.5, both=True)
            g = g.translate((IN_X1 + 1, gy, -EAR_T)) if sx > 0 else \
                g.mirror("YZ").translate((-(IN_X1 + 1), gy, -EAR_T))
            b = b.union(g)
        for sy in (1, -1):
            kx0, kx1 = sorted((sx * KEY_X0, sx * KEY_X1))
            b = b.union(box(kx0, kx1, sy * SLOT_Y - KEY_W / 2, sy * SLOT_Y + KEY_W / 2, 0, KEY_H))
            b = b.cut(cyl(BOLT_D, -20, 20, sx * BOLT_X, sy * SLOT_Y))
            b = b.cut(cq.Workplane("XY").polygon(6, NUT_AF / 0.866).extrude(NUT_DEPTH)
                      .translate((sx * BOLT_X, sy * SLOT_Y, -EAR_T)))

    # 3. PIC32 kit: center screw post at its mounting hole + 4 side supports with lips
    post = cyl(14.0, Z_FLOOR_TOP, Z_FLOOR_TOP + 1.0, HOLE_X, HOLE_Y)
    post = post.union(cq.Workplane("XY").circle(7.0).workplane(offset=4.0).circle(4.5).loft()
                      .translate((HOLE_X, HOLE_Y, Z_FLOOR_TOP + 1.0)))
    post = post.union(cyl(9.0, Z_FLOOR_TOP + 1.0, Z_KIT - 6.0, HOLE_X, HOLE_Y))
    post = post.union(cyl(5.5, Z_KIT - 6.0, Z_KIT, HOLE_X, HOLE_Y))
    post = post.cut(cyl(POST_PILOT, Z_KIT - 12.0, Z_KIT + 1, HOLE_X, HOLE_Y))
    b = b.union(post)
    lip_in = KIT_W / 2 + SIDE_CLEAR
    for (tx0, tx1) in ((KIT_X0 + 10, KIT_X0 + 16), (-3.0, 3.0)):
        for sy in (1, -1):
            t = box(tx0, tx1, sy * (KIT_W / 2 - 2.5), sy * IN_HW, Z_FLOOR_TOP, Z_KIT)
            t = t.union(box(tx0, tx1, sy * lip_in, sy * IN_HW, Z_KIT, Z_KIT + KIT_T + 2.0))
            b = b.union(t)

    # 4. rear window: J3 (program), J11 (PuTTY / tuner) on top, J4 (power) underneath
    b = b.cut(box(IN_X0 - WALL - 1, IN_X0 + 1, -22.5, 22.5, Z_KIT - 8.4, -5.0))

    # 5. power bay: 2 zip ties across the board (down through the floor, under it, back up),
    #    XT60 window above the board, vent slots next to the 7805
    for zx in ZIP_X:
        for sy in (1, -1):
            b = b.cut(box(zx - 2.6, zx + 2.6, sy * ZIP_Y - 1.0, sy * ZIP_Y + 1.0, Z_BOTTOM - 1, Z_FLOOR_TOP + 1))
    b = b.cut(box(IN_X1 - 1, IN_X1 + WALL + 1, XT60_Y - XT60_W / 2, XT60_Y + XT60_W / 2, XT60_Z0, XT60_Z0 + XT60_H))
    for vy in VENT_Y:
        b = b.cut(box(IN_X1 - 1, IN_X1 + WALL + 1, vy - VENT_W / 2, vy + VENT_W / 2, *VENT_Z))

    # 6. pockets in the top edge for the 4 arm screw heads under the plate
    for (ax, ay) in ARM_SCREWS:
        b = b.cut(cyl(ARM_HEAD_D, -ARM_HEAD_H, 1, ax, ay))

    # 7. battery strap ears at floor level on the long walls
    for sx in (1, -1):
        for sy in (1, -1):
            ex = sx * STRAP_X
            ear = box(ex - STRAP_W / 2 - 4, ex + STRAP_W / 2 + 4, sy * (OUT_HW - 0.5), sy * (OUT_HW + 8),
                      Z_BOTTOM, Z_BOTTOM + FLOOR).edges("|Z").fillet(1.5)
            slot = box(ex - STRAP_W / 2 - 1, ex + STRAP_W / 2 + 1,
                       sy * (OUT_HW + 3.2) - 1.75, sy * (OUT_HW + 3.2) + 1.75, Z_BOTTOM - 1, Z_BOTTOM + FLOOR + 1)
            b = b.union(ear).cut(slot)

    # 8. FRONT arrow engraved on the outside of the right (-Y) wall
    arrow = (cq.Workplane("XZ").polyline([(-8, -3), (2, -3), (2, -6), (10, 0), (2, 6), (2, 3), (-8, 3)])
             .close().extrude(-0.6).translate((0, -OUT_HW, Z_FLOOR_TOP + 22)))
    return b.cut(arrow)


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


def make_arm_screw_heads():
    # 8 arm screws come up through the bottom plate; 4 of them are over the box
    pts = ARM_SCREWS + [(sx * 32.0, sy * 46.5) for sx in (1, -1) for sy in (1, -1)]
    h = None
    for (ax, ay) in pts:
        s = cyl(5.5, -2.5, 0, ax, ay)
        h = s if h is None else h.union(s)
    return h


def make_parts():
    kit = box(KIT_X0, KIT_X1, -KIT_W / 2, KIT_W / 2, Z_KIT, Z_KIT + KIT_T)
    kit = kit.cut(cyl(3.0, Z_KIT - 1, Z_KIT + 3, HOLE_X, HOLE_Y))
    usb = (box(KIT_X0 - 0.5, KIT_X0 + 9, 14.0, 21.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 4)
           .union(box(KIT_X0 - 0.5, KIT_X0 + 9, -1.0, 6.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 4))
           .union(box(KIT_X0 - 1.0, KIT_X0 + 14, -23.5, -6.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 7)))
    j12 = box(KIT_X1 - J12_IN, KIT_X1 - 0.5, -25.4, 25.4, Z_KIT - 8.5, Z_KIT)
    jumpers = box(KIT_X1 - J12_IN, KIT_X1 - 0.5, -25.4, 25.4, Z_KIT - 8.5 - 14.0, Z_KIT - 8.5)
    bat = box(-BAT_L / 2, BAT_L / 2, -BAT_W / 2, BAT_W / 2, Z_BOTTOM - BAT_H, Z_BOTTOM)
    pwr = box(PWR_X0, PWR_X0 + PWR_L, -PWR_W / 2, PWR_W / 2, Z_FLOOR_TOP, Z_FLOOR_TOP + PWR_H)
    xt60 = box(IN_X1 - 8, IN_X1 + WALL + 8, XT60_Y - 7.75, XT60_Y + 7.75, XT60_Z0 + 0.4, XT60_Z0 + 0.4 + 8.1)
    screw = cyl(2.5, Z_KIT - 6.4, Z_KIT + KIT_T, HOLE_X, HOLE_Y).union(
        cyl(4.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 1.8, HOLE_X, HOLE_Y))
    return dict(kit=kit, usb=usb, j12=j12, jumpers=jumpers, bat=bat, pwr=pwr, xt60=xt60, screw=screw)


def main(out):
    os.makedirs(out, exist_ok=True)
    bx = make_box(); plate = make_plate(); P = make_parts(); heads = make_arm_screw_heads()
    cq.exporters.export(bx, f"{out}/box_v2.step")
    cq.exporters.export(bx.translate((0, 0, -Z_BOTTOM)), f"{out}/box_v2.stl", tolerance=0.05, angularTolerance=0.1)

    asm = cq.Assembly(name="ELC275_box_v2")
    asm.add(plate, name="F450_bottom_plate", color=cq.Color(0.15, 0.15, 0.15))
    asm.add(heads, name="arm_screw_heads", color=cq.Color(0.5, 0.5, 0.5))
    asm.add(bx, name="box_v2_printed", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(P["kit"].union(P["usb"]).union(P["j12"]), name="PIC32MZ_EF_starter_kit", color=cq.Color(0.1, 0.5, 0.2))
    asm.add(P["jumpers"], name="J12_jumper_housings_keepout", color=cq.Color(0.2, 0.2, 0.2))
    asm.add(P["screw"], name="M2.5x8_screw", color=cq.Color(0.6, 0.6, 0.6))
    asm.add(P["pwr"], name="power_kill_board_45x58_PLACEHOLDER", color=cq.Color(0.9, 0.4, 0.1))
    asm.add(P["xt60"], name="XT60_male", color=cq.Color(0.9, 0.8, 0.1))
    asm.add(P["bat"], name="Battery_3S_2200", color=cq.Color(0.8, 0.6, 0.1))
    asm.save(f"{out}/assembly_v2.step")

    def vol(s):
        try: return s.val().Volume()
        except Exception: return 0.0
    def inter(a, b):
        try: return a.intersect(b).val().Volume()
        except Exception: return 0.0
    L = []
    L.append(f"box outside {IN_X1 - IN_X0 + 2 * WALL:.0f} x {2 * OUT_HW:.0f} x {-Z_BOTTOM:.0f} mm (with end plates {2 * EAR_X1:.0f} long), "
             f"inside {IN_X1 - IN_X0:.0f} x {2 * IN_HW:.0f} x {H_IN:.1f} mm")
    L.append(f"box bottom z = {Z_BOTTOM:.1f}, battery bottom z = {Z_BOTTOM - BAT_H:.1f} (below the bottom plate)")
    L.append(f"PIC32: x {KIT_X0:.1f} .. {KIT_X1:.1f}, screw post at ({HOLE_X:.1f}, {HOLE_Y:.1f}); "
             f"front bay x {KIT_X1 + 1:.1f} .. {IN_X1:.1f} = {IN_X1 - KIT_X1 - 1:.1f} mm long, {2 * IN_HW:.0f} wide, {H_IN:.1f} tall")
    L.append(f"wire room beside the kit: {IN_HW - KIT_W / 2:.1f} mm each side; over the kit: {TOP_CLEAR:.0f} mm")
    L.append(f"box volume {vol(bx)/1000:.1f} cm3 -> about {vol(bx)/1000*1.24*0.6:.0f} g PLA (3 walls, 15% infill)")
    for name in ("kit", "usb", "j12", "jumpers", "bat", "pwr", "xt60", "screw"):
        part = P[name]
        if name == "screw":
            part = part.cut(cyl(2.7, Z_KIT - 12, Z_KIT + 0.01, HOLE_X, HOLE_Y))
        L.append(f"overlap box vs {name}: {inter(bx, part):.3f} mm3")
    L.append(f"overlap box vs bottom plate: {inter(bx, plate):.3f} mm3")
    L.append(f"overlap box vs arm screw heads: {inter(bx, heads):.3f} mm3")
    L.append(f"overlap power board vs J12 jumper keep-out: {inter(P['pwr'], P['jumpers']):.3f} mm3")
    keys = bx.intersect(box(-100, 100, -40, 40, 0, KEY_H))
    L.append(f"keys above z=0: {vol(keys):.1f} mm3, key/plate clash {inter(keys, plate):.3f} mm3")
    open(f"{out}/fit_check_v2.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
