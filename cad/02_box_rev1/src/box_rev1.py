# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/02_box_rev1/src/box_rev1.py
#  Part    : 02_box_rev1
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: box(), cyl(), make_box(), make_plate(), make_parts(), make_imu_on_plate(),
#    main()
# ==============================================================================================

# ELC275 drone - electronics box REV 1 (CadQuery 2.x)
# Ethan DiMeo
#
# Holds: PIC32MZ EF Starter Kit, Bluefruit LE SPI Friend, the power / Bluetooth-kill board
# (placeholder bay until the real parts are measured), battery strapped underneath.
# The IMU is NOT in the box: it stays on the slot adapter in the middle of the bottom plate.
#
# Run:   python box_rev1.py OUTPUT_DIR
#
# Frame: origin = frame center, +X = FRONT (red arms), +Y = left, +Z = up,
#        z = 0 = UNDERSIDE of the F450 bottom plate (the box hangs under it). Sizes in mm.
#
# Every number you might want to change is in the PARAMETERS block.

import sys, json, os
import cadquery as cq

# ============================== PARAMETERS ==================================
# F450 bottom plate (sliced from an open F450 model, matches the photos; +/-1 mm)
PLATE_T      = 1.6
SLOT_X       = 70.1      # tab slot centers at x = +/-70.1
SLOT_Y       = 17.0      #                    y = +/-17.0
TAB_HALF_W   = 26.0      # end plates stay inside the plate tab (tab is +/-28.5 wide)

# Box (inside), walls
IN_X0, IN_X1 = -60.0, 60.0          # inside length 120
IN_HW        = 32.0                 # inside half-width (64 inside)
WALL, FLOOR  = 2.0, 2.4

# PIC32MZ EF Starter Kit (MEASURE FIRST).  J12 is on the UNDERSIDE (front edge), so the
# board sits up on supports and the J12 jumpers hang down under it.
KIT_L, KIT_W, KIT_T = 71.5, 51.0, 1.6
KIT_X0       = -56.0     # rear (USB) edge. USB plugs come in through the rear window.
KIT_LIFT     = 30.0      # board underside above the floor (J12 jumpers hang down ~30)
TOP_CLEAR    = 14.0      # board top to the frame plate (take the Ethernet daughter board off)
HOLE_FROM_USB_EDGE = 19.2   # the kit's one mounting hole (d3.0): from the USB edge ...
HOLE_FROM_J5_EDGE  = 25.8   # ... and from the J5 (USB-A) side edge     (MEASURE FIRST)
SIDE_CLEAR   = 1.0       # gap between the board's long edges and the side lips
POST_PILOT   = 2.2       # M2.5 self-tapping screw into the center post
J12_IN       = 6.0       # J12 header body (underside) reaches this far in from the front edge

# Bluefruit LE SPI Friend: slides DOWN into a card slot (2 grooved rails) on the left wall
BLE_W, BLE_H, BLE_T = 22.86, 25.4, 1.6     # header edge = BLE_W, goes on TOP
BLE_XC       = 44.0      # center of the slot along X (front bay)
BLE_Y        = 26.0      # board plane (y) - 6 mm off the left wall for parts on the back
BLE_GAP      = 0.3       # side play in the grooves
BLE_GROOVE   = 2.0       # groove width (board is 1.6)

# Power / kill bay (front bay, x 17..60). Rev 1 = zip-tie slots + XT60 window.
ZIP_X        = (26.0, 50.0)   # 2 zip ties across the bay ...
ZIP_Y        = (-28.0, 14.0)  # ... each down through a floor slot at these y (under the floor, over the battery)
XT60_W, XT60_H = 16.6, 9.0    # front-wall window: an XT60 fits through it (body 15.5 x 8.1)
XT60_Y       = -12.0

# ESC signal (PWM) cables: U-slots in the top of both long walls, closed by the frame plate
PWM_X        = (12.0, 28.0)   # 2 per side: left wall = FL + RL, right wall = FR + RR
PWM_W, PWM_D = 4.5, 12.0

# Frame mounting (same as the case that passed the fit check): keys + M3 bolts in the tab slots
EAR_X1       = 86.0      # end plates reach out to x = +/-86
EAR_T        = 4.0
KEY_X0, KEY_X1 = 63.0, 73.0
KEY_W, KEY_H = 3.0, 1.4  # 0.5 mm clearance each side in the 4 mm slot, shorter than the 1.6 plate
BOLT_X, BOLT_D = 79.0, 3.4
NUT_AF, NUT_DEPTH = 5.7, 2.6

# Battery: Zeee 3S 2200 shorty 75 x 34 x 26.5 (the regular 102 x 34 x 23 also fits), 2 velcro straps
BAT_L, BAT_W, BAT_H = 75.0, 34.0, 26.5
STRAP_W, STRAP_X = 20.0, 28.0
# ============================================================================

H_IN        = KIT_LIFT + KIT_T + TOP_CLEAR          # 45.6 inside height
Z_FLOOR_TOP = -H_IN
Z_BOTTOM    = Z_FLOOR_TOP - FLOOR
Z_KIT       = Z_FLOOR_TOP + KIT_LIFT                # board underside
KIT_X1      = KIT_X0 + KIT_L                        # front (J12) edge
HOLE_X      = KIT_X0 + HOLE_FROM_USB_EDGE
HOLE_Y      = -KIT_W / 2 + HOLE_FROM_J5_EDGE        # J5 is on the right (-Y) side
OUT_HW      = IN_HW + WALL


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(
        ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl(d, z0, z1, x, y):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def make_box():
    # 1. open-top tray; the frame plate is the lid
    b = box(IN_X0 - WALL, IN_X1 + WALL, -OUT_HW, OUT_HW, Z_BOTTOM, 0).edges("|Z").fillet(3.0)
    b = b.cut(box(IN_X0, IN_X1, -IN_HW, IN_HW, Z_FLOOR_TOP, 1).edges("|Z").fillet(1.5))

    # 2. end plates against the plate tabs: key into each tab slot + M3 bolt + nut trap
    for sx in (1, -1):
        x0, x1 = sorted((sx * IN_X1, sx * EAR_X1))
        b = b.union(box(x0, x1, -TAB_HALF_W, TAB_HALF_W, -EAR_T, 0).edges("|Z").fillet(2.0))
        # 45 deg gussets under each end plate (prints without supports). Rear: none in
        # front of the USB plugs.
        gys = (-TAB_HALF_W + 1.5, 0.0, TAB_HALF_W - 1.5) if sx > 0 else \
              (-TAB_HALF_W + 1.5, -11.0, 10.2, TAB_HALF_W - 1.5)
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

    # 3. PIC32 kit: center screw post at its mounting hole + 4 side supports with lips.
    #    The screw holds it down; the lips stop it turning. Nothing touches the USB edge
    #    or the J12 edge.
    post = cyl(14.0, Z_FLOOR_TOP, Z_FLOOR_TOP + 1.0, HOLE_X, HOLE_Y)
    post = post.union(cq.Workplane("XY").circle(7.0).workplane(offset=4.0).circle(4.5).loft()
                      .translate((HOLE_X, HOLE_Y, Z_FLOOR_TOP + 1.0)))            # flared foot
    post = post.union(cyl(9.0, Z_FLOOR_TOP + 1.0, Z_KIT - 6.0, HOLE_X, HOLE_Y))
    post = post.union(cyl(5.5, Z_KIT - 6.0, Z_KIT, HOLE_X, HOLE_Y))               # thin top: clears parts under the board
    post = post.cut(cyl(POST_PILOT, Z_KIT - 12.0, Z_KIT + 1, HOLE_X, HOLE_Y))
    b = b.union(post)
    lip_in = KIT_W / 2 + SIDE_CLEAR
    for (tx0, tx1) in ((KIT_X0 + 10, KIT_X0 + 16), (-3.0, 3.0)):
        for sy in (1, -1):
            t = box(tx0, tx1, sy * (KIT_W / 2 - 2.5), sy * IN_HW, Z_FLOOR_TOP, Z_KIT)        # support under the edge
            t = t.union(box(tx0, tx1, sy * lip_in, sy * IN_HW, Z_KIT, Z_KIT + KIT_T + 2.0))  # lip beside the edge
            b = b.union(t)

    # 4. rear window: J3 (program), J11 (PuTTY/tuner) on top, J4 (power) underneath
    b = b.cut(box(IN_X0 - WALL - 1, IN_X0 + 1, -22.5, 22.5, Z_KIT - 8.4, -5.0))

    # 5. Bluefruit card slot: 2 rails on the left wall, grooves face each other, open at the top.
    #    The board slides down until it sits on the groove bottoms. Header edge up.
    z_rail_top = Z_FLOOR_TOP + 3.0 + BLE_H + 1.0
    for s in (1, -1):
        face = BLE_XC + s * (BLE_W / 2 + BLE_GAP)          # rail face toward the board
        r = box(face, face + s * 4.0, BLE_Y - 4.0, IN_HW + 0.5, Z_FLOOR_TOP, z_rail_top)
        r = r.cut(box(face - s * 0.1, face + s * (1.5 + BLE_GAP), BLE_Y - BLE_GROOVE / 2, BLE_Y + BLE_GROOVE / 2,
                      Z_FLOOR_TOP + 3.0, z_rail_top + 1))
        b = b.union(r)

    # 6. power bay: zip-tie slots through the floor, XT60 window in the front wall
    for zx in ZIP_X:
        for zy in ZIP_Y:
            b = b.cut(box(zx - 2.6, zx + 2.6, zy - 1.0, zy + 1.0, Z_BOTTOM - 1, Z_FLOOR_TOP + 1))
    b = b.cut(box(IN_X1 - 1, IN_X1 + WALL + 1, XT60_Y - XT60_W / 2, XT60_Y + XT60_W / 2,
                  Z_FLOOR_TOP + 3.0, Z_FLOOR_TOP + 3.0 + XT60_H))

    # 7. PWM cable slots in the top of both long walls (the frame plate closes them)
    for px in PWM_X:
        for sy in (1, -1):
            b = b.cut(box(px - PWM_W / 2, px + PWM_W / 2, sy * (IN_HW - 1), sy * (OUT_HW + 1), -PWM_D, 1))

    # 7b. side windows under the kit (reach the J12 jumpers, less plastic) + floor cutout under the kit
    for sy in (1, -1):
        b = b.cut(box(-38.0, -8.0, sy * (IN_HW - 1), sy * (OUT_HW + 1), Z_FLOOR_TOP + 6.0, Z_FLOOR_TOP + 24.0))
    b = b.cut(box(-28.0, -6.0, -22.0, 22.0, Z_BOTTOM - 1, Z_FLOOR_TOP + 1).edges("|Z").fillet(4.0))

    # 8. battery strap ears at floor level on the long walls
    for sx in (1, -1):
        for sy in (1, -1):
            ex = sx * STRAP_X
            ear = box(ex - STRAP_W / 2 - 4, ex + STRAP_W / 2 + 4, sy * (OUT_HW - 0.5), sy * (OUT_HW + 8),
                      Z_BOTTOM, Z_BOTTOM + FLOOR).edges("|Z").fillet(1.5)
            slot = box(ex - STRAP_W / 2 - 1, ex + STRAP_W / 2 + 1,
                       sy * (OUT_HW + 3.2) - 1.75, sy * (OUT_HW + 3.2) + 1.75, Z_BOTTOM - 1, Z_BOTTOM + FLOOR + 1)
            b = b.union(ear).cut(slot)

    # 9. FRONT arrow engraved on the outside of the right (-Y) wall
    arrow = (cq.Workplane("XZ").polyline([(-8, -3), (2, -3), (2, -6), (10, 0), (2, 6), (2, 3), (-8, 3)])
             .close().extrude(-0.6).translate((0, -OUT_HW, Z_FLOOR_TOP + 22)))
    b = b.cut(arrow)
    return b


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


def make_parts():
    """Plain blocks for the preview and the clash check."""
    kit = box(KIT_X0, KIT_X1, -KIT_W / 2, KIT_W / 2, Z_KIT, Z_KIT + KIT_T)
    kit = kit.cut(cyl(3.0, Z_KIT - 1, Z_KIT + 3, HOLE_X, HOLE_Y))
    # top-side USB connectors at the rear edge (J11 mini-B, J3 mini-B, J5 USB-A), approx.
    usb = (box(KIT_X0 - 0.5, KIT_X0 + 9, 14.0, 21.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 4)
           .union(box(KIT_X0 - 0.5, KIT_X0 + 9, -1.0, 6.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 4))
           .union(box(KIT_X0 - 1.0, KIT_X0 + 14, -23.5, -6.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 7)))
    j12 = box(KIT_X1 - J12_IN, KIT_X1 - 0.5, -25.4, 25.4, Z_KIT - 8.5, Z_KIT)          # header under the board
    jumpers = box(KIT_X1 - J12_IN, KIT_X1 - 0.5, -25.4, 25.4, Z_KIT - 8.5 - 14.0, Z_KIT - 8.5)  # female housings
    ble = box(BLE_XC - BLE_W / 2, BLE_XC + BLE_W / 2, BLE_Y - BLE_T / 2, BLE_Y + BLE_T / 2,
              Z_FLOOR_TOP + 3.0, Z_FLOOR_TOP + 3.0 + BLE_H)
    bat = box(-BAT_L / 2, BAT_L / 2, -BAT_W / 2, BAT_W / 2, Z_BOTTOM - BAT_H, Z_BOTTOM)
    relay = box(21.0, 47.0, -29.0, 19.0, Z_FLOOR_TOP, Z_FLOOR_TOP + 19.0)                # PLACEHOLDER power board
    xt60 = box(IN_X1 - 8, IN_X1 + WALL + 8, XT60_Y - 7.75, XT60_Y + 7.75,
               Z_FLOOR_TOP + 3.4, Z_FLOOR_TOP + 3.4 + 8.1)                                 # XT60 in its window
    screw = cyl(2.5, Z_KIT - 6.4, Z_KIT + KIT_T, HOLE_X, HOLE_Y).union(
        cyl(4.5, Z_KIT + KIT_T, Z_KIT + KIT_T + 1.8, HOLE_X, HOLE_Y))                    # M2.5 x 8
    return dict(kit=kit, usb=usb, j12=j12, jumpers=jumpers, ble=ble, bat=bat, relay=relay, xt60=xt60, screw=screw)


def make_imu_on_plate():
    # the slot adapter + IMU board on top of the plate (adapter frame: z = 0 = top of plate)
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in (("..", "imu_adapter", "src"), ("..", "..", "imu_adapter", "src")):   # cad/src or cad/box_rev1/src
        sys.path.insert(0, os.path.join(here, *rel))
    import imu_adapter as IA
    a = IA.make_adapter().translate((0, 0, PLATE_T))
    imu = IA.make_imu().translate((0, 0, PLATE_T))
    return a, imu


def main(out):
    os.makedirs(out, exist_ok=True)
    bx = make_box()
    plate = make_plate()
    P = make_parts()
    adapter, imu = make_imu_on_plate()

    cq.exporters.export(bx, f"{out}/box_rev1.step")
    cq.exporters.export(bx.translate((0, 0, -Z_BOTTOM)), f"{out}/box_rev1.stl", tolerance=0.05, angularTolerance=0.1)

    asm = cq.Assembly(name="ELC275_box_rev1")
    asm.add(plate, name="F450_bottom_plate", color=cq.Color(0.15, 0.15, 0.15))
    asm.add(bx, name="box_rev1_printed", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(P["kit"].union(P["usb"]).union(P["j12"]), name="PIC32MZ_EF_starter_kit", color=cq.Color(0.1, 0.5, 0.2))
    asm.add(P["jumpers"], name="J12_jumper_housings_keepout", color=cq.Color(0.2, 0.2, 0.2))
    asm.add(P["screw"], name="M2.5x8_screw", color=cq.Color(0.6, 0.6, 0.6))
    asm.add(P["ble"], name="Bluefruit_SPI_Friend", color=cq.Color(0.1, 0.2, 0.7))
    asm.add(P["relay"], name="PLACEHOLDER_power_kill_board", color=cq.Color(0.9, 0.4, 0.1))
    asm.add(P["xt60"], name="XT60_male", color=cq.Color(0.9, 0.8, 0.1))
    asm.add(P["bat"], name="Battery_3S_2200", color=cq.Color(0.8, 0.6, 0.1))
    asm.add(adapter, name="IMU_slot_adapter", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(imu, name="LSM6DSO_IMU", color=cq.Color(0.8, 0.1, 0.1))
    asm.save(f"{out}/assembly_rev1.step")

    # ---- fit check ----
    def vol(s):
        try:
            return s.val().Volume()
        except Exception:
            return 0.0
    def inter(a, b):
        try:
            return a.intersect(b).val().Volume()
        except Exception:
            return 0.0
    L = []
    L.append(f"box outside {IN_X1 - IN_X0 + 2 * WALL:.0f} x {2 * OUT_HW:.0f} mm (with end plates {2 * EAR_X1:.0f} long), "
             f"inside {IN_X1 - IN_X0:.0f} x {2 * IN_HW:.0f} x {H_IN:.1f} mm")
    L.append(f"box bottom z = {Z_BOTTOM:.1f}, battery bottom z = {Z_BOTTOM - BAT_H:.1f} (below the bottom plate)")
    L.append(f"PIC32 board: z {Z_KIT:.1f} .. {Z_KIT + KIT_T:.1f}, x {KIT_X0:.1f} .. {KIT_X1:.1f}; "
             f"screw post at ({HOLE_X:.1f}, {HOLE_Y:.1f}); board top to plate {-(Z_KIT + KIT_T):.1f} mm")
    L.append(f"box volume {vol(bx)/1000:.1f} cm3 -> about {vol(bx)/1000*1.24*0.6:.0f} g PLA (3 walls, 15% infill)")
    for name in ("kit", "usb", "j12", "jumpers", "ble", "bat", "relay", "xt60", "screw"):
        part = P[name]
        if name == "screw":
            part = part.cut(cyl(2.7, Z_KIT - 12, Z_KIT + 0.01, HOLE_X, HOLE_Y))   # threaded part bites into the pilot on purpose
        L.append(f"overlap box vs {name}: {inter(bx, part):.3f} mm3")
    L.append(f"overlap box vs bottom plate: {inter(bx, plate):.3f} mm3")
    keys = bx.intersect(box(-100, 100, -40, 40, 0, KEY_H))
    L.append(f"keys above z=0: {vol(keys):.1f} mm3 (must sit inside the tab slots), key/plate clash {inter(keys, plate):.3f} mm3")
    L.append(f"overlap BLE vs placeholder power board: {inter(P['ble'], P['relay']):.3f} mm3")
    open(f"{out}/fit_check_rev1.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
