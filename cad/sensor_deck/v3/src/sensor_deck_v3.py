# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/sensor_deck/v3/src/sensor_deck_v3.py
#  Part    : sensor_deck
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: box(), cyl(), col_x(), row_y(), make_deck(), for_print(), pin(), make_imu(),
#    make_imu_headers(), make_pins(), make_ble(), make_ble_parts(), make_hub(), make_links(),
#    make_plate(), make_slot_patch(), vol(), main()
# ==============================================================================================

# ELC275 drone - SENSOR DECK v3: IMU + Bluefruit + hub board, middle of the bottom plate
# Ethan DiMeo
#
# Replaces the IMU slot adapter v2. Same keys into the 3 center slots + 2 screws in the middle
# slot, so the IMU still sits square in the exact middle of the frame.
#
#   IMU ........ center, X arrow to the FRONT (red arms) -> its GND...CS pin row is on the LEFT
#   Bluefruit .. just left of the IMU, header edge facing the IMU, antenna end pointing outward
#   Hub board .. a 9 x 4 hole strip of perfboard laid ON TOP of both pin rows (the pins come up
#                through it). Shared lines are joined on the strip; a 7-pin header on the strip
#                is the one cable to J12.
#
# Prints top-face DOWN (STL is pre-flipped), no supports, like v2.
# Frame: +X = front, +Y = left, z = 0 = TOP of the bottom plate. mm.
#
# Run:  python sensor_deck_v3.py OUTPUT_DIR

import sys, os, json, math
import cadquery as cq

# ============================== PARAMETERS ==================================
# plate center slots (Ethan's caliper)
OUTER_EDGES, SLOT_W, CENTER_LEN, OUTER_LEN, PLATE_T = 20.3, 3.0, 38.0, 28.0, 1.6
KEY_CLEAR, KEY_W, KEY_L, KEY_H = 0.10, 1.8, 16.0, 1.4
MID_KEY_W, MID_KEY_L = 1.6, 19.0

BASE_T       = 4.0       # deck thickness, IMU and Bluefruit sit on this flat top
IMU_SIZE     = 25.4
IMU_HOLE_SP  = 20.32
PILOT_D, PILOT_DEPTH = 2.2, 3.6      # M2.5 self-tapping for the IMU
SCREW_X, SCREW_D = 17.0, 2.8         # 2 x M2.5 down through the middle slot

PITCH        = 2.54
ROW_IMU      = 11.43     # the IMU's pin rows are 0.05" in from its edges (y = +/-11.43)
COL0_X       = 11.43     # hub column 0 (front end). column c is at x = COL0_X - c * PITCH
ROW_BLE      = ROW_IMU + 3 * PITCH    # Bluefruit pin row: 3 rows over -> y = 19.05

BLE_W, BLE_L = 22.86, 25.4           # Bluefruit: header edge (along X) x length (along Y)
BLE_ROW_IN   = 1.27                  # its pin row is 0.05" in from the header edge
BLE_HOLE_SP, BLE_HOLE_IN = 17.78, 2.54
BLE_PILOT, BLE_PILOT_DEPTH = 1.6, 3.6   # M2 self-tapping into its 2 holes

HEADER_H     = 2.5       # black plastic of a 0.1" header
BOARD_T      = 1.6
# ============================================================================

OUTER_C = OUTER_EDGES / 2 - SLOT_W / 2
BLE_Y0 = ROW_BLE - BLE_ROW_IN                  # header edge of the Bluefruit
BLE_Y1 = BLE_Y0 + BLE_L                        # antenna end
BLE_XC = COL0_X - 4 * PITCH                    # Bluefruit pin 4 (IRQ) is its middle pin
Z_PINS = BASE_T + BOARD_T + HEADER_H           # top of both header plastics = bottom of the hub strip
HUB_X0, HUB_X1 = COL0_X - 8 * PITCH - PITCH / 2, COL0_X + PITCH / 2
HUB_Y0, HUB_Y1 = ROW_IMU - PITCH / 2, ROW_BLE + PITCH / 2


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl(d, z0, z1, x, y):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def col_x(c):
    return COL0_X - c * PITCH


def row_y(r):            # r = 0 IMU pins, 1, 2 = output header, 3 = Bluefruit pins
    return ROW_IMU + r * PITCH


def make_deck(key_clear=KEY_CLEAR):
    d = box(-22, 22, -13, 13, 0, BASE_T).edges("|Z").fillet(4.0)
    wing = box(BLE_XC - BLE_W / 2 - 1.4, BLE_XC + BLE_W / 2 + 1.4, 10.0, BLE_Y1 + 1.4, 0, BASE_T).edges("|Z").fillet(3.0)
    d = d.union(wing)

    # keys (same as v2)
    y_out = OUTER_EDGES / 2 - key_clear
    for s in (1, -1):
        d = d.union(box(-KEY_L / 2, KEY_L / 2, s * y_out, s * (y_out - KEY_W), -KEY_H, 0).edges("<Z").chamfer(0.4))
    d = d.union(box(-MID_KEY_L / 2, MID_KEY_L / 2, -MID_KEY_W / 2, MID_KEY_W / 2, -KEY_H, 0).edges("<Z").chamfer(0.3))

    # IMU: 4 blind pilots
    h = IMU_HOLE_SP / 2
    for sx in (1, -1):
        for sy in (1, -1):
            d = d.cut(cyl(PILOT_D, BASE_T - PILOT_DEPTH, BASE_T + 1, sx * h, sy * h))
    # through reliefs under the IMU's edges (pin solder joints), as v2
    d = d.cut(box(-7.4, 7.4, -14, -10.2, -1, BASE_T + 1))
    d = d.cut(box(-7.4, 7.4, 10.2, 13.0, -1, BASE_T + 1))
    for s in (1, -1):
        d = d.cut(box(s * 10.2, s * 13.0, -7.4, 7.4, -1, BASE_T + 1))
    # relief under the Bluefruit's header solder joints
    d = d.cut(box(BLE_XC - BLE_W / 2 - 0.3, BLE_XC + BLE_W / 2 + 0.3, BLE_Y0 - 0.8, BLE_Y0 + 3.2, -1, BASE_T + 1))
    # Bluefruit: 2 blind pilots (M2) at its far-end holes
    for s in (1, -1):
        d = d.cut(cyl(BLE_PILOT, BASE_T - BLE_PILOT_DEPTH, BASE_T + 1, BLE_XC + s * BLE_HOLE_SP / 2, BLE_Y1 - BLE_HOLE_IN))
    # deck screws through the middle slot
    for sx in (1, -1):
        d = d.cut(cyl(SCREW_D, -5, 10, sx * SCREW_X, 0))
    # FRONT arrow engraved on top
    arrow = (cq.Workplane("XY").polyline([(-2.5, -1.2), (0.8, -1.2), (0.8, -2.6), (3.6, 0), (0.8, 2.6), (0.8, 1.2), (-2.5, 1.2)])
             .close().extrude(0.6).translate((16.8, -7.2, BASE_T - 0.6)))
    return d.cut(arrow)


def for_print(shape):
    return shape.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, BASE_T))


# ---------------------------------------------------------------- parts (preview)
# Hub wiring (see the wiring diagram):  cols 0..8 front -> rear, rows 0 IMU / 1 OUT / 2 / 3 BLE
# The 7-pin output header goes in from UNDER the strip (plastic below, long pins up), so every
# joint is soldered on top.
IMU_PINS = {2: "GND", 3: "3V3", 4: "SDA", 5: "SCL", 6: "SDO", 7: "CS"}           # row 0 (IMU's left row)
BLE_PINS = {0: "VIN", 1: "RST", 2: "GND", 3: "DFU", 4: "IRQ", 5: "CS", 6: "MOSI", 7: "MISO", 8: "SCK"}  # row 3
OUT_ROW = 1
OUT_PINS = {2: ("GND", 6), 3: ("3V3", 1), 4: ("MOSI", 37), 5: ("SCK", 23), 6: ("MISO", 7),
            7: ("CS IMU", 16), 8: ("CS BLE", 13)}                                   # row 1 -> J12 pin
BRIDGES = [((c, 0), (c, 1)) for c in range(2, 8)] + [((2, 3), (2, 2)), ((2, 2), (2, 1))]   # solder bridges
LINKS = [((0, 3), (3, 1), "3V3"), ((6, 3), (4, 1), "MOSI"), ((8, 3), (5, 1), "SCK"),
         ((7, 3), (6, 1), "MISO"), ((5, 3), (8, 1), "CS BLE")]           # 5 short insulated wire links


def pin(x, y, z0, z1):
    return box(x - 0.32, x + 0.32, y - 0.32, y + 0.32, z0, z1)


def make_imu():
    b = box(-IMU_SIZE / 2, IMU_SIZE / 2, -IMU_SIZE / 2, IMU_SIZE / 2, BASE_T, BASE_T + BOARD_T)
    for s in (1, -1):
        b = b.cut(cyl(3.0, BASE_T - 1, BASE_T + 3, 10.16, s * 10.16)).cut(cyl(3.0, BASE_T - 1, BASE_T + 3, -10.16, s * 10.16))
    return b


def make_imu_headers():
    p = None
    for s in (1, -1):
        h = box(-7.62, 7.62, s * ROW_IMU - 1.27, s * ROW_IMU + 1.27, BASE_T + BOARD_T, Z_PINS)
        p = h if p is None else p.union(h)
    return p


def make_pins():
    p = None
    top = Z_PINS + BOARD_T + 2.0
    for c in IMU_PINS:
        q = pin(col_x(c), row_y(0), BASE_T, top)
        p = q if p is None else p.union(q)
    for c in range(2, 8):                                  # the IMU's other row (unused)
        p = p.union(pin(col_x(c), -ROW_IMU, BASE_T, Z_PINS + 3.5))
    for c in BLE_PINS:
        p = p.union(pin(col_x(c), row_y(3), BASE_T, top))
    for c in OUT_PINS:                                     # output header: plastic under the strip, long pins up
        p = p.union(pin(col_x(c), row_y(OUT_ROW), Z_PINS - HEADER_H - 0.5, Z_PINS + BOARD_T + 4.4))
    return p


def make_ble():
    b = box(BLE_XC - BLE_W / 2, BLE_XC + BLE_W / 2, BLE_Y0, BLE_Y1, BASE_T, BASE_T + BOARD_T)
    for s in (1, -1):
        b = b.cut(cyl(2.5, BASE_T - 1, BASE_T + 3, BLE_XC + s * BLE_HOLE_SP / 2, BLE_Y1 - BLE_HOLE_IN))
    return b


def make_ble_parts():
    hdr = box(BLE_XC - BLE_W / 2, BLE_XC + BLE_W / 2, ROW_BLE - 1.27, ROW_BLE + 1.27, BASE_T + BOARD_T, Z_PINS)
    module = box(BLE_XC - 5.5, BLE_XC + 5.5, BLE_Y0 + 7.5, BLE_Y1 - 2.0, BASE_T + BOARD_T, BASE_T + BOARD_T + 2.2)
    return hdr, module


def make_hub():
    s = box(HUB_X0, HUB_X1, HUB_Y0, HUB_Y1, Z_PINS, Z_PINS + BOARD_T)
    out_hdr = box(col_x(8) - 1.27, col_x(2) + 1.27, row_y(OUT_ROW) - 1.27, row_y(OUT_ROW) + 1.27,
                  Z_PINS - HEADER_H, Z_PINS)                       # plastic UNDER the strip
    return s, out_hdr


def make_links():
    zt = Z_PINS + BOARD_T + 0.6
    w = None
    for (a, b, _) in LINKS:
        p0 = cq.Vector(col_x(a[0]), row_y(a[1]), zt)
        p1 = cq.Vector(col_x(b[0]), row_y(b[1]), zt)
        mid = (p0 + p1) * 0.5 + cq.Vector(0, 0, 1.6)
        for (q0, q1) in ((p0, mid), (mid, p1)):
            v = q1 - q0
            s = cq.Workplane("XY").add(cq.Solid.makeCylinder(0.45, v.Length, q0, v.normalized()))
            w = s if w is None else w.union(s)
    return w


def make_plate():
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in (("..", "..", "src"), ("..", "..", "box_v2", "src"), (".",)):
        f = os.path.join(here, *rel, "f450_bottom_plate.json")
        if os.path.exists(f):
            break
    d = json.load(open(f))
    p = cq.Workplane("XY").polyline(d["exterior"][:-1]).close().extrude(PLATE_T).translate((0, 0, -PLATE_T))
    for ring in d["interiors"]:
        if len(ring) > 3:
            try:
                p = p.cut(cq.Workplane("XY").polyline(ring[:-1]).close().extrude(PLATE_T + 2).translate((0, 0, -PLATE_T - 1)))
            except Exception:
                pass
    return p


def make_slot_patch():
    p = box(-30, 30, -20, 20, -PLATE_T, 0)
    p = p.cut(box(-CENTER_LEN / 2, CENTER_LEN / 2, -SLOT_W / 2, SLOT_W / 2, -5, 5).edges("|Z").fillet(SLOT_W / 2 - 0.01))
    for s in (1, -1):
        p = p.cut(box(-OUTER_LEN / 2, OUTER_LEN / 2, s * (OUTER_C - SLOT_W / 2), s * (OUTER_C + SLOT_W / 2), -5, 5)
                  .edges("|Z").fillet(SLOT_W / 2 - 0.01))
    return p


def vol(s):
    try:
        return s.val().Volume()
    except Exception:
        return 0.0


def main(out):
    os.makedirs(out, exist_ok=True)
    deck = make_deck()
    for name, clr in (("snug", 0.05), ("normal", KEY_CLEAR), ("loose", 0.20)):
        cq.exporters.export(for_print(make_deck(clr)), f"{out}/sensor_deck_v3_{name}.stl", tolerance=0.02, angularTolerance=0.05)
    cq.exporters.export(deck, f"{out}/sensor_deck_v3.step")

    imu, imu_h = make_imu(), make_imu_headers()
    ble = make_ble(); ble_h, ble_m = make_ble_parts()
    hub, out_h = make_hub(); pins = make_pins(); links = make_links()
    asm = cq.Assembly(name="sensor_deck_v3")
    asm.add(make_plate(), name="F450_bottom_plate", color=cq.Color(0.15, 0.15, 0.15))
    asm.add(deck, name="sensor_deck_v3_printed", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(imu.union(imu_h), name="LSM6DSO_IMU", color=cq.Color(0.8, 0.1, 0.1))
    asm.add(ble.union(ble_h).union(ble_m), name="Bluefruit_SPI_Friend", color=cq.Color(0.1, 0.2, 0.7))
    asm.add(hub.union(out_h), name="hub_perfboard_9x4_and_7pin_header", color=cq.Color(0.1, 0.35, 0.15))
    asm.add(pins, name="header_pins", color=cq.Color(0.85, 0.75, 0.3))
    asm.add(links, name="wire_links", color=cq.Color(0.95, 0.5, 0.1))
    asm.save(f"{out}/sensor_deck_v3_assembly.step")

    patch = make_slot_patch()
    R = []
    R.append(f"keys vs slot patch overlap {vol(deck.intersect(patch)):.4f} mm3 (0 = keys fit)")
    for nm, part in (("IMU", imu), ("Bluefruit", ble), ("hub strip", hub)):
        R.append(f"deck vs {nm} overlap {vol(deck.intersect(part)):.4f} mm3")
    R.append(f"IMU vs Bluefruit overlap {vol(imu.intersect(ble)):.4f} mm3, gap between them "
             f"{BLE_Y0 - IMU_SIZE / 2:.2f} mm")
    R.append(f"hub strip {HUB_X1 - HUB_X0:.2f} x {HUB_Y1 - HUB_Y0:.2f} mm (9 x 4 holes), bottom at z = {Z_PINS:.1f}, "
             f"top at z = {Z_PINS + BOARD_T:.1f}; cable pins stick up 4.4 mm, plugged cable top ~ "
             f"{Z_PINS + BOARD_T + 14:.0f} mm above the plate (plates are 36.8 apart)")
    R.append(f"cable header plastic vs IMU headers {vol(out_h.intersect(imu_h)):.4f} mm3, vs Bluefruit {vol(out_h.intersect(ble.union(ble_h))):.4f} mm3, "
             f"vs IMU board {vol(out_h.intersect(imu)):.4f} mm3")
    bb = deck.val().BoundingBox()
    R.append(f"deck {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, volume {vol(deck)/1000:.2f} cm3 ~ {vol(deck)/1000*1.24:.1f} g")
    open(f"{out}/fit_check_v3.txt", "w").write("\n".join(R) + "\n")
    print("\n".join(R))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
