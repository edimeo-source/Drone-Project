# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/04_box_v3_tray_and_collar/src/collar_v3_1.py
#  Part    : 04_box_v3_tray_and_collar
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: bracket_outline(), per_corner(), pockets(), corridor(), make_legs_measured(),
#    make_collar_v31(), main()
# ==============================================================================================

# ELC275 drone - box V3, COLLAR v3.1 (Nov 20, 2026) - Ethan DiMeo
# Same collar as V3 (same joint to the printed tray, same keys / bolts into the frame), but the
# 4 landing-gear pockets are now traced from Ethan's caliper measurements and close-up photos:
#   leg bracket: 23.2 mm long (nose to outer edge), 28.5 mm wide, 7.3 mm thick (calipers)
#   leg root:    14.9 mm wide (calipers)
#   bracket screws = the plate's 2 arm screws; nose points to the frame center, 18.5 mm in from them.
# Each pocket goes straight down from the plate to a 2 mm floor at the joint face, so the collar
# prints with no supports and still closes the tray. Nothing else changed.
#
# Run:  python collar_v3_1.py OUTPUT_DIR
import sys, os, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B

# ---- leg bracket, measured (local frame per arm: u = out along the arm from the frame center,
#      s = across the arm; the 2 bracket screws are the plate's arm screws at u = 55.5, s = +/-10.25)
SCREW_U, SCREW_S = 55.5, 10.25
BR_NOSE_U  = SCREW_U - 18.5      # nose flat (photos: 17.7 - 18.7 in from the screw line)
BR_NOSE_HS = 5.3                 # half width of the nose flat
BR_CORNER_U = SCREW_U - 10.5     # where the nose chamfers reach full width (photos: 7.5 - 11)
BR_HS      = 14.25               # half width (caliper 28.5)
BR_OUT_U   = BR_NOSE_U + 23.2    # outer end (caliper 23.2)
BR_T       = 7.3                 # thickness under the plate (caliper)
HEAD_D, HEAD_H = 5.5, 3.0        # M3 socket head on each screw
LEG_HS     = 7.45                # leg half width (caliper 14.9)
FIN_HS     = 2.2                 # the thin rib that runs from the nose up into the leg
POCKET_CLR = 1.0                 # gap kept around the bracket
FLOOR_T    = 2.0                 # pocket floor at the joint face
CORRIDOR_U, CORRIDOR_HS = SCREW_U - 1.0, 9.5   # floor left open where the leg leaves the box


def bracket_outline(grow, u_far=140.0):
    """bracket footprint (+ everything further out along the arm), grown by 'grow' mm"""
    pts = [(BR_NOSE_U, -BR_NOSE_HS), (BR_CORNER_U, -BR_HS), (u_far, -BR_HS),
           (u_far, BR_HS), (BR_CORNER_U, BR_HS), (BR_NOSE_U, BR_NOSE_HS)]
    w = cq.Workplane("XY").polyline(pts).close()
    if grow > 0:
        w = w.offset2D(grow, kind="arc")
    return w


def per_corner(make):
    out = None
    for sx in (1, -1):
        for sy in (1, -1):
            ang = math.degrees(math.atan2(sy, sx))
            c = make().rotate((0, 0, 0), (0, 0, 1), ang)
            out = c if out is None else out.union(c)
    return out


def pockets(grow, z_bot, z_top=10.0):
    return per_corner(lambda: bracket_outline(grow).extrude(z_top - z_bot).translate((0, 0, z_bot)))


def corridor(z_bot, z_top):
    return per_corner(lambda: B.box(CORRIDOR_U, 140, -CORRIDOR_HS, CORRIDOR_HS, z_bot, z_top))


def make_legs_measured():
    """the 4 legs as measured (bracket, screw heads, fin, leg) - for the fit check and pictures"""
    def one():
        br = bracket_outline(0.0, u_far=BR_OUT_U).extrude(BR_T).translate((0, 0, -BR_T))
        heads = B.U(*[B.cyl(HEAD_D, -BR_T - HEAD_H, -BR_T, SCREW_U, s) for s in (-SCREW_S, SCREW_S)])
        # fin: from the nose (at the bracket face) out to where it meets the leg (~20 mm down)
        fin = (cq.Workplane("XZ").polyline([(BR_NOSE_U, -BR_T), (SCREW_U + 4.5, -20.0),
                                            (SCREW_U + 12, -20.0), (SCREW_U + 12, -BR_T)]).close()
               .extrude(FIN_HS, both=True))
        # leg: lands on the outer half of the bracket, goes down, then curves out along the arm (~45 deg)
        leg = (cq.Workplane("XZ").polyline([(SCREW_U + 1.5, -BR_T), (SCREW_U + 4.5, -20.0),
                                            (SCREW_U + 4.5 + 0.95 * 70, -90.0), (SCREW_U + 30 + 0.95 * 70, -90.0),
                                            (SCREW_U + 30, -BR_T)]).close()
               .extrude(LEG_HS, both=True))
        return B.U(br, heads, fin, leg)
    return per_corner(one)


def make_collar_v31():
    z0 = B.Z_SPLIT
    outer = B.outline(0.0, z0, 0.0).cut(pockets(POCKET_CLR, z0 + FLOOR_T)).cut(corridor(z0 - 1, z0 + FLOOR_T + 0.01))
    inner = B.outline(B.WALL, z0 - 1, 10.0).cut(pockets(POCKET_CLR + B.WALL, z0 - 2))
    shell = outer.cut(inner)
    return B.make_collar(shell)


def main(out):
    os.makedirs(out, exist_ok=True)
    collar = make_collar_v31()
    tray = B.make_tray(B.make_shell())
    old_collar = B.make_collar(B.make_shell())
    legs = make_legs_measured()
    plate = B.make_plate()
    cq.exporters.export(collar, f"{out}/collar_v3_1.step")
    cq.exporters.export(collar.translate((0, 0, -B.Z_SPLIT)), f"{out}/collar_v3_1.stl", tolerance=0.03, angularTolerance=0.1)

    L = []
    L.append("COLLAR v3.1 fit check (legs = Ethan's caliper numbers + photos)")
    L.append(f"collar v3.1 volume {B.vol(collar)/1000:.1f} cm3 (V3 collar {B.vol(old_collar)/1000:.1f})")
    L.append(f"old V3 collar vs measured legs: {B.inter(old_collar, legs):8.1f} mm3   <- why it didn't go on")
    L.append(f"new collar   vs measured legs:  {B.inter(collar, legs):8.3f} mm3")
    grown = per_corner(lambda: bracket_outline(POCKET_CLR - 0.05, u_far=B.ARM_SCREW_MID + 30).extrude(BR_T + HEAD_H + 0.95)
                       .translate((0, 0, -(BR_T + HEAD_H + 0.95))))
    L.append(f"new collar vs bracket grown by {POCKET_CLR - 0.05:.2f} mm: {B.inter(collar, grown):.3f} mm3  (= the gap is all round)")
    L.append(f"new collar vs plate:            {B.inter(collar, plate):.3f} mm3")
    L.append(f"new collar vs printed tray:     {B.inter(collar, tray):.3f} mm3")
    L.append(f"printed tray vs measured legs:  {B.inter(tray, legs):.3f} mm3")
    # joint: same face as the tray rim? compare the 2 mm slabs either side of the joint
    c_face = collar.intersect(B.box(-200, 200, -200, 200, B.Z_SPLIT, B.Z_SPLIT + 0.5))
    o_face = old_collar.intersect(B.box(-200, 200, -200, 200, B.Z_SPLIT, B.Z_SPLIT + 0.5))
    t_face = tray.intersect(B.box(-200, 200, -200, 200, B.Z_SPLIT - 0.5, B.Z_SPLIT)).translate((0, 0, 0.5))
    L.append(f"joint face: tray rim area {B.vol(t_face)/0.5:.0f} mm2, covered by new collar {B.inter(t_face, c_face)/0.5:.0f} mm2"
             f" (old collar covered {B.inter(t_face, o_face)/0.5:.0f} mm2)")
    for sx in (1, -1):
        for sy in (1, -1):
            hole = B.cyl(2.0, B.Z_SPLIT - 0.1, B.Z_SPLIT + B.COLLAR_TAB_T + 0.1, sx * B.JT_X, sy * B.JT_Y)
            L.append(f"joint screw hole at ({sx*B.JT_X:+.0f}, {sy*B.JT_Y:+.0f}) open: {B.inter(collar, hole) < 0.01}")
    keys = collar.intersect(B.box(-100, 100, -40, 40, 0, B.KEY_H))
    L.append(f"keys {B.vol(keys):.0f} mm3 (V3 {B.vol(old_collar.intersect(B.box(-100, 100, -40, 40, 0, B.KEY_H))):.0f}), clash with plate {B.inter(keys, plate):.3f}")
    open(f"{out}/fit_check_collar_v3_1.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
