# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/landing_leg/v1_CURRENT/src/f450_leg.py
#  Part    : landing_leg
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: edge_u(), profile_pts(), side(), make_leg(), grams(), solve_k()
# ==============================================================================================

# F450 replacement landing leg (Nov 25, 2026) - Ethan DiMeo
# Run:  python f450_leg.py OUTPUT_DIR   (needs cadquery and shapely)
#
# Where each number comes from:
#   bracket 23.2 x 28.5 x 7.3, nose 18.5 in from the screw line, 10.6 flat, chamfer to full
#   width 10.5 in, leg root 14.9 wide, rib from the nose  -> Ethan's calipers + photos (Nov 20,
#   same numbers as collar v3.1, which fits the real legs)
#   screws = the plate's 2 arm screws, 20.5 apart                -> F450 plate model (collar v3.1)
#   height 150 mm (plate to ground), length ~200 mm               -> F450 tall-leg listings
#   leg leaves the bracket, drops ~13 mm, then goes out ~0.95 mm per mm down -> photo trace (v3.1)
#   weight ~20.8 g: frame listed 395 g, measured 312 g without the 4 legs
#   -> (395 - 312) / 4 = 20.75 g per leg (tall-leg listings: 18 - 22.5 g)
#
# Local frame (one leg): u = out along its arm from the frame center, s = across the arm,
# z = up, z = 0 = underside of the bottom plate (where the bracket's top face sits).
import sys, os, math
import cadquery as cq
from shapely.geometry import Polygon, box as sbox

# ---------------------------------------------------------------- PARAMETERS (mm, g)
SCREW_U, SCREW_S = 55.5, 10.25     # the 2 arm screws the bracket hangs on
HOLE_D      = 3.2                  # through holes (M2.5 or M3 frame screws; drill 3.4 if M3 is tight)
BR_NOSE_U   = SCREW_U - 18.5       # nose flat, toward the frame center
BR_NOSE_HS  = 5.3                  # half of the 10.6 nose flat
BR_CORNER_U = SCREW_U - 10.5       # chamfers reach full width here
BR_HS       = 14.25                # half of 28.5
BR_OUT_U    = BR_NOSE_U + 23.2     # outer end of the bracket
BR_T        = 7.3                  # bracket thickness
LEG_HS      = 7.45                 # half of the 14.9 leg width
FIN_HS      = 2.2                  # rib from the nose into the leg
HEIGHT      = 150.0                # plate underside to the ground
DROP        = 20.0                 # leg goes straight down to here first (z = -DROP)
SLOPE       = 0.95                 # then out this many mm per mm down
ROOT_IN_U   = SCREW_U + 1.5        # inner edge where the leg leaves the bracket
TOP_OUT_U   = SCREW_U + 12.0       # outer edge at the bracket level (leg top runs out under the arm;
                                   # the photo trace went to +30 - pulled in to 45 deg so it prints)
DEPTH       = 15.0                 # beam depth (square to the leg), below the drop
FLANGE      = 1.6                  # I-beam faces (inner and outer edge of the leg's side view)
WEB         = 1.8                  # I-beam center web
NECK        = 5.0                  # solid for this far under the bracket
FOOT_CAP    = 4.0                  # solid foot pad at the bottom
BR_SKIN     = 2.0                  # bracket: solid top skin (against the plate)
BR_WALL     = 1.8                  # bracket: side walls around the hollow underside
BOSS_D      = 8.0                  # bracket: solid ring around each screw
FOOT_R      = 6.0                  # round on the outer toe
TARGET_G    = 20.8                 # weight to match (weigh one of your good legs and put it here)
PLA         = 1.24                 # g/cm3
PRINT_FILL  = 0.98                 # a 100 % infill FDM print weighs about 98 % of the solid CAD
MIN_WALL    = 1.2                  # never thinner than this (3 lines of a 0.4 nozzle)


def edge_u(z, outer):
    """u of the inner / outer face of the straight beam at height z (z <= -DROP)"""
    off = DEPTH * math.sqrt(1 + SLOPE * SLOPE) if outer else 0.0
    return (SCREW_U + 4.5) + off + SLOPE * (-DROP - z)


def profile_pts():
    zg = -HEIGHT
    return [(ROOT_IN_U, -BR_T), (SCREW_U + 4.5, -DROP), (edge_u(zg, False), zg),
            (edge_u(zg, True), zg), (edge_u(-DROP, True), -DROP), (TOP_OUT_U, -BR_T)]


def side(pts, hs):
    """polygon in the u-z plane, extruded +/- hs across the arm"""
    return cq.Workplane("XZ").polyline(pts).close().extrude(hs, both=True)


def make_leg(k=1.0):
    """k scales the I-beam faces and web (solve_k picks it so the print weighs TARGET_G)"""
    FLANGE_, WEB_ = max(MIN_WALL, FLANGE * k), max(MIN_WALL, WEB * k)
    foot = [(BR_NOSE_U, -BR_NOSE_HS), (BR_CORNER_U, -BR_HS), (BR_OUT_U, -BR_HS),
            (BR_OUT_U, BR_HS), (BR_CORNER_U, BR_HS), (BR_NOSE_U, BR_NOSE_HS)]
    prism = lambda z0, z1: cq.Workplane("XY").polyline(foot).close().extrude(z1 - z0).translate((0, 0, z0))
    br = prism(-BR_T, 0.0)
    # hollow underside (like a molded part): open at the bottom, BR_SKIN on top, BR_WALL around,
    # a solid boss around each screw
    cav = (cq.Workplane("XY").polyline(foot).close().offset2D(-BR_WALL, kind="intersection")
           .extrude(BR_T - BR_SKIN + 1).translate((0, 0, -BR_T - 1)))
    for s in (-SCREW_S, SCREW_S):
        cav = cav.cut(cq.Workplane("XY").circle(BOSS_D / 2).extrude(BR_T + 2).translate((SCREW_U, s, -BR_T - 1)))
    br = br.cut(cav)
    # rib from the nose into the leg, and the leg itself (side view extruded across the arm)
    fin = side([(BR_NOSE_U, -BR_T), (SCREW_U + 4.5, -DROP), (TOP_OUT_U, -DROP), (TOP_OUT_U, -BR_T)], FIN_HS)
    body = side(profile_pts(), LEG_HS)
    # inside the bracket's hollow, the rib and the leg run up to the top skin (solid joint)
    fill = (side([(BR_NOSE_U, -BR_SKIN), (BR_NOSE_U, -BR_T - 0.01), (BR_OUT_U, -BR_T - 0.01), (BR_OUT_U, -BR_SKIN)], FIN_HS)
            .union(side([(ROOT_IN_U, -BR_SKIN), (ROOT_IN_U, -BR_T - 0.01), (BR_OUT_U, -BR_T - 0.01), (BR_OUT_U, -BR_SKIN)], LEG_HS))
            .intersect(prism(-BR_T - 0.01, -BR_SKIN)))
    # 45 deg gusset off the bracket's outer end: holds up the leg top in print, stays under plate level
    gus = side([(BR_OUT_U - 0.01, 0.0), (BR_OUT_U - 0.01, -BR_T - 0.01), (BR_OUT_U + BR_T, -BR_T - 0.01)], LEG_HS)
    leg = br.union(fin).union(body).union(fill).union(gus)
    # round the outer toe (where it would dig in)
    toe = (cq.Workplane("XZ").polyline([(edge_u(-HEIGHT, True) - FOOT_R * 1.2, -HEIGHT - 1),
                                        (edge_u(-HEIGHT, True) + 1, -HEIGHT - 1),
                                        (edge_u(-HEIGHT, True) + 1, -HEIGHT + FOOT_R * 1.2)]).close()
           .extrude(LEG_HS + 1, both=True))
    leg = leg.cut(toe)
    # I-beam pockets on both sides of the web: the side view shrunk by FLANGE all round, from NECK
    # under the bracket down to FOOT_CAP above the foot; the lower end is square to the leg (prints
    # as a 44 deg ceiling, no support)
    k2 = math.sqrt(1 + SLOPE * SLOPE)
    along = lambda u, z: (u * SLOPE - z) / k2                      # distance along the leg
    end = along(edge_u(-HEIGHT, False), -HEIGHT) - FOOT_CAP
    d = (SLOPE / k2, -1 / k2)
    big = 400
    half = Polygon([(0 + d[0] * end - d[1] * big, d[1] * end + d[0] * big),
                    (0 + d[0] * end + d[1] * big, d[1] * end - d[0] * big),
                    (0 + d[0] * end + d[1] * big - d[0] * big, d[1] * end - d[0] * big - d[1] * big),
                    (0 + d[0] * end - d[1] * big - d[0] * big, d[1] * end + d[0] * big - d[1] * big)])
    shrunk = Polygon(profile_pts()).buffer(-FLANGE_, join_style=2)
    band = shrunk.intersection(sbox(-500, -HEIGHT, 500, -BR_T - NECK)).intersection(half)
    pk = [(round(x, 4), round(y, 4)) for x, y in list(band.exterior.coords)[:-1]]
    for sgn in (1, -1):
        p = side(pk, (LEG_HS - WEB_ / 2) / 2 + 0.5).translate((0, sgn * (WEB_ / 2 + (LEG_HS - WEB_ / 2) / 2 + 0.5), 0))
        leg = leg.cut(p)
    # the 2 screw holes
    for s in (-SCREW_S, SCREW_S):
        leg = leg.cut(cq.Workplane("XY").circle(HOLE_D / 2).extrude(BR_T + 4).translate((SCREW_U, s, -BR_T - 2)))
    return leg


def grams(leg):
    """expected print weight in PLA at 100 % infill"""
    return leg.val().Volume() / 1000.0 * PLA * PRINT_FILL


def solve_k():
    """thickness factor that makes the print weigh TARGET_G (bisection)"""
    lo, hi = 0.5, 2.0
    for _ in range(14):
        mid = (lo + hi) / 2
        if grams(make_leg(mid)) > TARGET_G:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    k = solve_k()
    leg = make_leg(k)
    bb = leg.val().BoundingBox()
    print(f"size  : {bb.xlen:.1f} (along arm) x {bb.ylen:.1f} (across) x {bb.zlen:.1f} (tall) mm")
    print(f"faces {max(MIN_WALL, FLANGE * k):.2f} mm, web {max(MIN_WALL, WEB * k):.2f} mm (k = {k:.3f})")
    print(f"volume {leg.val().Volume() / 1000:.2f} cm3 -> print about {grams(leg):.1f} g PLA at 100 % infill (target {TARGET_G} g)")
    cq.exporters.export(leg, os.path.join(out, "f450_leg.step"))
    # print orientation: bracket face (the face against the plate) down on the bed
    cq.exporters.export(leg.rotate((0, 0, 0), (1, 0, 0), 180), os.path.join(out, "f450_leg_print.stl"),
                        tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(leg, os.path.join(out, "f450_leg_as_mounted.stl"), tolerance=0.02, angularTolerance=0.1)
    print("ok")
