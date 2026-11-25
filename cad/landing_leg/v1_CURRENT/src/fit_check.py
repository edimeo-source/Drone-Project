# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/landing_leg/v1_CURRENT/src/fit_check.py
#  Part    : landing_leg
#  What    : Fit check of the part against the frame and the printed box (prints overlap
#            volumes).
#  FUNCTIONS: at()
# ==============================================================================================

# Fit check: the new leg at all 4 corners against collar v3.1, tray v4.1, the plate and the
# measured legs that collar v3.1 was built around.  Run:  python fit_check.py OUTPUT_DIR
import sys, os, math
import cadquery as cq
HERE = os.path.dirname(os.path.abspath(__file__))
OUTD = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
CAD = next(c for c in (os.path.join(HERE, "../../../06_box_v4_1_tray_CURRENT/src"),
                       "/home/claude/Drone-Project/cad/06_box_v4_1_tray_CURRENT/src") if os.path.isdir(c))
sys.path.insert(0, CAD)
os.chdir(CAD)
import box_v3 as B, collar_v3_1 as C
sys.path.insert(0, HERE)
import f450_leg as L

leg1 = L.make_leg(L.solve_k())
def at(sx, sy, part):
    return part.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(sy, sx)))
legs = [at(sx, sy, leg1) for sx in (1, -1) for sy in (1, -1)]
collar = C.make_collar_v31()
plate = B.make_plate()
import importlib; T = importlib.import_module("tray_v4_1")
tray = T.make_tray()
measured = C.make_legs_measured()
lines = ["NEW F450 LEG fit check (4 corners)"]
for name, part in (("collar v3.1", collar), ("plate", plate), ("tray v4.1", tray)):
    if part is None:
        lines.append(f"{name}: not built"); continue
    v = sum(B.inter(l, part) for l in legs)
    lines.append(f"new legs vs {name:12s}: {v:8.3f} mm3")
# how close is it to the leg shape the collar was traced around (top 20 mm: bracket + root)
top = cq.Workplane("XY").box(400, 400, 20.1).translate((0, 0, -10.05))
m_top = measured.intersect(top)
n_top = legs[0].union(legs[1]).union(legs[2]).union(legs[3]).intersect(top)
lines.append(f"top 20 mm: measured {m_top.val().Volume():.0f} mm3, new {n_top.val().Volume():.0f} mm3, "
             f"common {B.inter(m_top, n_top):.0f} mm3")
bb = leg1.val().BoundingBox()
lines.append(f"lowest point of the leg: z = {bb.zmin:.1f} mm; lowest box point (tray v4.1 / battery): z = -86.1 / -112.6")
print("\n".join(lines))
open(os.path.join(OUTD, "fit_check.txt"), "w").write("\n".join(lines) + "\n")
