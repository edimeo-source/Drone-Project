# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/landing_leg/v1_CURRENT/src/print_check.py
#  Part    : landing_leg
#  What    : Printability check of the print STL: watertight, weight, and overhang area that
#            would need support.
# ==============================================================================================

# Printability check of f450_leg_print.stl (bracket face on the bed): downward-facing area flatter
# than 43 deg from horizontal (would need support), not on the bed. Also watertight and weight.
import sys, trimesh, numpy as np
f = sys.argv[1] if len(sys.argv) > 1 else "f450_leg_print.stl"
m = trimesh.load(f)
n, c, a = m.face_normals, m.triangles_center, m.area_faces
bad = (n[:, 2] < -np.sin(np.radians(47))) & (c[:, 2] > 0.05)
print(f"watertight: {m.is_watertight}")
print(f"size: {m.extents[0]:.1f} x {m.extents[1]:.1f} x {m.extents[2]:.1f} mm (x y z on the bed)")
print(f"volume {m.volume / 1000:.2f} cm3 -> about {m.volume / 1000 * 1.24 * 0.98:.1f} g PLA at 100 % infill")
print(f"overhang flatter than 43 deg (needs support): {a[bad].sum():.1f} mm2")
