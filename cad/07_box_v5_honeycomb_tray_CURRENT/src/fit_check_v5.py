# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/07_box_v5_honeycomb_tray_CURRENT/src/fit_check_v5.py
#  Part    : 07_box_v5_honeycomb_tray_CURRENT
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: pb_at(), sweep()
# ==============================================================================================

# ELC275 drone - tray V5 fit check + print check (Nov 25, 2026) - Ethan DiMeo
# Run:  python fit_check_v5.py OUTPUT_DIR   (writes fit_check_v5.txt, tray_v5.stl/.step, test_relay_bay_v5.stl)
import sys, os, math
import numpy as np
import cadquery as cq
import trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B, collar_v3_1 as C, tray_v5 as T

box, cyl, U = B.box, B.cyl, B.U
out = sys.argv[1] if len(sys.argv) > 1 else "out"
os.makedirs(out, exist_ok=True)

tray = T.make_tray()
cq.exporters.export(tray, f"{out}/tray_v5.step")
cq.exporters.export(tray.translate((0, 0, -T.ZB)), f"{out}/tray_v5.stl", tolerance=0.03, angularTolerance=0.1)
coupon = tray.intersect(box(-B.LOBE_X - 3, B.LOBE_X + 3, -B.LOBE_Y - T.BACK_PAD_T - 1, -30.0, T.ZS - 56.0, T.ZS))
cq.exporters.export(coupon.translate((0, 0, -(T.ZS - 56.0))), f"{out}/test_relay_bay_v5.stl", tolerance=0.03, angularTolerance=0.1)

collar = C.make_collar_v31()
plate = B.make_plate()
P0, Q = B.make_parts(), T.make_parts()
inter, vol = B.inter, B.vol
L = []
L.append("TRAY V5 (honeycomb) fit check - collar = the printed collar v3.1, landing legs removed")
bb = tray.val().BoundingBox()
L.append(f"tray V5: x {bb.xmin:.1f} .. {bb.xmax:.1f}, y {bb.ymin:.1f} .. {bb.ymax:.1f}, z {bb.zmin:.1f} .. {bb.zmax:.1f}"
         f"  ({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f})")
L.append(f"valid solid: {tray.val().isValid()},  volume {vol(tray)/1000:.1f} cm3 -> about {vol(tray)/1000*1.24:.0f} g of PLA at 100% "
         f"(V4.1: 100.8 cm3)")
L.append(f"floor {-T.ZB - 86.1:.0f} mm lower than V4.1: bottom {-T.ZB:.1f} mm under the plate; relay stays at V4.1 height")

# inside room compared with V4.1
inner_v5 = (T.draft_solid(T.WALL * T.S2, T.ZF).intersect(T.footprint(T.WALL, T.ZF, T.ZS))
            .intersect(box(T.X_L + T.WALL, 200, -200, 200, T.ZF, T.ZS)).union(T.lobes(B.WALL, T.ZF, T.ZS)))
L.append(f"inside volume (walls to roof, joint down to the floor): {vol(inner_v5)/1000:.0f} cm3 (V4.1: 725 cm3, V3: 612 cm3)")
L.append(f"body below the roof: {T.X_R - T.X_L:.0f} x {2*T.Y_W:.0f} mm outside, {T.X_R_IN - T.X_L - T.WALL:.1f} x {2*T.Y_IN:.1f} inside; "
         f"long walls vertical below z {T.Z_ROOF_SIDE:.0f}, right end below z {T.Z_ROOF_END:.0f}")
gap_kit = T.Y_IN - (B.KIT_W / 2 + B.FENCE_GAP + B.FENCE_T)
L.append(f"room beside the kit fence to the long wall: {gap_kit:.1f} mm each side (V4.1: {B.OUT_Y - B.WALL - (B.KIT_W/2 + B.FENCE_GAP + B.FENCE_T):.1f})")
L.append(f"proto board to the right end wall: {T.X_R_IN - T.PB_X1:.1f} mm; free strip in front of it: y {T.PB_Y1:.1f} .. {T.Y_IN:.1f} "
         f"({T.Y_IN - T.PB_Y1:.1f} mm)")

pts = tray.vertices().vals()
L.append(f"farthest tray point from the frame center: {max(math.hypot(v.X, v.Y) for v in pts):.1f} mm (prop discs start at 98.0)")

# honeycomb summary
R = T.HEX_F / math.sqrt(3)
tot = 0
for (pl, c, d0, d1, nm) in T.hex_panels():
    L.append(f"honeycomb {nm:15s}: {len(c):2d} cells")
    tot += len(c)
L.append(f"honeycomb: {tot} hexagons, {T.HEX_F:.0f} mm across the flats ({2*R:.1f} across the corners), ribs {T.HEX_RIB:.1f} mm, "
         f"open area {tot * math.sqrt(3) / 2 * T.HEX_F**2 / 100:.0f} cm2; flat top of each hexagon = {R:.1f} mm bridge")

# joint with the collar
L.append(f"tray vs collar v3.1 overlap: {inter(tray, collar):.3f} mm3")
c_face = collar.intersect(box(-200, 200, -200, 200, T.ZS, T.ZS + 0.5))
t_face = tray.intersect(box(-200, 200, -200, 200, T.ZS - 0.5, T.ZS)).translate((0, 0, 0.5))
L.append(f"joint face: tray rim {vol(t_face)/0.5:.0f} mm2, covered by the collar {inter(t_face, c_face)/0.5:.0f} mm2 "
         f"(V4.1: 1380 / 1296)")
for sx in (1, -1):
    for sy in (1, -1):
        hole = cyl(2.0, T.ZS - 4.0, T.ZS, sx * B.JT_X, sy * B.JT_Y)
        mat = vol(cyl(5.0, T.ZS - 9.0, T.ZS, sx * B.JT_X, sy * B.JT_Y).intersect(tray))
        L.append(f"joint pilot ({sx*B.JT_X:+.0f}, {sy*B.JT_Y:+.0f}): material around it {mat:.0f} mm3, hole clear {inter(tray, hole) < 0.01}")
L.append(f"tray vs bottom plate: {inter(tray, plate):.3f} mm3")

# parts
for nm in ("kit_top", "kit_bot", "jumpers", "plugs"):
    L.append(f"overlap {nm:12s}: {inter(tray, P0[nm]):8.3f} mm3")
for nm in ("pb", "pb_parts", "usb_plug", "relay", "relay_parts", "bullets", "wires"):
    L.append(f"overlap {nm:12s}: {inter(tray, Q[nm]):8.3f} mm3")
ribs = U(*[cyl(2 * B.RIB_R, T.ZF - 0.1, B.Z_FENCE, B.kx(p), sy * (B.KIT_W / 2 + B.FENCE_GAP + B.RIB_R - B.RIB_IN))
           for p in B.RIB_P for sy in (1, -1)])
sweep_kit = U(box(B.KIT_X0, B.KIT_X1, -B.KIT_W / 2, B.KIT_W / 2, B.Z_PCB_BOT, T.ZS + 5),
              box(B.KIT_X0 - 2.5, B.KIT_X0 + 26.0, 2.7, 28.0, B.Z_PCB_TOP + 7.1, T.ZS + 5))
L.append(f"kit drop-in sweep (straight down through the top) vs tray, minus the crush ribs: {inter(tray.cut(ribs), sweep_kit):.3f} mm3")
# proto board goes in BEFORE the kit: lower it through the top opening (shifted left 14, forward 13),
# slide it back 13 and right 14 just above the kit fence, then drop it 3 mm between the corner lips
DX, DY = -14.0, 13.0
zl = B.Z_FENCE + 0.3 - T.Z_RAIL                     # lift: board bottom just above the fence top
PB_PIECES = [Q["pb"]] + [cq.Workplane("XY").add(x) for x in Q["pb_parts"].solids().vals()]
def pb_at(dx, dy, dz):
    w = cq.Workplane("XY")
    for p in PB_PIECES:
        w = w.add(p.translate((dx, dy, dz)).val())
    return w
def sweep(a, b):                                    # swept volume: box hull of each separate piece between 2 positions
    out = None
    for sa, sb in zip(a.solids().vals(), b.solids().vals()):
        bb0, bb1 = sa.BoundingBox(), sb.BoundingBox()
        h = box(min(bb0.xmin, bb1.xmin), max(bb0.xmax, bb1.xmax), min(bb0.ymin, bb1.ymin), max(bb0.ymax, bb1.ymax),
                min(bb0.zmin, bb1.zmin), max(bb0.zmax, bb1.zmax))
        out = h if out is None else out.union(h)
    return out
steps = [("lower through the top", sweep(pb_at(DX, DY, zl), pb_at(DX, DY, T.ZS + 5 - T.Z_RAIL))),
         ("slide back 13", sweep(pb_at(DX, DY, zl), pb_at(DX, 0, zl))),
         ("slide right 14", sweep(pb_at(DX, 0, zl), pb_at(0, 0, zl)))]
res = ", ".join(f"{nm} {inter(tray, sw):.3f}" for nm, sw in steps)
for nm, sw in steps:
    x = tray.intersect(sw)
    if vol(x) > 0.01:
        b2 = x.val().BoundingBox()
        L.append(f"   ({nm} touches the tray at x {b2.xmin:.1f}..{b2.xmax:.1f}, y {b2.ymin:.1f}..{b2.ymax:.1f}, z {b2.zmin:.1f}..{b2.zmax:.1f})")
drop = box(T.PB_X0, T.PB_X1, T.PB_Y0, T.PB_Y1, T.Z_RAIL + 0.01, T.Z_RAIL + zl + T.PB_T)
L.append(f"proto board install path (before the kit) vs tray, mm3: {res}, drop onto the rails {inter(tray, drop):.3f}")
rs = Q["relay_screws"].cut(U(*[T.ybox_cyl(3.0, T.RV_Y_BACK - 7, T.RV_Y_BACK + 0.01, hx, hz) for (hx, hz) in T.RV_HOLES]))
L.append(f"relay screw heads vs tray: {inter(tray, rs):.3f} mm3")
L.append(f"bullets + leads vs collar: {inter(U(Q['bullets'], Q['wires']), collar):.3f} mm3")
L.append(f"relay vs kit + jumpers + plugs: {inter(U(Q['relay'], Q['relay_parts']), U(P0['pcb'], P0['kit_top'], P0['jumpers'], P0['plugs'])):.3f}; "
         f"relay vs proto board: {inter(U(Q['relay'], Q['relay_parts']), U(Q['pb'], Q['pb_parts'])):.3f} mm3")

# flat bottom
bot = tray.intersect(box(-200, 200, -200, 200, T.ZB - 1, T.ZB + 0.2))
top_of_bot = tray.faces("<Z").vals()
zs = sorted(set(round(f.Center().z, 3) for f in top_of_bot))
L.append(f"bottom: every face facing straight down at the bottom is at z = {zs}; lowest point {bb.zmin:.2f} (= {T.ZB}); "
         f"flat landing area {vol(bot)/0.2:.0f} mm2 (strap ears removed)")

# print check on the STL, floor down: faces that point down more than 45 deg and are not on the bed
m = trimesh.load(f"{out}/tray_v5.stl")
n = m.face_normals
c = m.triangles_center
a = m.area_faces
down = n[:, 2] < -math.cos(math.radians(45)) - 1e-3
on_bed = c[:, 2] < 0.05
flat = n[:, 2] < -0.999
L.append(f"STL: {len(m.faces)} triangles, watertight {m.is_watertight}, volume {m.volume/1000:.1f} cm3")
bad = down & ~on_bed
L.append(f"downward faces steeper than 45 deg (not on the bed): {a[bad].sum():.0f} mm2, of which flat (bridges) {a[bad & flat].sum():.0f} mm2")
# group the flat ones by height and size so the bridges can be named
groups = {}
for i in np.where(bad & flat)[0]:
    z = round(c[i, 2], 1)
    groups.setdefault(z, 0.0)
    groups[z] += a[i]
for z in sorted(groups):
    if groups[z] > 2:
        L.append(f"   flat overhang at {z:5.1f} mm above the bed: {groups[z]:6.0f} mm2")
sl = bad & ~flat
if sl.any():
    zz = c[sl, 2]
    L.append(f"   sloped >45 deg: {a[sl].sum():.0f} mm2 between {zz.min():.1f} and {zz.max():.1f} mm above the bed")
L.append(f"test_relay_bay_v5 coupon: {vol(coupon)/1000:.1f} cm3 (back wire bay: relay posts + lead slot)")

open(f"{out}/fit_check_v5.txt", "w").write("\n".join(L) + "\n")
print("\n".join(L))
