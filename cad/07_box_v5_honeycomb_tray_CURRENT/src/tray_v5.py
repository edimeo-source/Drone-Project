# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/07_box_v5_honeycomb_tray_CURRENT/src/tray_v5.py
#  Part    : 07_box_v5_honeycomb_tray_CURRENT
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: rim_wire(), draft_solid(), footprint(), lobes(), ybox_cyl(), hex_centers(),
#    hex_cutter(), make_shell(), make_tray(), hex_panels(), make_hex_cutters(), make_parts()
# ==============================================================================================

# ELC275 drone - electronics box V5: HONEYCOMB TRAY (Nov 25, 2026) - Ethan DiMeo
#
# Why V5: the box was too tight, the landing legs are off (the drone now lands on the box), and the
# battery moved to the top of the frame. So:
#   - BIGGER: below the joint the walls flare out at 45 deg (prints with no supports) to a wider,
#     longer body: 151 x 96 mm (V4.1: left part 130 x 72, right bay 169 x 86). Right corners are
#     chamfered so every point stays inside 94 mm of the frame center (the prop discs start at 98).
#   - FLAT BOTTOM: no battery-strap ears under the floor any more. The floor is one flat plane to land on.
#   - HONEYCOMB: thick-ribbed hexagon windows in the walls and in the free parts of the floor. Open for
#     reaching in and routing wires, lighter, faster to print. Flat-top hexagons: the only overhang is the
#     7.5 mm flat top of each hexagon, a short bridge.
#   - SAME JOINT: bolts to the printed COLLAR v3.1 (same rim, same 4 corner tabs, M3 x 10). Reprint the tray only.
#   - KEPT from V4.1: PIC32 kit cradle + port windows (left end), proto board rails / lips / zip slots +
#     USB window, relay on edge in the back wire bay with the battery-lead U-slot, front wire bay.
#
# Run:   python tray_v5.py OUTPUT_DIR
# Frame: origin = frame center, +X = true RIGHT, +Y = true FRONT, z = 0 = underside of the plate.

import sys, os, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B          # kit cradle numbers, joint, plate (unchanged V3)
import collar_v3_1 as C     # the printed collar v3.1 (for the fit check)

box, cyl, U = B.box, B.cyl, B.U

# DEEPER: the floor drops 12 mm (about 1/2 in). The kit cradle, port windows and proto board move down
# with it (all their V3 heights hang off the floor). The relay does NOT move, so the battery leads still
# reach out of the slot.
DEPTH_ADD = 12.0
ZF0 = B.Z_FLOOR_TOP                     # V3 / V4.1 floor top, -83.7
if not getattr(B, "_v5_deepened", False):
    B._v5_deepened = True
    B.Z_BOTTOM -= DEPTH_ADD
    B.Z_FLOOR_TOP -= DEPTH_ADD
    B.Z_PCB_BOT -= DEPTH_ADD
    B.Z_PCB_TOP -= DEPTH_ADD
    B.Z_FENCE -= DEPTH_ADD
    B.WIN_A = (B.WIN_A[0], B.WIN_A[1], B.WIN_A[2] - DEPTH_ADD, B.WIN_A[3] - DEPTH_ADD)
    B.WIN_B = (B.WIN_B[0], B.WIN_B[1], B.WIN_B[2] - DEPTH_ADD, B.WIN_B[3] - DEPTH_ADD)
ZB = B.Z_BOTTOM             # -98.1 bottom of the tray (= the landing surface)
ZF = B.Z_FLOOR_TOP          # -95.7 top of the floor
ZS = B.Z_SPLIT              # -22   tray / collar joint
S2 = math.sqrt(2.0)

# ============================== PARAMETERS ==================================
# body footprint below the 45 deg roof
X_L     = -B.OUT_X          # -65: left end wall stays where it was (port windows), vertical up to the joint
X_R     = 86.0              # right end wall (V4.1: 84.5)
Y_W     = 48.0              # long walls at y = +/-48 (V4.1: 36 left part, 42.75 right bay)
CH_R    = 123.0             # right corners chamfered: x + |y| <= 123 -> farthest point 93.6 from center
CH_L    = 5.0               # small chamfer on the left corners
CORNER_R = 3.0              # fillet on the vertical corner edges
WALL    = 2.4               # body walls and roof (V4.1: 2.0); the wire-bay lobes keep 2.0 (collar rim)
DRAFT   = 30.0              # how far the 45 deg roof loft is built out (just has to be bigger than needed)

# honeycomb: flat-top hexagons, F across the flats, RIB between neighbours
HEX_F   = 11.0
HEX_RIB = 3.0
HEX_EDGE = 3.0              # solid border kept around every honeycomb panel
BAND_BOT = 3.5              # solid band above the floor top on every wall (floor-to-wall joint)

# 7805 proto board (V4 numbers): 58.3 x 46.2, USB-A socket toward the back wall
PB_L, PB_W, PB_T = 58.3, 46.2, 1.6
PB_X0   = 16.5
SOCK_OUT, SOCK_FROM_END = 4.8, 14.4
SOCK_W, SOCK_H, SOCK_D = 13.1, 7.0, 14.0
BRK_L, BRK_W = 26.6, 23.6
RAIL_H, RAIL_W = 3.0, 3.0
LIP_T, LIP_L, LIP_CLR = 1.2, 5.0, 0.4
PB_TALL = 22.0
USB_WIN_W, USB_WIN_Z = 20.0, (3.0, 18.0)
ZIP_X   = (24.0, 70.8)

# relay module RM1HLE on edge in the back wire bay (V4.1, unchanged)
RL_L, RL_W, RL_T = 50.0, 26.0, 1.6
HOLE_LONG, HOLE_SHORT = 44.5, 20.5
RV_X0   = -12.0
RV_Z0   = ZF0 + 9.0                     # same height as V4.1 (relay top 24.7 mm under the plate)
STANDOFF = 6.0
BOSS_D, BOSS_PILOT, BOSS_PILOT_DEPTH = 7.0, 2.6, 8.9
BACK_PAD_D, BACK_PAD_T = 9.0, 2.0
RELAY_BODY = (19.2, 15.6, 15.3)
TB_H    = 10.0
SLOT_W  = 11.0
SLOT_BOTTOM = ZS - 14.0
BULLET_D = 8.0
# ============================================================================

Y_IN    = Y_W - WALL                       # 45.6 inner face of the long walls
X_R_IN  = X_R - WALL                       # 83.6
SOCK_FACE_Y = -Y_IN + 0.5
PB_Y0   = SOCK_FACE_Y + SOCK_OUT
PB_Y1   = PB_Y0 + PB_W
PB_X1   = PB_X0 + PB_L
SOCK_X  = PB_X1 - SOCK_FROM_END
Z_RAIL  = ZF + RAIL_H
Z_PB_TOP = Z_RAIL + PB_T
ZIP_YS  = ((-Y_IN + PB_Y0) / 2, PB_Y1 + 3.0)

LOBE_BACK_IN = -(B.LOBE_Y - B.WALL)        # -54 inner face of the back lobe's back wall
RV_X1   = RV_X0 + RL_W
RV_Z1   = RV_Z0 + RL_L
RV_CX, RV_CZ = (RV_X0 + RV_X1) / 2, (RV_Z0 + RV_Z1) / 2
RV_Y_BACK = LOBE_BACK_IN + STANDOFF
RV_Y_FRONT = RV_Y_BACK + RL_T
RV_HOLES = [(RV_CX + sx * HOLE_SHORT / 2, RV_CZ + sz * HOLE_LONG / 2) for sx in (1, -1) for sz in (1, -1)]

# where the roof meets the walls (outer and inner faces)
Z_ROOF_SIDE  = ZS - (Y_W - B.OUT_Y)                       # -34: long walls vertical below this
Z_ROOF_END   = ZS - (X_R - B.OUT_X)                       # -43: right end wall vertical below this
Z_ROOF_SIDE_IN = ZS - (Y_IN - (B.OUT_Y - WALL * S2))      # inner face
Z_ROOF_END_IN  = ZS - (X_R_IN - (B.OUT_X - WALL * S2))


# ------------------------------------------------------------------ helpers
def rim_wire(shrink, z):
    """V3 main-body rim outline (130 x 72, r3 corners) shrunk by 'shrink', as a wire at height z"""
    w, h = 2 * (B.OUT_X - shrink), 2 * (B.OUT_Y - shrink)
    r = max(3.0 - shrink, 0.6)
    f = cq.Workplane("XY").sketch().rect(w, h).vertices().fillet(r).finalize().extrude(1.0).faces("<Z").wires().val()
    return f.translate(cq.Vector(0, 0, z - f.Center().z))


def draft_solid(shrink, z_low):
    """45 deg roof: everything within (ZS - z) of the rim outline, from the joint down to z_low"""
    w0 = rim_wire(shrink, ZS)
    w1 = w0.offset2D(DRAFT, "arc")[0].translate(cq.Vector(0, 0, -DRAFT))
    loft = cq.Workplane("XY").add(cq.Solid.makeLoft([w0, w1], True))
    below = cq.Workplane("XY").add(cq.Solid.extrudeLinear(cq.Face.makeFromWires(w1), cq.Vector(0, 0, -(ZS - DRAFT - z_low))))
    return loft.union(below)


def footprint(t, z0, z1):
    """the body footprint (chamfered rectangle) shrunk by t, as a prism from z0 to z1"""
    xl, xr, yw = X_L + t, X_R - t, Y_W - t
    cr = CH_R - t * S2
    cl = CH_L + t * (S2 - 1)
    pts = [(xl + cl, -yw), (cr - yw, -yw), (xr, -(cr - xr)), (xr, cr - xr), (cr - yw, yw), (xl + cl, yw),
           (xl, yw - cl), (xl, -(yw - cl))]
    p = cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0))
    return p.edges("|Z").fillet(max(CORNER_R - t, 0.6))


def lobes(t, z0, z1):
    """front / back wire bays (V3 lobes, 38 x 112), shrunk by t"""
    return box(-(B.LOBE_X - t), B.LOBE_X - t, -(B.LOBE_Y - t), B.LOBE_Y - t, z0, z1).edges("|Z").fillet(max(3.0 - t, 0.6))


def ybox_cyl(d, y0, y1, x, z):
    return cq.Workplane("XZ").circle(d / 2).extrude(-(y1 - y0)).translate((x, y0, z))


def hex_centers(u0, u1, v0, v1, keepouts=()):
    """centers of whole flat-top hexagons inside [u0,u1] x [v0,v1], none touching a keep-out (u0,u1,v0,v1).
    Tries a grid of phase shifts and keeps the one that fits the most whole cells."""
    R = HEX_F / math.sqrt(3)                 # corner radius: the hexagon is 2R wide (u) and HEX_F tall (v)
    P = HEX_F + HEX_RIB                      # centre spacing across a flat
    du, dv = P * math.sqrt(3) / 2, P
    best = []
    for a in range(10):
        for b in range(10):
            pu, pv = u0 + R + a * du / 10, v0 + HEX_F / 2 + b * dv / 10
            out = []
            i = -1
            while pu + (i - 1) * du <= u1:
                u = pu + i * du
                off = (i % 2) * dv / 2
                j = -2
                while pv + (j - 1) * dv <= v1:
                    v = pv + j * dv + off
                    j += 1
                    if u - R < u0 - 1e-6 or u + R > u1 + 1e-6 or v - HEX_F / 2 < v0 - 1e-6 or v + HEX_F / 2 > v1 + 1e-6:
                        continue
                    if any(u + R > ka and u - R < kb and v + HEX_F / 2 > kc and v - HEX_F / 2 < kd
                           for (ka, kb, kc, kd) in keepouts):
                        continue
                    out.append((u, v))
                i += 1
            if len(out) > len(best):
                best = out
    return best


def hex_cutter(plane, centers, depth0, depth1):
    """hexagon prisms through a wall. plane 'XZ' = wall normal Y (u=x, v=z), 'YZ' = normal X (u=y, v=z),
    'XY' = floor (u=x, v=y). depth0..depth1 along the normal."""
    if not centers:
        return None
    d = 2 * HEX_F / math.sqrt(3)
    if plane == "XZ":
        wp = cq.Workplane("XZ", origin=(0, depth1, 0))           # extrudes toward -Y
    elif plane == "YZ":
        wp = cq.Workplane("YZ", origin=(depth0, 0, 0))           # extrudes toward +X
    else:
        wp = cq.Workplane("XY", origin=(0, 0, depth0))           # extrudes toward +Z
    return wp.pushPoints(centers).polygon(6, d).extrude(depth1 - depth0)


# ------------------------------------------------------------------ shell
def make_shell():
    outer_body = (draft_solid(0.0, ZB - 1).intersect(footprint(0.0, ZB, ZS + 1))
                  .intersect(box(X_L, 200, -200, 200, ZB, ZS)))
    outer = outer_body.union(lobes(0.0, ZB, ZS))
    inner_body = (draft_solid(WALL * S2, ZF - 1).intersect(footprint(WALL, ZF, ZS + 1))
                  .intersect(box(X_L + WALL, 200, -200, 200, ZF, ZS + 1)))
    top_open = cq.Workplane("XY").add(cq.Solid.extrudeLinear(cq.Face.makeFromWires(rim_wire(WALL * S2, ZS - 0.01)),
                                                             cq.Vector(0, 0, 12)))
    inner = inner_body.union(top_open).union(lobes(B.WALL, ZF, ZS + 10))
    return outer.cut(inner).intersect(box(-200, 200, -200, 200, ZB - 1, ZS))


def make_tray(with_hex=True):
    t = make_shell()
    kx, ky = B.kx, B.ky

    # 1. joint tabs. Left pair: V3 tabs with gussets (outside the left end wall).
    #    Right pair: solid blocks under the roof, 45 deg underside toward the inside (no overhang).
    for sy in (1, -1):
        tab = box(-B.JT_X1, -B.JT_X0, sy * (B.JT_Y - B.JT_HW), sy * (B.JT_Y + B.JT_HW),
                  ZS - B.TRAY_TAB_T, ZS).edges("|Z").fillet(1.5)
        g = B.gusset_x(-1, sy * B.JT_Y, B.OUT_X - 0.5, ZS - B.TRAY_TAB_T, B.JT_X1 - (B.OUT_X - 0.5), 6.0)
        t = t.union(tab).union(g)
        blk = (cq.Workplane("XZ").polyline([(B.JT_X0, ZS), (B.JT_X1 + 2, ZS), (B.JT_X1 + 2, ZS - 16),
                                             (B.JT_X0, ZS - 4.0)]).close()
               .extrude(B.JT_HW, both=True).translate((0, sy * B.JT_Y, 0)))
        env = draft_solid(0.0, ZB).intersect(footprint(0.0, ZB, ZS))
        t = t.union(blk.intersect(env))
    for sx in (1, -1):
        for sy in (1, -1):
            t = t.cut(cyl(B.JT_PILOT, ZS - B.TRAY_TAB_T - 5.0, ZS + 1, sx * B.JT_X, sy * B.JT_Y))

    # 2. honeycomb (before the inside parts go in, so it never cuts them)
    if with_hex:
        t = t.cut(make_hex_cutters())

    # 3. PIC32 cradle - V3 unchanged
    fy0 = B.KIT_W / 2 + B.FENCE_GAP
    fx1 = B.KIT_X1 + B.FENCE_GAP
    fence = U(box(B.KIT_X0, fx1 + B.FENCE_T, fy0, fy0 + B.FENCE_T, ZF - 0.1, B.Z_FENCE),
              box(B.KIT_X0, fx1 + B.FENCE_T, -fy0, -fy0 - B.FENCE_T, ZF - 0.1, B.Z_FENCE),
              box(fx1, fx1 + B.FENCE_T, -fy0 - B.FENCE_T, fy0 + B.FENCE_T, ZF - 0.1, B.Z_FENCE))
    for p in B.RIB_P:
        for sy in (1, -1):
            fence = fence.union(cyl(2 * B.RIB_R, ZF - 0.1, B.Z_FENCE, kx(p), sy * (fy0 + B.RIB_R - B.RIB_IN)))
    boss = U(B.cone(9.0, B.BOSS_D, ZF - 0.1, B.Z_PCB_BOT - 2.0, B.HOLE_X, B.HOLE_Y),
             cyl(B.BOSS_D, B.Z_PCB_BOT - 2.0, B.Z_PCB_BOT, B.HOLE_X, B.HOLE_Y))
    pad = box(kx(B.PAD_P[0]), kx(B.PAD_P[1]), -fy0, ky(B.PAD_Q[1]), ZF - 0.1, B.Z_PCB_BOT)
    j1 = box(kx(B.J1_P[0]), kx(B.J1_P[1]), -B.J1_HALF, B.J1_HALF, ZF - 0.1, B.Z_PCB_BOT - B.J1_H - B.J1_GAP)
    foot = cyl(B.FOOT_D + 3.5, ZF - 0.1, B.Z_PCB_BOT - B.FOOT_H, kx(B.FOOT_P), ky(B.FOOT_Q))
    t = t.union(fence).union(boss).union(pad).union(j1).union(foot)
    t = t.cut(cyl(B.BOSS_PILOT, B.Z_PCB_BOT - 7.0, B.Z_PCB_BOT + 1, B.HOLE_X, B.HOLE_Y))

    # 4. port windows in the left end wall + floor dip for the micro-USB plug (V3 sizes, thicker wall)
    for (y0, y1, z0, z1) in (B.WIN_A, B.WIN_B):
        t = t.cut(box(X_L - 1, X_L + WALL + 0.5, y0, y1, z0, z1))
    t = t.cut(box(X_L + WALL - 0.5, B.KIT_X0 - 0.5, B.WIN_A[0], -10.0, B.WIN_A[2], ZF + 0.01))

    # 5. proto board cradle (V4): rails, 4 corner lips, zip-tie slots, USB window in the back wall
    rails = U(box(PB_X0, PB_X0 + RAIL_W, PB_Y0, PB_Y1, ZF - 0.1, Z_RAIL),
              box(PB_X1 - RAIL_W, PB_X1, PB_Y0, PB_Y1, ZF - 0.1, Z_RAIL))
    lips = None
    z_lip = Z_PB_TOP - 0.2
    for (cx, dx) in ((PB_X0, 1), (PB_X1, -1)):
        for (cy, dy) in ((PB_Y0, 1), (PB_Y1, -1)):
            ox, oy = cx - dx * LIP_CLR, cy - dy * LIP_CLR
            l1 = box(ox - dx * LIP_T, ox + dx * LIP_L, oy - dy * LIP_T, oy, ZF - 0.1, z_lip)
            l2 = box(ox - dx * LIP_T, ox, oy - dy * LIP_T, oy + dy * LIP_L, ZF - 0.1, z_lip)
            lips = U(*(s for s in (lips, l1, l2) if s is not None))
    t = t.union(rails).union(lips)
    for zx in ZIP_X:
        for zy in ZIP_YS:
            t = t.cut(box(zx - 2.6, zx + 2.6, zy - 1.0, zy + 1.0, ZB - 1, ZF + 1))
    zs0, zs1 = ZF + USB_WIN_Z[0], ZF + USB_WIN_Z[1]
    t = t.cut(box(SOCK_X - USB_WIN_W / 2, SOCK_X + USB_WIN_W / 2, -Y_W - 1, -Y_IN + 0.5, zs0, zs1)
              .edges("|Y").fillet(2.0))

    # 6. relay posts on the back bay's back wall (V4.1): 45 deg wedges under them, blind M3 pilots
    for (hx, hz) in RV_HOLES:
        post = ybox_cyl(BOSS_D, LOBE_BACK_IN - 1.0, RV_Y_BACK, hx, hz)
        zb = hz - BOSS_D / 2
        yw = RV_Y_BACK - 2.5
        wedge = (cq.Workplane("YZ").polyline([(LOBE_BACK_IN - 1.0, zb + 0.5), (yw, zb + 0.5),
                                               (LOBE_BACK_IN - 1.0, zb + 0.5 - (yw - LOBE_BACK_IN + 1.0))]).close()
                 .extrude(BOSS_D / 2 - 1.0, both=True).translate((hx, 0, 0)))
        t = t.union(post).union(wedge)
    for (hx, hz) in RV_HOLES:
        t = t.union(ybox_cyl(BACK_PAD_D, -B.LOBE_Y - BACK_PAD_T, -B.LOBE_Y + 0.5, hx, hz))
        t = t.cut(ybox_cyl(BOSS_PILOT, RV_Y_BACK - BOSS_PILOT_DEPTH, RV_Y_BACK + 1.0, hx, hz))
    # 7. U-slot for the battery leads in the back bay's back wall (V4.1)
    t = t.cut(box(RV_CX - SLOT_W / 2, RV_CX + SLOT_W / 2, -B.LOBE_Y - BACK_PAD_T - 1, LOBE_BACK_IN + 0.5,
                  SLOT_BOTTOM, ZS + 1.0).edges("|Y and <Z").fillet(4.0))

    # 8. engraving: FRONT on the front bay, V5 on the left end wall, BATT under the lead slot
    marks = [("FRONT", cq.Plane(origin=(0, B.LOBE_Y, ZS - 13.0), xDir=(-1, 0, 0), normal=(0, 1, 0)), 6.0),
             ("V5", cq.Plane(origin=(X_L, 0.0, ZS - 7.5), xDir=(0, -1, 0), normal=(-1, 0, 0)), 6.0),
             ("BATT", cq.Plane(origin=(0, -B.LOBE_Y, ZS - 30.0), xDir=(1, 0, 0), normal=(0, -1, 0)), 5.0)]
    for (txt, pl, h) in marks:
        try:
            t = t.cut(cq.Workplane(pl).text(txt, h, -0.6, combine=False, halign="center", valign="center"))
        except Exception as e:
            print("text skipped:", e)
    return t


# ------------------------------------------------------------------ honeycomb panels
def hex_panels():
    """(plane, centers, depth0, depth1, name) for every honeycomb panel"""
    E = HEX_EDGE
    zlo = ZF + BAND_BOT
    lobe_u = B.LOBE_X + E                      # stay off the wire-bay walls
    P = []
    # long walls, vertical part
    side_z1 = min(Z_ROOF_SIDE, Z_ROOF_SIDE_IN) - E
    xl, xr = X_L + CH_L + E, (CH_R - Y_W) - E
    usb_keep = (SOCK_X - USB_WIN_W / 2 - E, SOCK_X + USB_WIN_W / 2 + E, ZB, ZF + USB_WIN_Z[1] + E)
    for sy, nm in ((1, "front wall"), (-1, "back wall")):
        keep = [usb_keep] if sy < 0 else []
        c = hex_centers(xl, -lobe_u, zlo, side_z1, keep) + hex_centers(lobe_u, xr, zlo, side_z1, keep)
        P.append(("XZ", c, sy * Y_W - 1.0 if sy > 0 else -Y_W - 1.0, sy * Y_W + 1.0 if sy > 0 else -Y_IN + 1.0, nm))
    # fix depths: through the wall only
    P[0] = ("XZ", P[0][1], Y_IN - 1.0, Y_W + 1.0, "front wall")
    P[1] = ("XZ", P[1][1], -Y_W - 1.0, -Y_IN + 1.0, "back wall")
    # right end wall
    end_z1 = min(Z_ROOF_END, Z_ROOF_END_IN) - E
    yc = (CH_R - X_R) - E
    P.append(("YZ", hex_centers(-yc, yc, zlo, end_z1), X_R_IN - 1.0, X_R + 1.0, "right end wall"))
    # left end wall, above the port windows, between the joint-tab gussets
    win_top = max(B.WIN_A[3], B.WIN_B[3]) + E + 1.0
    P.append(("YZ", hex_centers(-20.0, 20.0, win_top, ZS - 13.0), X_L - 1.0, X_L + WALL + 1.0, "left end wall"))
    # floor: right bay (around the proto board rails, lips and zip slots) and both sides of the kit
    fk = []
    for xr0 in (PB_X0, PB_X1 - RAIL_W):
        fk.append((xr0 - E, xr0 + RAIL_W + E, PB_Y0 - E, PB_Y1 + E))
    for cx in (PB_X0, PB_X1):
        for cy in (PB_Y0, PB_Y1):
            fk.append((cx - LIP_L - LIP_T - E, cx + LIP_L + LIP_T + E, cy - LIP_L - LIP_T - E, cy + LIP_L + LIP_T + E))
    for zx in ZIP_X:
        for zy in ZIP_YS:
            fk.append((zx - 2.6 - E, zx + 2.6 + E, zy - 1.0 - E, zy + 1.0 + E))
    fence_x1 = B.KIT_X1 + B.FENCE_GAP + B.FENCE_T
    fy = B.KIT_W / 2 + B.FENCE_GAP + B.FENCE_T
    floor = hex_centers(fence_x1 + E, X_R_IN - E, -Y_IN + E, Y_IN - E, fk)
    # keep the chamfered corners solid
    R = HEX_F / math.sqrt(3)
    floor = [(u, v) for (u, v) in floor if u + R + abs(v) + HEX_F / 2 <= CH_R - WALL * S2 - E * S2]
    for sy in (1, -1):
        lo, hi = sorted((sy * (fy + E), sy * (Y_IN - E)))
        side = hex_centers(X_L + WALL + E, fence_x1 + E, lo, hi)
        side = [(u, v) for (u, v) in side if abs(u) > B.LOBE_X + E or abs(v) < B.LOBE_Y]
        floor += side
    # stay clear of the wire bays' floors (they hold leads and the relay)
    floor = [(u, v) for (u, v) in floor if not (abs(u) - R < B.LOBE_X + E and abs(v) + HEX_F / 2 > Y_IN - E)]
    P.append(("XY", floor, ZB - 1.0, ZF + 1.0, "floor"))
    return P


def make_hex_cutters():
    cut = None
    for (pl, c, d0, d1, nm) in hex_panels():
        h = hex_cutter(pl, c, d0, d1)
        if h is not None:
            cut = h if cut is None else cut.union(h)
    return cut


# ------------------------------------------------------------------ parts (for the fit check and pictures)
def make_parts():
    P = {}
    P["pb"] = box(PB_X0, PB_X1, PB_Y0, PB_Y1, Z_RAIL, Z_PB_TOP)
    brk = box(PB_X1 - BRK_L - 0.9, PB_X1 - 0.9, PB_Y0 - 0.4, PB_Y0 - 0.4 + BRK_W, Z_PB_TOP, Z_PB_TOP + 1.6)
    sock = box(SOCK_X - SOCK_W / 2, SOCK_X + SOCK_W / 2, SOCK_FACE_Y, SOCK_FACE_Y + SOCK_D, Z_PB_TOP + 1.6, Z_PB_TOP + 1.6 + SOCK_H)
    tall = box(PB_X0 + 3.0, PB_X0 + 33.0, PB_Y0 + 3.0, PB_Y1 - 3.0, Z_PB_TOP, Z_PB_TOP + PB_TALL)
    P["pb_parts"] = U(brk, sock, tall)
    zc = Z_PB_TOP + 1.6 + SOCK_H / 2
    P["usb_plug"] = U(box(SOCK_X - 6.0, SOCK_X + 6.0, SOCK_FACE_Y, SOCK_FACE_Y + 12.0, zc - 2.25, zc + 2.25),
                      box(SOCK_X - 8.5, SOCK_X + 8.5, SOCK_FACE_Y - 28.0, SOCK_FACE_Y, zc - 4.5, zc + 4.5),
                      box(SOCK_X - 2.0, SOCK_X + 2.0, SOCK_FACE_Y - 60.0, SOCK_FACE_Y - 28.0, zc - 2.0, zc + 2.0))
    yb, yf = RV_Y_BACK, RV_Y_FRONT
    pcb = box(RV_X0, RV_X1, yb, yf, RV_Z0, RV_Z1)
    for (hx, hz) in RV_HOLES:
        pcb = pcb.cut(ybox_cyl(3.1, yb - 1, yf + 1, hx, hz))
    bl, bw, bh = RELAY_BODY
    P["relay"] = pcb
    pins = box(RV_X0 + 1, RV_X1 - 1, yb - 2.0, yb, RV_Z0 + 1, RV_Z1 - 1)
    pins = pins.cut(U(*[ybox_cyl(7.5, yb - 3, yb + 1, hx, hz) for (hx, hz) in RV_HOLES]))
    P["relay_parts"] = U(box(RV_CX - bw / 2, RV_CX + bw / 2, yf, yf + bh, RV_Z1 - 9.0 - bl, RV_Z1 - 9.0),
                         box(RV_CX - 7.7, RV_CX + 7.7, yf, yf + TB_H, RV_Z1 - 8.0, RV_Z1),
                         box(RV_CX - 10.2, RV_CX + 10.2, yf, yf + TB_H, RV_Z0, RV_Z0 + 8.0),
                         pins)
    P["relay_screws"] = U(*[ybox_cyl(3.0, yb - 6.4, yf, hx, hz).union(ybox_cyl(5.5, yf, yf + 2.0, hx, hz))
                            for (hx, hz) in RV_HOLES])
    w, yt = 4.2, yf + TB_H / 2
    zc = RV_Z1 + 2.1 + 0.2
    zo = ZS - 2.1 - 0.2
    P["wires"] = U(cyl(w, RV_Z1, ZS, RV_CX + 5.08, yt),
                   box(RV_CX - w / 2, RV_CX + w / 2, yt - w / 2, yt + w / 2, RV_Z1, zc + w / 2),
                   box(RV_CX - w / 2, RV_CX + w / 2, RV_Y_BACK, yt + w / 2, zc - w / 2, zc + w / 2),
                   box(RV_CX - w / 2, RV_CX + w / 2, -B.LOBE_Y - 3.0, RV_Y_BACK, zo - w / 2, zo + w / 2))
    P["bullets"] = U(*[cyl(BULLET_D, zo - 28.0, zo + 2.0, RV_CX + dx, -B.LOBE_Y - BACK_PAD_T - BULLET_D / 2 - 1.0)
                       for dx in (-3.0, 3.0)])
    # battery on TOP of the frame now (Nov 25), only for the pictures
    P["bat_top"] = box(-B.BAT_L / 2, B.BAT_L / 2, -B.BAT_W / 2, B.BAT_W / 2, B.PLATE_T + 2.0, B.PLATE_T + 2.0 + B.BAT_H)
    return P


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "out"
    os.makedirs(out, exist_ok=True)
    tr = make_tray()
    cq.exporters.export(tr, f"{out}/tray_v5.step")
    cq.exporters.export(tr.translate((0, 0, -ZB)), f"{out}/tray_v5.stl", tolerance=0.03, angularTolerance=0.1)
    print("ok", tr.val().isValid(), tr.val().Volume() / 1000)
