# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/05_box_v4_tray/src/tray_v4.py
#  Part    : 05_box_v4_tray
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: loft_rect(), make_shell_v4(), make_tray_v4(), make_parts_v4(), main()
# ==============================================================================================

# ELC275 drone - electronics box V4: TRAY ONLY (Nov 24, 2026) - Ethan DiMeo
#
# Why V4: the 7805 proto board (58.3 x 46.2 mm, caliper photos) and the relay module (RM1HLE,
# 50 x 26 mm, holes on 44.5 x 20.5) did not both fit in V3's 45 x 58 power bay.
#
# What changed (everything else is box V3):
#   - The right bay is longer (end wall at x = 84.5 instead of 65) and wider (y = +/-42.75 instead of
#     +/-36). The proto board and the relay lie flat on the floor, side by side.
#   - The collar joint at z = -22 is exactly V3's, so the printed COLLAR v3.1 bolts on unchanged.
#     Above the extension the tray closes itself with a 45 deg roof (prints with no supports).
#   - Proto board: 3 mm rails under its short ends, 4 corner lips, 2 zip-tie pairs. Its USB-A socket
#     faces a window in the BACK wall, so the USB cable plugs in from outside and runs to the kit.
#   - Relay: 4 posts (8 mm tall, 2.6 mm pilots) on its 44.5 x 20.5 hole pattern. Control end
#     (CH1, VREF, DC-, DC+) faces J12; power end (NC, COM, NO) faces the end wall and the XT60.
#   - XT60 window (same y and height as V3) and vents move to the new end wall; 4 more vents in the
#     back wall by the 7805. TEST PRINT: test_bay_v4.stl = just the right bay floor (rails, lips, posts).
#
# Run:   python tray_v4.py OUTPUT_DIR
# Frame: same as V3: origin = frame center, +X = true RIGHT, +Y = true FRONT,
#        z = 0 = underside of the F450 bottom plate. Sizes in mm.

import sys, os, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B          # unchanged V3 (kit cradle numbers, joint, legs, plate)
import collar_v3_1 as C     # unchanged collar v3.1 (for the fit check)

box, cyl, U = B.box, B.cyl, B.U
ZF = B.Z_FLOOR_TOP          # -83.7, top of the floor
ZS = B.Z_SPLIT              # -22, tray / collar joint

# ============================== V4 PARAMETERS ===============================
X_END   = 84.5              # outside of the new right end wall (V3: 65)
Y_END   = 42.75             # outside of the right bay's long walls (V3: 36)
X_BAY0  = 19.0              # the wider bay starts at the right edge of the wire bays (lobes)
ROOF_T  = 2.0 * math.sqrt(2)  # 45 deg roof: 2 mm thick measured square to the slope

# 7805 proto board (DFRobot perfboard, cut): caliper photos 58.3 x 46.2, USB-A breakout on a long edge
PB_L, PB_W, PB_T = 58.3, 46.2, 1.6
PB_X0   = 16.5              # left end, 1.9 mm from the kit cradle fence
SOCK_OUT  = 4.8             # USB-A socket sticks out 4.8 past the board edge (photo)
SOCK_FROM_END = 14.4        # socket center from the board's right end (photo)
SOCK_W, SOCK_H, SOCK_D = 13.1, 7.0, 14.0
BRK_L, BRK_W = 26.6, 23.6   # SparkFun USB-A breakout board (photo)
RAIL_H, RAIL_W = 3.0, 3.0   # board sits 3 mm up (clears its solder joints)
LIP_T, LIP_L, LIP_CLR = 1.2, 5.0, 0.4
PB_TALL = 22.0              # tallest part on the board (7805 + heatsink, caps) - envelope for checks
USB_WIN_W, USB_WIN_Z = 20.0, (3.0, 18.0)   # window for a USB-A plug over-mold (17 x 9), z above floor

# relay module RM1HLE (listing + calipers): 50 x 26 PCB, 4 x 3.1 holes on 44.5 x 20.5, Songle relay
RL_L, RL_W, RL_T = 50.0, 26.0, 1.6
RL_X0   = 19.0
HOLE_DX, HOLE_DY = 44.5, 20.5
POST_D, POST_H, POST_PILOT, PILOT_DEPTH = 7.0, 8.0, 2.6, 9.0
RELAY_BODY = (19.2, 15.6, 15.3)     # Songle SRD-05VDC-SL-C
TB_H    = 10.0                      # screw terminal blocks

GAP_PB_RL = 3.0             # proto board <-> relay (room for the zip ties)
ZIP_X   = (24.0, 70.8)      # zip-tie pairs across the proto board
XT60_Y, XT60_UP = -10.0, 27.0       # XT60 window: same y and height as V3 (battery lead reaches it)
END_VENT_Y = (-35.0, -29.0, -23.0, -17.0)
BACK_VENT_X = (30.0, 35.0, 40.0, 45.0)
VENT_W, VENT_UP = 3.0, (3.6, 19.6)
# ============================================================================

Y_IN   = Y_END - B.WALL                   # 40.75 inside half width of the right bay
SOCK_FACE_Y = -Y_IN + 0.5                 # socket face 0.5 mm from the back wall
PB_Y0  = SOCK_FACE_Y + SOCK_OUT           # back edge of the perfboard
PB_Y1  = PB_Y0 + PB_W
PB_X1  = PB_X0 + PB_L
RL_Y0  = PB_Y1 + GAP_PB_RL
RL_Y1  = RL_Y0 + RL_W
RL_X1  = RL_X0 + RL_L
RL_CX, RL_CY = (RL_X0 + RL_X1) / 2, (RL_Y0 + RL_Y1) / 2
SOCK_X = PB_X1 - SOCK_FROM_END
Z_RAIL = ZF + RAIL_H
Z_PB_TOP = Z_RAIL + PB_T
Z_RL_BOT = ZF + POST_H
ZB_OUT = ZS - (X_END - B.OUT_X)           # where the roof meets the end wall (outside)
X_IN_TOP = B.OUT_X - ROOF_T               # inner roof edge at the joint
ZB_IN  = ZS - ((X_END - B.WALL) - X_IN_TOP)
POSTS  = [(RL_CX + sx * HOLE_DX / 2, RL_CY + sy * HOLE_DY / 2) for sx in (1, -1) for sy in (1, -1)]


def loft_rect(r0, z0, r1, z1):
    """solid between rectangle r0 = (x0, x1, y0, y1) at z0 and r1 at z1 (z0 < z1)"""
    def wire(r, z):
        x0, x1, y0, y1 = r
        return cq.Wire.makePolygon([cq.Vector(x0, y0, z), cq.Vector(x1, y0, z), cq.Vector(x1, y1, z),
                                    cq.Vector(x0, y1, z)], close=True)
    return cq.Workplane("XY").add(cq.Solid.makeLoft([wire(r0, z0), wire(r1, z1)], True))


def make_shell_v4():
    # outside: V3 body (main + wire bays) + the bigger right bay with a 45 deg roof up to the joint
    bay_out = U(box(X_BAY0, X_END, -Y_END, Y_END, B.Z_BOTTOM, ZB_OUT),
                loft_rect((X_BAY0, X_END, -Y_END, Y_END), ZB_OUT, (X_BAY0, B.OUT_X, -B.OUT_Y, B.OUT_Y), ZS))
    bay_out = bay_out.edges("|Z").fillet(3.0)
    # right joint tabs (same place and size as V3), filled down to the roof instead of a gusset
    tabs = U(*[box(B.JT_X0, B.JT_X1, sy * (B.JT_Y - B.JT_HW), sy * (B.JT_Y + B.JT_HW),
                   ZS - (B.JT_X1 - B.OUT_X) - 1, ZS).edges("|Z").fillet(1.5) for sy in (1, -1)])
    outer = B.outline(0.0, B.Z_BOTTOM, ZS).union(bay_out).union(tabs).cut(B.leg_keepout(0.0))
    # inside: V3 cavity + the bay cavity (its roof 2 mm thick, square to the 45 deg slope)
    bay_in = U(box(X_BAY0 - B.WALL, X_END - B.WALL, -Y_IN, Y_IN, ZF, ZB_IN),
               loft_rect((X_BAY0 - B.WALL, X_END - B.WALL, -Y_IN, Y_IN), ZB_IN,
                         (X_BAY0 - B.WALL, X_IN_TOP, -(B.OUT_Y - B.WALL), B.OUT_Y - B.WALL), ZS))
    inner = B.outline(B.WALL, ZF, 10.0).union(bay_in).cut(B.leg_keepout(B.WALL))
    return outer.cut(inner).intersect(box(-200, 200, -200, 200, B.Z_BOTTOM - 1, ZS))


def make_tray_v4():
    t = make_shell_v4()
    kx, ky = B.kx, B.ky

    # 1. PIC32 cradle - copied from V3 unchanged
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

    # 2. port windows in the left end wall + floor dip for the micro-USB plug - V3 unchanged
    for (y0, y1, z0, z1) in (B.WIN_A, B.WIN_B):
        t = t.cut(box(-B.OUT_X - 1, -B.OUT_X + B.WALL + 0.5, y0, y1, z0, z1))
    t = t.cut(box(-B.OUT_X + B.WALL - 0.5, B.KIT_X0 - 0.5, B.WIN_A[0], -10.0, B.WIN_A[2], ZF + 0.01))

    # 3. proto board: rails under the short ends, 4 corner lips (locate the PCB edge), zip-tie slots
    rails = U(box(PB_X0, PB_X0 + RAIL_W, PB_Y0, PB_Y1, ZF - 0.1, Z_RAIL),
              box(PB_X1 - RAIL_W, PB_X1, PB_Y0, PB_Y1, ZF - 0.1, Z_RAIL))
    lips = None
    z_lip = Z_PB_TOP - 0.2                 # lips grip the PCB edge only (the breakout sits on top)
    for (cx, dx) in ((PB_X0, 1), (PB_X1, -1)):
        for (cy, dy) in ((PB_Y0, 1), (PB_Y1, -1)):
            ox, oy = cx - dx * LIP_CLR, cy - dy * LIP_CLR
            l1 = box(ox - dx * LIP_T, ox + dx * LIP_L, oy - dy * LIP_T, oy, ZF - 0.1, z_lip)
            l2 = box(ox - dx * LIP_T, ox, oy - dy * LIP_T, oy + dy * LIP_L, ZF - 0.1, z_lip)
            lips = U(*(s for s in (lips, l1, l2) if s is not None))
    t = t.union(rails).union(lips)
    for zx in ZIP_X:
        for zy in ((-Y_IN + PB_Y0) / 2, (PB_Y1 + RL_Y0) / 2):
            t = t.cut(box(zx - 2.6, zx + 2.6, zy - 1.0, zy + 1.0, B.Z_BOTTOM - 1, ZF + 1))
    # USB-A window in the back wall, in front of the socket (the plug goes in from outside)
    zs0, zs1 = ZF + USB_WIN_Z[0], ZF + USB_WIN_Z[1]
    t = t.cut(box(SOCK_X - USB_WIN_W / 2, SOCK_X + USB_WIN_W / 2, -Y_END - 1, -Y_IN + 0.5, zs0, zs1)
              .edges("|Y").fillet(2.0))

    # 4. relay: 4 posts on the module's hole pattern, M3 pilot holes (blind: they stop above the floor bottom)
    for (px, py) in POSTS:
        t = t.union(B.cone(POST_D + 3.0, POST_D, ZF - 0.1, ZF + 1.5, px, py)).union(cyl(POST_D, ZF + 1.0, Z_RL_BOT, px, py))
        t = t.cut(cyl(POST_PILOT, Z_RL_BOT - PILOT_DEPTH, Z_RL_BOT + 1, px, py))

    # 5. new end wall: XT60 window + vents; vents in the back wall next to the proto board
    xz0 = B.Z_BOTTOM + XT60_UP
    t = t.cut(box(X_END - B.WALL - 0.5, X_END + 1, XT60_Y - B.XT60_W / 2, XT60_Y + B.XT60_W / 2, xz0, xz0 + B.XT60_H))
    vz0, vz1 = ZF + VENT_UP[0], ZF + VENT_UP[1]
    for vy in END_VENT_Y:
        t = t.cut(box(X_END - B.WALL - 0.5, X_END + 1, vy - VENT_W / 2, vy + VENT_W / 2, vz0, vz1))
    for vx in BACK_VENT_X:
        t = t.cut(box(vx - VENT_W / 2, vx + VENT_W / 2, -Y_END - 1, -Y_IN + 0.5, vz0, vz1))

    # 6. battery strap ears: left pair as V3, right pair moved out to the wider walls
    for sx in (1, -1):
        wall_y = Y_END if sx > 0 else B.OUT_Y
        for sy in (1, -1):
            ex = sx * B.STRAP_X
            ear = box(ex - B.STRAP_W / 2 - 3.5, ex + B.STRAP_W / 2 + 3.5, sy * (wall_y - 0.5), sy * (wall_y + 8),
                      B.Z_BOTTOM, B.Z_BOTTOM + B.FLOOR).edges("|Z").fillet(1.5)
            slot = box(ex - B.STRAP_W / 2 - 1, ex + B.STRAP_W / 2 + 1,
                       sy * (wall_y + 3.2) - 1.75, sy * (wall_y + 3.2) + 1.75, B.Z_BOTTOM - 1, B.Z_BOTTOM + B.FLOOR + 1)
            t = t.union(ear).cut(slot)

    # 7. joint tabs: left pair exactly as V3 (with gussets); right pair are already part of the roof.
    #    M3 pilots in all 4 (same spots as V3, so collar v3.1's screws line up)
    for sy in (1, -1):
        tab = box(-B.JT_X1, -B.JT_X0, sy * (B.JT_Y - B.JT_HW), sy * (B.JT_Y + B.JT_HW),
                  ZS - B.TRAY_TAB_T, ZS).edges("|Z").fillet(1.5)
        g = B.gusset_x(-1, sy * B.JT_Y, B.OUT_X - 0.5, ZS - B.TRAY_TAB_T, B.JT_X1 - (B.OUT_X - 0.5), 6.0)
        t = t.union(tab).union(g)
    for sx in (1, -1):
        for sy in (1, -1):
            t = t.cut(cyl(B.JT_PILOT, ZS - B.TRAY_TAB_T - 5.0 + (1.5 if sx > 0 else 0), ZS + 1, sx * B.JT_X, sy * B.JT_Y))

    # 8. FRONT on the front wire bay (V3) + V4 on the end wall
    for (txt, pl) in (("FRONT", cq.Plane(origin=(0, B.LOBE_Y, ZS - 13.0), xDir=(-1, 0, 0), normal=(0, 1, 0))),
                      ("V4", cq.Plane(origin=(X_END, -26.0, ZF + 32.0), xDir=(0, 1, 0), normal=(1, 0, 0)))):
        try:
            t = t.cut(cq.Workplane(pl).text(txt, 6.0, -0.6, combine=False, halign="center", valign="center"))
        except Exception as e:
            print("text skipped:", e)
    return t


def make_parts_v4():
    """the new boards (for the fit check and the pictures); the kit etc. come from box_v3.make_parts()"""
    P = {}
    pb = box(PB_X0, PB_X1, PB_Y0, PB_Y1, Z_RAIL, Z_PB_TOP)
    brk = box(PB_X1 - BRK_L - 0.9, PB_X1 - 0.9, PB_Y0 - 0.4, PB_Y0 - 0.4 + BRK_W, Z_PB_TOP, Z_PB_TOP + 1.6)
    sock = box(SOCK_X - SOCK_W / 2, SOCK_X + SOCK_W / 2, SOCK_FACE_Y, SOCK_FACE_Y + SOCK_D, Z_PB_TOP + 1.6, Z_PB_TOP + 1.6 + SOCK_H)
    tall = box(PB_X0 + 3.0, PB_X0 + 33.0, PB_Y0 + 3.0, PB_Y1 - 3.0, Z_PB_TOP, Z_PB_TOP + PB_TALL)   # 7805 + caps zone
    P["pb"] = pb
    P["pb_parts"] = U(brk, sock, tall)
    zc = Z_PB_TOP + 1.6 + SOCK_H / 2
    P["usb_plug"] = U(box(SOCK_X - 6.0, SOCK_X + 6.0, SOCK_FACE_Y, SOCK_FACE_Y + 12.0, zc - 2.25, zc + 2.25),   # metal, all the way in
                      box(SOCK_X - 8.5, SOCK_X + 8.5, SOCK_FACE_Y - 28.0, SOCK_FACE_Y, zc - 4.5, zc + 4.5),      # over-mold
                      box(SOCK_X - 2.0, SOCK_X + 2.0, SOCK_FACE_Y - 60.0, SOCK_FACE_Y - 28.0, zc - 2.0, zc + 2.0))
    zr = Z_RL_BOT + RL_T
    rl = box(RL_X0, RL_X1, RL_Y0, RL_Y1, Z_RL_BOT, zr)
    for (px, py) in POSTS:
        rl = rl.cut(cyl(3.1, Z_RL_BOT - 1, zr + 1, px, py))
    bl, bw, bh = RELAY_BODY
    P["relay"] = rl
    P["relay_parts"] = U(box(RL_X1 - 9.0 - bl, RL_X1 - 9.0, RL_CY - bw / 2, RL_CY + bw / 2, zr, zr + bh),       # relay body
                         box(RL_X0, RL_X0 + 8.0, RL_CY - 10.2, RL_CY + 10.2, zr, zr + TB_H),             # CH1 VREF DC- DC+
                         box(RL_X1 - 8.0, RL_X1, RL_CY - 7.7, RL_CY + 7.7, zr, zr + TB_H),               # NC COM NO
                         box(RL_X0 + 1, RL_X1 - 1, RL_Y0 + 1, RL_Y1 - 1, Z_RL_BOT - 2.0, Z_RL_BOT)       # solder pins,
                         .cut(U(*[cyl(7.5, Z_RL_BOT - 3, Z_RL_BOT + 1, px, py) for (px, py) in POSTS])))  # none at the holes
    P["relay_screws"] = U(*[cyl(3.0, Z_RL_BOT - 6.4, zr, px, py).union(cyl(5.5, zr, zr + 2.0, px, py)) for (px, py) in POSTS])
    xz0 = B.Z_BOTTOM + XT60_UP + 0.4
    P["xt60"] = box(X_END - 12.0, X_END + 4.0, XT60_Y - 7.75, XT60_Y + 7.75, xz0, xz0 + 8.1)
    P["xt60_plug"] = box(X_END + 4.0, X_END + 26.0, XT60_Y - 7.75, XT60_Y + 7.75, xz0, xz0 + 8.1)
    return P


def main(out):
    os.makedirs(out, exist_ok=True)
    tray = make_tray_v4()
    cq.exporters.export(tray, f"{out}/tray_v4.step")
    cq.exporters.export(tray.translate((0, 0, -B.Z_BOTTOM)), f"{out}/tray_v4.stl", tolerance=0.03, angularTolerance=0.1)
    coupon = tray.intersect(box(13.0, X_END + 1, -Y_END - 1, Y_END + 1, B.Z_BOTTOM - 1, ZF + 12.0))
    cq.exporters.export(coupon.translate((0, 0, -B.Z_BOTTOM)), f"{out}/test_bay_v4.stl", tolerance=0.03, angularTolerance=0.1)

    collar = C.make_collar_v31()
    tray3 = B.make_tray(B.make_shell())
    legs_m, legs_e, plate = C.make_legs_measured(), B.make_legs_estimate(), B.make_plate()
    P, Q = B.make_parts(), make_parts_v4()
    inter, vol = B.inter, B.vol

    L = []
    L.append("TRAY V4 fit check (collar = the printed collar v3.1, legs = caliper-measured model)")
    bb = tray.val().BoundingBox()
    L.append(f"tray V4: x {bb.xmin:.1f} .. {bb.xmax:.1f}, y {bb.ymin:.1f} .. {bb.ymax:.1f}, z {bb.zmin:.1f} .. {bb.zmax:.1f}"
             f"  ({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}), volume {vol(tray)/1000:.1f} cm3 (V3 tray {vol(tray3)/1000:.1f})")
    L.append(f"right bay inside: x {B.KIT_X1 + B.FENCE_GAP + B.FENCE_T:.1f} .. {X_END - B.WALL:.1f}, y +/-{Y_IN:.2f};"
             f" roof meets the end wall at z = {ZB_IN:.1f} inside ({ZB_IN - ZF:.1f} above the floor)")
    L.append(f"proto board x {PB_X0:.1f} .. {PB_X1:.1f}, y {PB_Y0:.2f} .. {PB_Y1:.2f} on 3 mm rails; USB socket at x {SOCK_X:.1f}, face {0.5:.1f} mm from the back wall")
    L.append(f"relay x {RL_X0:.1f} .. {RL_X1:.1f}, y {RL_Y0:.2f} .. {RL_Y1:.2f}; posts at " +
             ", ".join(f"({px:.2f}, {py:.2f})" for (px, py) in POSTS) +
             f"; power end has {X_END - B.WALL - RL_X1:.1f} mm to the end wall")
    # farthest point from the center (prop discs start at 98 mm)
    pts = tray.vertices().vals()
    rmax = max(math.hypot(v.X, v.Y) for v in pts)
    L.append(f"farthest tray point from the frame center: {rmax:.1f} mm (prop discs start at 98.0)")
    L.append(f"tray V4 vs collar v3.1 overlap: {inter(tray, collar):.3f} mm3")
    c_face = collar.intersect(box(-200, 200, -200, 200, ZS, ZS + 0.5))
    t_face = tray.intersect(box(-200, 200, -200, 200, ZS - 0.5, ZS)).translate((0, 0, 0.5))
    t3_face = tray3.intersect(box(-200, 200, -200, 200, ZS - 0.5, ZS)).translate((0, 0, 0.5))
    L.append(f"joint face: V4 rim {vol(t_face)/0.5:.0f} mm2 (V3 rim {vol(t3_face)/0.5:.0f}), "
             f"covered by the collar {inter(t_face, c_face)/0.5:.0f} mm2 (V3: {inter(t3_face, c_face)/0.5:.0f})")
    for sx in (1, -1):
        for sy in (1, -1):
            hole = cyl(2.0, ZS - 4.0, ZS, sx * B.JT_X, sy * B.JT_Y)
            L.append(f"joint pilot at ({sx*B.JT_X:+.0f}, {sy*B.JT_Y:+.0f}): material around it in the top 4 mm "
                     f"{vol(cyl(5.0, ZS - 4.0, ZS, sx*B.JT_X, sy*B.JT_Y).intersect(tray)):.0f} mm3, hole clear {inter(tray, hole) < 0.01}")
    L.append(f"tray V4 vs measured legs: {inter(tray, legs_m):.3f}   vs photo-estimate legs: {inter(tray, legs_e):.3f}"
             f"   vs conservative leg zone: {inter(tray, B.leg_keepout(0.0, clr=0.0)):.3f} mm3")
    L.append(f"tray V4 vs bottom plate: {inter(tray, plate):.3f} mm3")
    for nm in ("kit_top", "kit_bot", "jumpers", "plugs", "bat"):
        L.append(f"overlap {nm:12s}: {inter(tray, P[nm]):8.3f} mm3")
    L.append(f"kit board vs tray (crush ribs only, intended squeeze): {inter(tray, P['pcb']):.2f} mm3")
    for nm in ("pb", "pb_parts", "usb_plug", "relay", "relay_parts", "xt60", "xt60_plug"):
        L.append(f"overlap {nm:12s}: {inter(tray, Q[nm]):8.3f} mm3")
    rs = Q["relay_screws"].cut(U(*[cyl(3.0, Z_RL_BOT - 9, Z_RL_BOT + 0.01, px, py) for (px, py) in POSTS]))
    L.append(f"relay screw heads vs tray: {inter(tray, rs):.3f} mm3; pilot depth {PILOT_DEPTH:.1f} leaves "
             f"{Z_RL_BOT - PILOT_DEPTH - B.Z_BOTTOM:.1f} mm of floor under each (an M3 x 10 goes {10 - RL_T:.1f} into a post)")
    L.append(f"proto board vs relay parts: {inter(U(Q['pb'], Q['pb_parts']), U(Q['relay'], Q['relay_parts'])):.3f} mm3; "
             f"kit + jumpers vs new boards: {inter(U(P['kit_top'], P['jumpers'], P['pcb']), U(Q['pb'], Q['pb_parts'], Q['relay'], Q['relay_parts'])):.3f} mm3")
    L.append(f"test_bay_v4 coupon: {vol(coupon)/1000:.1f} cm3 (right bay floor + 12 mm of wall: rails, lips, posts, zip slots)")
    open(f"{out}/fit_check_v4.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
