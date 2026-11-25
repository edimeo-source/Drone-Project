# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/06_box_v4_1_tray_CURRENT/src/tray_v4_1.py
#  Part    : 06_box_v4_1_tray_CURRENT
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: loft_rect(), ybox_cyl(), make_shell(), make_tray(), make_parts(), main()
# ==============================================================================================

# ELC275 drone - electronics box V4.1: TRAY ONLY (Nov 24, 2026) - Ethan DiMeo
#
# V4.1 = V4 with the battery connection fixed. The battery has separate FEMALE bullets (+ and -),
# and the drone side has two short male-bullet leads:
#   - (-): about 2 - 2.5 in (51 - 64 mm) from the plate's minus pad at the BACK of the frame
#   - (+): about 1.5 in (38 mm) from the relay's COM terminal
# Both leads have to reach outside the box, so the relay must sit right under the battery pads.
#
# What changed from V4:
#   - The relay moves out of the right bay into the BACK wire bay (the lobe right under the plate's
#     back edge opening and its battery pads). It stands on edge on 4 sideways posts on the lobe's
#     back wall, power end (NC, COM, NO) at the TOP, about 25 mm under the plate, control end
#     (CH1, VREF, DC-, DC+) at the bottom.
#   - The back wall of the back lobe gets a U-slot (11 wide, 14 deep, open at the top) right behind
#     the relay's top terminals, between its 2 top posts. Both leads drop in from above and their
#     bullets hang outside at the true back of the drone, where the battery plugs in. (A slot in a side
#     wall would leave only 6 - 8 mm of the 38 mm + lead outside; straight back leaves about 19 mm.)
#   - No XT60 any more, so its window is gone. The relay's floor posts in the right bay are gone.
#   - Everything else is V4: collar joint (collar v3.1 unchanged), right bay size and roof, proto board
#     cradle, USB window, vents, kit cradle, port windows, strap ears.
#
# Run:   python tray_v4_1.py OUTPUT_DIR
# Frame: origin = frame center, +X = true RIGHT, +Y = true FRONT, z = 0 = underside of the plate.

import sys, os, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B          # unchanged V3 (kit cradle numbers, joint, legs, plate)
import collar_v3_1 as C     # unchanged collar v3.1 (for the fit check)

box, cyl, U = B.box, B.cyl, B.U
ZF = B.Z_FLOOR_TOP          # -83.7, top of the floor
ZS = B.Z_SPLIT              # -22, tray / collar joint

# ============================== PARAMETERS ==================================
# right bay (V4, unchanged)
X_END   = 84.5
Y_END   = 42.75
X_BAY0  = 19.0
ROOF_T  = 2.0 * math.sqrt(2)

# 7805 proto board (V4, unchanged): 58.3 x 46.2, USB-A socket toward the back wall
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
ZIP_Y_FRONT = 12.25          # second zip slot row, in front of the proto board
END_VENT_Y = (-35.0, -29.0, -23.0, -17.0)
BACK_VENT_X = (30.0, 35.0, 40.0, 45.0)
VENT_W, VENT_UP = 3.0, (3.6, 19.6)

# relay module RM1HLE, standing on edge in the back lobe
RL_L, RL_W, RL_T = 50.0, 26.0, 1.6
HOLE_LONG, HOLE_SHORT = 44.5, 20.5      # hole pattern: 44.5 along the length, 20.5 across
RV_X0   = -12.0                         # board spans x -12 .. 14 (lobe inside is -17 .. 17)
RV_Z0   = ZF + 9.0                      # bottom edge 9 mm above the floor (room for the control wires)
STANDOFF = 6.0                          # board 6 mm off the lobe's back wall (pins are about 2 mm)
BOSS_D, BOSS_PILOT, BOSS_PILOT_DEPTH = 7.0, 2.6, 8.9
BACK_PAD_D, BACK_PAD_T = 9.0, 2.0       # small pads on the OUTSIDE of the back wall behind each post,
                                        # so an M3 x 10 still stops 1.1 mm short of the outside
RELAY_BODY = (19.2, 15.6, 15.3)
TB_H    = 10.0

# U-slot for the two battery leads in the back lobe's back wall, centered behind the relay
SLOT_W  = 11.0                          # two 10 AWG leads side by side; bullets pass one at a time
SLOT_BOTTOM = ZS - 14.0                 # open from the tray top down to z = -36
BULLET_D = 8.0                          # male bullet + heat shrink, for the clearance check
# ============================================================================

Y_IN   = Y_END - B.WALL
SOCK_FACE_Y = -Y_IN + 0.5
PB_Y0  = SOCK_FACE_Y + SOCK_OUT
PB_Y1  = PB_Y0 + PB_W
PB_X1  = PB_X0 + PB_L
SOCK_X = PB_X1 - SOCK_FROM_END
Z_RAIL = ZF + RAIL_H
Z_PB_TOP = Z_RAIL + PB_T
ZB_OUT = ZS - (X_END - B.OUT_X)
X_IN_TOP = B.OUT_X - ROOF_T
ZB_IN  = ZS - ((X_END - B.WALL) - X_IN_TOP)

LOBE_BACK_IN = -(B.LOBE_Y - B.WALL)     # -54, inner face of the back lobe's back wall
LOBE_SIDE_IN = B.LOBE_X - B.WALL        # 17
RV_X1  = RV_X0 + RL_W
RV_Z1  = RV_Z0 + RL_L
RV_CX, RV_CZ = (RV_X0 + RV_X1) / 2, (RV_Z0 + RV_Z1) / 2
RV_Y_BACK = LOBE_BACK_IN + STANDOFF     # board back face (pins side)
RV_Y_FRONT = RV_Y_BACK + RL_T           # board front face (parts side, toward the box center)
RV_HOLES = [(RV_CX + sx * HOLE_SHORT / 2, RV_CZ + sz * HOLE_LONG / 2) for sx in (1, -1) for sz in (1, -1)]


def loft_rect(r0, z0, r1, z1):
    def wire(r, z):
        x0, x1, y0, y1 = r
        return cq.Wire.makePolygon([cq.Vector(x0, y0, z), cq.Vector(x1, y0, z), cq.Vector(x1, y1, z),
                                    cq.Vector(x0, y1, z)], close=True)
    return cq.Workplane("XY").add(cq.Solid.makeLoft([wire(r0, z0), wire(r1, z1)], True))


def ybox_cyl(d, y0, y1, x, z):
    """cylinder along Y (a sideways post) from y0 to y1, centered at (x, z)"""
    return cq.Workplane("XZ").circle(d / 2).extrude(-(y1 - y0)).translate((x, y0, z))


def make_shell():
    bay_out = U(box(X_BAY0, X_END, -Y_END, Y_END, B.Z_BOTTOM, ZB_OUT),
                loft_rect((X_BAY0, X_END, -Y_END, Y_END), ZB_OUT, (X_BAY0, B.OUT_X, -B.OUT_Y, B.OUT_Y), ZS))
    bay_out = bay_out.edges("|Z").fillet(3.0)
    tabs = U(*[box(B.JT_X0, B.JT_X1, sy * (B.JT_Y - B.JT_HW), sy * (B.JT_Y + B.JT_HW),
                   ZS - (B.JT_X1 - B.OUT_X) - 1, ZS).edges("|Z").fillet(1.5) for sy in (1, -1)])
    outer = B.outline(0.0, B.Z_BOTTOM, ZS).union(bay_out).union(tabs).cut(B.leg_keepout(0.0))
    bay_in = U(box(X_BAY0 - B.WALL, X_END - B.WALL, -Y_IN, Y_IN, ZF, ZB_IN),
               loft_rect((X_BAY0 - B.WALL, X_END - B.WALL, -Y_IN, Y_IN), ZB_IN,
                         (X_BAY0 - B.WALL, X_IN_TOP, -(B.OUT_Y - B.WALL), B.OUT_Y - B.WALL), ZS))
    inner = B.outline(B.WALL, ZF, 10.0).union(bay_in).cut(B.leg_keepout(B.WALL))
    return outer.cut(inner).intersect(box(-200, 200, -200, 200, B.Z_BOTTOM - 1, ZS))


def make_tray():
    t = make_shell()
    kx, ky = B.kx, B.ky

    # 1. PIC32 cradle - V3 unchanged
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

    # 3. proto board cradle (V4): rails, 4 corner lips, zip-tie slots, USB window in the back wall
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
        for zy in ((-Y_IN + PB_Y0) / 2, ZIP_Y_FRONT):
            t = t.cut(box(zx - 2.6, zx + 2.6, zy - 1.0, zy + 1.0, B.Z_BOTTOM - 1, ZF + 1))
    zs0, zs1 = ZF + USB_WIN_Z[0], ZF + USB_WIN_Z[1]
    t = t.cut(box(SOCK_X - USB_WIN_W / 2, SOCK_X + USB_WIN_W / 2, -Y_END - 1, -Y_IN + 0.5, zs0, zs1)
              .edges("|Y").fillet(2.0))

    # 4. relay posts: 4 sideways posts on the back lobe's back wall, each with a 45 deg wedge under it
    #    so it prints without supports; blind M3 pilots (1.1 mm of wall left behind them)
    for (hx, hz) in RV_HOLES:
        post = ybox_cyl(BOSS_D, LOBE_BACK_IN - 1.0, RV_Y_BACK, hx, hz)
        zb = hz - BOSS_D / 2
        L = RV_Y_BACK - LOBE_BACK_IN
        yw = RV_Y_BACK - 2.5                     # wedge stops 2.5 mm short of the board (its pins)
        wedge = (cq.Workplane("YZ").polyline([(LOBE_BACK_IN - 1.0, zb + 0.5), (yw, zb + 0.5),
                                               (LOBE_BACK_IN - 1.0, zb + 0.5 - (yw - LOBE_BACK_IN + 1.0))]).close()
                 .extrude(BOSS_D / 2 - 1.0, both=True).translate((hx, 0, 0)))
        t = t.union(post).union(wedge)
        t = t.cut(ybox_cyl(BOSS_PILOT, RV_Y_BACK - BOSS_PILOT_DEPTH, RV_Y_BACK + 1.0, hx, hz))

    # 5. pads behind the posts (outside), then the U-slot for the battery leads in the back wall
    for (hx, hz) in RV_HOLES:
        t = t.union(ybox_cyl(BACK_PAD_D, -B.LOBE_Y - BACK_PAD_T, -B.LOBE_Y + 0.5, hx, hz))
        t = t.cut(ybox_cyl(BOSS_PILOT, RV_Y_BACK - BOSS_PILOT_DEPTH, RV_Y_BACK + 1.0, hx, hz))
    t = t.cut(box(RV_CX - SLOT_W / 2, RV_CX + SLOT_W / 2, -B.LOBE_Y - BACK_PAD_T - 1, LOBE_BACK_IN + 0.5,
                  SLOT_BOTTOM, ZS + 1.0).edges("|Y and <Z").fillet(4.0))

    # 6. vents (V4): end wall and back wall next to the proto board (the XT60 window is gone)
    vz0, vz1 = ZF + VENT_UP[0], ZF + VENT_UP[1]
    for vy in END_VENT_Y:
        t = t.cut(box(X_END - B.WALL - 0.5, X_END + 1, vy - VENT_W / 2, vy + VENT_W / 2, vz0, vz1))
    for vx in BACK_VENT_X:
        t = t.cut(box(vx - VENT_W / 2, vx + VENT_W / 2, -Y_END - 1, -Y_IN + 0.5, vz0, vz1))

    # 7. battery strap ears: left pair as V3, right pair on the wider walls (V4)
    for sx in (1, -1):
        wall_y = Y_END if sx > 0 else B.OUT_Y
        for sy in (1, -1):
            ex = sx * B.STRAP_X
            ear = box(ex - B.STRAP_W / 2 - 3.5, ex + B.STRAP_W / 2 + 3.5, sy * (wall_y - 0.5), sy * (wall_y + 8),
                      B.Z_BOTTOM, B.Z_BOTTOM + B.FLOOR).edges("|Z").fillet(1.5)
            slot = box(ex - B.STRAP_W / 2 - 1, ex + B.STRAP_W / 2 + 1,
                       sy * (wall_y + 3.2) - 1.75, sy * (wall_y + 3.2) + 1.75, B.Z_BOTTOM - 1, B.Z_BOTTOM + B.FLOOR + 1)
            t = t.union(ear).cut(slot)

    # 8. joint tabs: left pair as V3 (with gussets), right pair are part of the roof; 4 M3 pilots
    for sy in (1, -1):
        tab = box(-B.JT_X1, -B.JT_X0, sy * (B.JT_Y - B.JT_HW), sy * (B.JT_Y + B.JT_HW),
                  ZS - B.TRAY_TAB_T, ZS).edges("|Z").fillet(1.5)
        g = B.gusset_x(-1, sy * B.JT_Y, B.OUT_X - 0.5, ZS - B.TRAY_TAB_T, B.JT_X1 - (B.OUT_X - 0.5), 6.0)
        t = t.union(tab).union(g)
    for sx in (1, -1):
        for sy in (1, -1):
            t = t.cut(cyl(B.JT_PILOT, ZS - B.TRAY_TAB_T - 5.0 + (1.5 if sx > 0 else 0), ZS + 1, sx * B.JT_X, sy * B.JT_Y))

    # 9. text: FRONT on the front lobe (V3), V4.1 on the end wall, + and - next to the slots
    marks = [("FRONT", cq.Plane(origin=(0, B.LOBE_Y, ZS - 13.0), xDir=(-1, 0, 0), normal=(0, 1, 0)), 6.0),
             ("V4.1", cq.Plane(origin=(X_END, -10.0, ZF + 32.0), xDir=(0, 1, 0), normal=(1, 0, 0)), 6.0),
             ("BATT", cq.Plane(origin=(0, -B.LOBE_Y, ZS - 30.0), xDir=(1, 0, 0), normal=(0, -1, 0)), 5.0)]
    for (txt, pl, h) in marks:
        try:
            t = t.cut(cq.Workplane(pl).text(txt, h, -0.6, combine=False, halign="center", valign="center"))
        except Exception as e:
            print("text skipped:", e)
    return t


def make_parts():
    P = {}
    # proto board (V4)
    P["pb"] = box(PB_X0, PB_X1, PB_Y0, PB_Y1, Z_RAIL, Z_PB_TOP)
    brk = box(PB_X1 - BRK_L - 0.9, PB_X1 - 0.9, PB_Y0 - 0.4, PB_Y0 - 0.4 + BRK_W, Z_PB_TOP, Z_PB_TOP + 1.6)
    sock = box(SOCK_X - SOCK_W / 2, SOCK_X + SOCK_W / 2, SOCK_FACE_Y, SOCK_FACE_Y + SOCK_D, Z_PB_TOP + 1.6, Z_PB_TOP + 1.6 + SOCK_H)
    tall = box(PB_X0 + 3.0, PB_X0 + 33.0, PB_Y0 + 3.0, PB_Y1 - 3.0, Z_PB_TOP, Z_PB_TOP + PB_TALL)
    P["pb_parts"] = U(brk, sock, tall)
    zc = Z_PB_TOP + 1.6 + SOCK_H / 2
    P["usb_plug"] = U(box(SOCK_X - 6.0, SOCK_X + 6.0, SOCK_FACE_Y, SOCK_FACE_Y + 12.0, zc - 2.25, zc + 2.25),
                      box(SOCK_X - 8.5, SOCK_X + 8.5, SOCK_FACE_Y - 28.0, SOCK_FACE_Y, zc - 4.5, zc + 4.5),
                      box(SOCK_X - 2.0, SOCK_X + 2.0, SOCK_FACE_Y - 60.0, SOCK_FACE_Y - 28.0, zc - 2.0, zc + 2.0))
    # relay on edge in the back lobe
    yb, yf = RV_Y_BACK, RV_Y_FRONT
    pcb = box(RV_X0, RV_X1, yb, yf, RV_Z0, RV_Z1)
    for (hx, hz) in RV_HOLES:
        pcb = pcb.cut(ybox_cyl(3.1, yb - 1, yf + 1, hx, hz))
    bl, bw, bh = RELAY_BODY
    P["relay"] = pcb
    pins = box(RV_X0 + 1, RV_X1 - 1, yb - 2.0, yb, RV_Z0 + 1, RV_Z1 - 1)
    pins = pins.cut(U(*[ybox_cyl(7.5, yb - 3, yb + 1, hx, hz) for (hx, hz) in RV_HOLES]))
    P["relay_parts"] = U(box(RV_CX - bw / 2, RV_CX + bw / 2, yf, yf + bh, RV_Z1 - 9.0 - bl, RV_Z1 - 9.0),   # relay
                         box(RV_CX - 7.7, RV_CX + 7.7, yf, yf + TB_H, RV_Z1 - 8.0, RV_Z1),              # NC COM NO (top)
                         box(RV_CX - 10.2, RV_CX + 10.2, yf, yf + TB_H, RV_Z0, RV_Z0 + 8.0),            # CH1 VREF DC- DC+
                         pins)
    P["relay_screws"] = U(*[ybox_cyl(3.0, yb - 6.4, yf, hx, hz).union(ybox_cyl(5.5, yf, yf + 2.0, hx, hz))
                            for (hx, hz) in RV_HOLES])
    # leads: NO (10 AWG, ~4.2 mm) straight up to the plate; COM up just over the board's top edge,
    # back over the 2 top posts into the U-slot and out; the bullets hang outside the back wall
    w, yt = 4.2, yf + TB_H / 2
    zc = RV_Z1 + 2.1 + 0.2                              # lead center just above the board edge
    zo = ZS - 2.1 - 0.2                                 # where it crosses the wall (under the collar)
    P["wires"] = U(cyl(w, RV_Z1, ZS, RV_CX + 5.08, yt),                                              # NO
                   box(RV_CX - w / 2, RV_CX + w / 2, yt - w / 2, yt + w / 2, RV_Z1, zc + w / 2),       # COM up
                   box(RV_CX - w / 2, RV_CX + w / 2, RV_Y_BACK, yt + w / 2, zc - w / 2, zc + w / 2),  # over the edge
                   box(RV_CX - w / 2, RV_CX + w / 2, -B.LOBE_Y - 3.0, RV_Y_BACK, zo - w / 2, zo + w / 2))  # dip + out
    P["bullets"] = U(*[cyl(BULLET_D, zo - 28.0, zo + 2.0, RV_CX + dx, -B.LOBE_Y - BACK_PAD_T - BULLET_D / 2 - 1.0)
                       for dx in (-3.0, 3.0)])
    return P


def main(out):
    os.makedirs(out, exist_ok=True)
    tray = make_tray()
    cq.exporters.export(tray, f"{out}/tray_v4_1.step")
    cq.exporters.export(tray.translate((0, 0, -B.Z_BOTTOM)), f"{out}/tray_v4_1.stl", tolerance=0.03, angularTolerance=0.1)
    coupon = tray.intersect(box(-B.LOBE_X - 3, B.LOBE_X + 3, -B.LOBE_Y - BACK_PAD_T - 1, -30.0, ZS - 56.0, ZS))
    cq.exporters.export(coupon.translate((0, 0, -(ZS - 56.0))), f"{out}/test_relay_bay_v4_1.stl", tolerance=0.03, angularTolerance=0.1)

    collar = C.make_collar_v31()
    legs_m, legs_e, plate = C.make_legs_measured(), B.make_legs_estimate(), B.make_plate()
    P0, Q = B.make_parts(), make_parts()
    inter, vol = B.inter, B.vol

    L = []
    L.append("TRAY V4.1 fit check (collar = the printed collar v3.1, legs = caliper-measured model)")
    bb = tray.val().BoundingBox()
    L.append(f"tray V4.1: x {bb.xmin:.1f} .. {bb.xmax:.1f}, y {bb.ymin:.1f} .. {bb.ymax:.1f}, z {bb.zmin:.1f} .. {bb.zmax:.1f}"
             f"  ({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}), volume {vol(tray)/1000:.1f} cm3")
    L.append(f"relay on edge in the back lobe: x {RV_X0:.1f} .. {RV_X1:.1f}, z {RV_Z0:.1f} .. {RV_Z1:.1f} "
             f"({RV_Z1 - ZF:.1f} above the floor, top {-RV_Z1:.1f} mm under the plate), board {STANDOFF:.0f} mm off the back wall")
    L.append("relay posts (x, z): " + ", ".join(f"({hx:.2f}, {hz:.2f})" for (hx, hz) in RV_HOLES))
    L.append(f"lead slot: back wall of the back lobe, x {RV_CX - SLOT_W/2:.1f} .. {RV_CX + SLOT_W/2:.1f}, "
             f"from the tray top (z {ZS:.0f}) down to z {SLOT_BOTTOM:.0f}")
    yt = RV_Y_FRONT + TB_H / 2
    path = 2.3 + (yt - LOBE_BACK_IN) + 1.0 + B.WALL
    L.append(f"COM lead path to the outside of the back wall: about {path:.0f} mm (up {2.3:.1f}, back {yt - LOBE_BACK_IN:.1f}, "
             f"down {1.0:.1f}, wall {B.WALL:.0f}) -> a 38 mm lead leaves about {38.1 - path:.0f} mm outside the box")
    L.append(f"plate underside to the slot: {-ZS:.0f} .. {-SLOT_BOTTOM:.0f} mm (the minus lead is 51 - 64 mm)")
    pts = tray.vertices().vals()
    L.append(f"farthest tray point from the frame center: {max(math.hypot(v.X, v.Y) for v in pts):.1f} mm (prop discs start at 98.0)")
    L.append(f"tray vs collar v3.1 overlap: {inter(tray, collar):.3f} mm3")
    c_face = collar.intersect(box(-200, 200, -200, 200, ZS, ZS + 0.5))
    t_face = tray.intersect(box(-200, 200, -200, 200, ZS - 0.5, ZS)).translate((0, 0, 0.5))
    L.append(f"joint face: rim {vol(t_face)/0.5:.0f} mm2, covered by the collar {inter(t_face, c_face)/0.5:.0f} mm2 (V3: 1321)")
    for sx in (1, -1):
        for sy in (1, -1):
            hole = cyl(2.0, ZS - 4.0, ZS, sx * B.JT_X, sy * B.JT_Y)
            L.append(f"joint pilot ({sx*B.JT_X:+.0f}, {sy*B.JT_Y:+.0f}): material {vol(cyl(5.0, ZS - 4.0, ZS, sx*B.JT_X, sy*B.JT_Y).intersect(tray)):.0f} mm3, hole clear {inter(tray, hole) < 0.01}")
    L.append(f"tray vs measured legs: {inter(tray, legs_m):.3f}   photo-estimate legs: {inter(tray, legs_e):.3f}"
             f"   conservative leg zone: {inter(tray, B.leg_keepout(0.0, clr=0.0)):.3f} mm3")
    L.append(f"tray vs bottom plate: {inter(tray, plate):.3f} mm3")
    for nm in ("kit_top", "kit_bot", "jumpers", "plugs", "bat"):
        L.append(f"overlap {nm:12s}: {inter(tray, P0[nm]):8.3f} mm3")
    for nm in ("pb", "pb_parts", "usb_plug", "relay", "relay_parts", "bullets", "wires"):
        L.append(f"overlap {nm:12s}: {inter(tray, Q[nm]):8.3f} mm3")
    rs = Q["relay_screws"].cut(U(*[ybox_cyl(3.0, RV_Y_BACK - 7, RV_Y_BACK + 0.01, hx, hz) for (hx, hz) in RV_HOLES]))
    L.append(f"relay screw heads vs tray: {inter(tray, rs):.3f} mm3; pilots {BOSS_PILOT_DEPTH:.1f} deep leave "
             f"{(RV_Y_BACK - BOSS_PILOT_DEPTH) - (-B.LOBE_Y - BACK_PAD_T):.1f} mm behind them (M3 x 8 goes {8 - RL_T:.1f} in, M3 x 10 {10 - RL_T:.1f})")
    L.append(f"relay vs kit + jumpers + plugs: {inter(U(Q['relay'], Q['relay_parts']), U(P0['pcb'], P0['kit_top'], P0['jumpers'], P0['plugs'])):.3f} mm3; "
             f"relay vs proto board: {inter(U(Q['relay'], Q['relay_parts']), U(Q['pb'], Q['pb_parts'])):.3f} mm3")
    L.append(f"bullets + leads vs collar: {inter(U(Q['bullets'], Q['wires']), collar):.3f}, vs measured legs: {inter(Q['bullets'], legs_m):.3f} mm3")
    L.append(f"test_relay_bay_v4_1 coupon: {vol(coupon)/1000:.1f} cm3 (back lobe only: relay posts + lead slot)")
    open(f"{out}/fit_check_v4_1.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
