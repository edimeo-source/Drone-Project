# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/05_box_v4_tray/src/box_v3.py
#  Part    : 05_box_v4_tray
#  What    : CadQuery model source (every size is in the PARAMETERS block at the top). Run it to
#            re-make the STL/STEP files.
#  FUNCTIONS: kx(), ky(), box(), cyl(), cone(), U(), outline(), leg_keepout(),
#    make_legs_estimate(), make_shell(), gusset_x(), make_tray(), make_collar(), make_plate(),
#    make_parts(), vol(), inter(), main()
# ==============================================================================================

# ELC275 drone - electronics box V3 (CadQuery 2.x)
# Ethan DiMeo
#
# Two printed parts, bolted together with 4x M3 x 10:
#   TRAY   - the PIC32MZ EF Starter Kit lies RIGHT SIDE UP, flat, in a snug cradle with a screw boss
#            (J1, the rubber foot and J4 on its underside drop into a 5 mm relief). Port windows at the
#            left end for USB-A J5, both mini-USBs (J3 debug, J11 UART) and micro-USB J4. Power / kill
#            board bay at the right end. Battery strapped under the floor (same as V2).
#   COLLAR - the "slotting bracket": keys + 4x M3 into the frame's 4 tab slots (same as V2), but its
#            walls wrap around the 4 landing-gear legs (skinny at the top, flaring out lower down),
#            so the box no longer hits the legs.
# Both parts have the front / back wire bays (lobes) right under the plate's front and back edge
# openings, so the ESC servo leads drop straight into the box instead of hanging outside.
#
# Run:   python box_v3.py OUTPUT_DIR
# Frame: origin = frame center, +X = true RIGHT (red arms), +Y = true FRONT, +Z = up,
#        z = 0 = UNDERSIDE of the F450 bottom plate (the box hangs under it). Sizes in mm.

import sys, json, os, math
import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))

# ============================== PARAMETERS ==================================
PLATE_T        = 1.6
SLOT_X, SLOT_Y = 70.1, 17.0          # frame tab-slot centers (same as V2, they fit)
TAB_HALF_W     = 26.0

# ---- outline (outside of the walls) -------------------------------------------------------
OUT_X, OUT_Y   = 65.0, 36.0          # main body half sizes (same as V2: 130 x 72)
LOBE_X, LOBE_Y = 19.0, 56.0          # front / back wire bays under the plate's 32 x 12 edge openings
WALL, FLOOR    = 2.0, 2.4
EXTRA_DEPTH    = 38.1                # Nov 19: 1.5 in deeper than V2 for more room inside
Z_BOTTOM       = -48.0 - EXTRA_DEPTH # -86.1: bottom 86.1 under the plate, battery bottom 112.6
Z_FLOOR_TOP    = Z_BOTTOM + FLOOR
Z_SPLIT        = -22.0               # tray / collar joint

# ---- landing-gear legs (traced from the photos - see README "measure") ---------------------
# Each leg's top bracket bolts under the bottom plate on its arm's 2 screws, then the leg curves
# down and OUT along the arm. The box keeps LEG_CLR away from this (conservative) shape.
ARM_SCREW_MID  = 55.5                # center -> midpoint of an arm's 2 screws, along the arm (39.25, 39.25)
LEG_FLANGE_L   = 36.0                # bracket length along the line of the 2 screws
LEG_FLANGE_W   = 18.0                # bracket width along the arm
LEG_FLANGE_T   = 9.0                 # bracket + screw nuts below the plate
LEG_SLOPE      = 1.0                 # leg moves out 1 mm per mm down (photos say ~1.4: more room)
LEG_BODY_HW    = 9.0                 # half width of the leg (photos: ~6)
LEG_CLR        = 1.5

# ---- PIC32MZ EF Starter Kit, right side up ------------------------------------------------
KIT_L, KIT_W, KIT_T = 71.5, 51.0, 1.6
KIT_X0         = -59.0               # port edge (USB / RJ45 end) faces the LEFT end wall
BED_H          = 5.0                 # board underside above the floor (J1 3.5, rubber foot 3.4, J4 3.0)
HOLE_FROM_PORT_EDGE = 19.2
HOLE_Y         = -0.3                # mounting hole (V2 post matched it with the board flipped)
FENCE_GAP, FENCE_T = 0.5, 1.6        # cradle fence: 0.5 gap per side ...
RIB_R, RIB_IN  = 0.8, 0.65           # ... closed up by crush ribs (snug for a 50.5 - 51.2 wide board)
RIB_P          = (10.0, 34.0, 50.0)  # rib positions, mm from the port edge
FENCE_ABOVE    = 1.0                 # fence top above the board top
BOSS_D, BOSS_PILOT = 4.6, 2.2        # M2.5 x 8 self-tapping (board pad ring is 4.3, parts 2.9 mm away)
J1_H           = 3.5                 # Hirose FX10A-168P-SV1 header height (5 mm stack)
J1_P           = (58.3, 62.8)        # J1 body, mm from the port edge (body 57.6 - 63.5)
J1_HALF        = 23.0                # pedestal half length (J1 is 49.1 long)
J1_GAP         = 0.15                # J1 just clears its pedestal; it lands on it when you push on J12
FOOT_D, FOOT_H = 9.5, 3.43           # 3M SJ61A8 bumpon (rubber foot)
FOOT_P, FOOT_Q = 5.0, 40.5           # its center: from the port edge, from the J5 (-Y) edge
PAD_P, PAD_Q   = (0.5, 5.0), (0.0, 2.6)   # rigid corner pad, port end, J5 side

# ports on the port edge (kit y, right side up), measured from the photos
USBA_Y, MINI1_Y, MINI2_Y, MICRO_Y = -16.0, -3.4, 13.1, -18.0
# windows: y0, y1, z0, z1 (z from the board underside ZB / top ZT, set below)
WIN_A_Y, WIN_B_Y = (-25.5, 3.4), (6.3, 20.5)   # A: USB-A J5 + mini-USB + micro-USB J4, B: second mini-USB

# ---- power / kill board bay (right end) ---------------------------------------------------
PWR_L, PWR_W, PWR_H = 45.0, 58.0, 22.0
PWR_X0         = 15.5
ZIP_X, ZIP_Y   = (24.0, 52.0), 31.0
XT60_W, XT60_H, XT60_Y, XT60_UP = 16.6, 9.0, -10.0, 27.0   # XT60 window 27 mm above the box bottom (as V2)
VENT_Y, VENT_W, VENT_UP = (6.0, 11.0, 16.0, 21.0), 3.0, (3.6, 19.6)   # vents, mm above the floor

# ---- frame mount (collar top, unchanged from V2) ------------------------------------------
EAR_X1, EAR_T  = 86.0, 4.0
KEY_X0, KEY_X1, KEY_W, KEY_H = 63.0, 73.0, 3.0, 1.4
BOLT_X, BOLT_D = 79.0, 3.4
NUT_AF, NUT_DEPTH = 5.7, 2.6
GUS_Y_R        = (-22.0, 0.0, 22.0)  # gussets under the right end plate
GUS_Y_L        = (-22.0, 0.0, 22.0)

# ---- tray <-> collar joint: 4 tabs at the end-wall corners, M3 x 10 down into the tray ------
JT_X, JT_Y, JT_HW = 69.0, 30.0, 6.0
JT_X0, JT_X1   = 64.0, 74.0
TRAY_TAB_T, COLLAR_TAB_T = 4.0, 3.0
JT_PILOT, JT_CLEAR = 2.6, 3.4

# ---- battery + straps ----------------------------------------------------------------------
BAT_L, BAT_W, BAT_H = 75.0, 34.0, 26.5
STRAP_W, STRAP_X = 20.0, 30.5
# ============================================================================

Z_PCB_BOT = Z_FLOOR_TOP + BED_H
Z_PCB_TOP = Z_PCB_BOT + KIT_T
KIT_X1    = KIT_X0 + KIT_L
HOLE_X    = KIT_X0 + HOLE_FROM_PORT_EDGE
Z_FENCE   = Z_PCB_TOP + FENCE_ABOVE
WIN_A     = (*WIN_A_Y, Z_FLOOR_TOP - 1.0, Z_PCB_TOP + 9.0)
WIN_B     = (*WIN_B_Y, Z_PCB_BOT - 1.8, Z_PCB_TOP + 9.0)
XT60_Z0   = Z_BOTTOM + XT60_UP
VENT_Z    = (Z_FLOOR_TOP + VENT_UP[0], Z_FLOOR_TOP + VENT_UP[1])
SQ2       = math.sqrt(2.0)


def kx(p): return KIT_X0 + p
def ky(q): return -KIT_W / 2 + q


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl(d, z0, z1, x, y):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def cone(d0, d1, z0, z1, x, y):
    return (cq.Workplane("XY").circle(d0 / 2).workplane(offset=z1 - z0).circle(d1 / 2).loft()
            .translate((x, y, z0)))


def U(*shapes):
    out = None
    for s in shapes:
        out = s if out is None else out.union(s)
    return out


# ------------------------------------------------------------------ outline and leg keep-out
def outline(t, z0, z1):
    """the box outline (main body + front/back wire bays), shrunk by t"""
    r = max(3.0 - t, 0.6)
    a = box(-(OUT_X - t), OUT_X - t, -(OUT_Y - t), OUT_Y - t, z0, z1).edges("|Z").fillet(r)
    b = box(-(LOBE_X - t), LOBE_X - t, -(LOBE_Y - t), LOBE_Y - t, z0, z1).edges("|Z").fillet(r)
    return a.union(b)


def leg_keepout(g, clr=LEG_CLR):
    """space the 4 legs can be in, grown by clr + g (g = WALL for the inside surface)"""
    m = clr + g
    S = LEG_SLOPE
    u_f = ARM_SCREW_MID - LEG_FLANGE_W / 2 - m
    u0 = ARM_SCREW_MID - LEG_FLANGE_W / 2 - m * math.sqrt(1 + S * S)
    z_t, z_l = -LEG_FLANGE_T, -100.0
    ui = lambda z: u0 + S * (-z - LEG_FLANGE_T)
    one = box(u_f, 150, -(LEG_FLANGE_L / 2 + m), LEG_FLANGE_L / 2 + m, -(LEG_FLANGE_T + m), 20)
    body = (cq.Workplane("XZ").polyline([(ui(z_t), z_t), (150, z_t), (150, z_l), (ui(z_l), z_l)]).close()
            .extrude(LEG_BODY_HW + m, both=True))
    one = one.union(body)
    k = None
    for sx in (1, -1):
        for sy in (1, -1):
            ang = math.degrees(math.atan2(sy, sx))
            c = one.rotate((0, 0, 0), (0, 0, 1), ang)
            k = c if k is None else k.union(c)
    return k


def make_legs_estimate():
    """what the legs probably look like (from the photos) - for pictures and a 2nd fit check"""
    S = 1.39
    out = None
    for sx in (1, -1):
        for sy in (1, -1):
            fl = box(ARM_SCREW_MID - 9, ARM_SCREW_MID + 9, -18, 18, -4.5, 0)
            nuts = U(*[cyl(6.0, -8.5, -4.5, ARM_SCREW_MID, s) for s in (-10.25, 10.25)])
            u_in0 = 53.0
            bar = (cq.Workplane("XZ").polyline([(u_in0, -4.5), (u_in0 + 14, -4.5), (u_in0 + 14 + S * 95, -99.5),
                                                (u_in0 + S * 95, -99.5)]).close().extrude(6.0, both=True))
            leg = U(fl, nuts, bar).rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(sy, sx)))
            out = leg if out is None else out.union(leg)
    return out


# ------------------------------------------------------------------ shell
def make_shell():
    outer = outline(0.0, Z_BOTTOM, 0.0).cut(leg_keepout(0.0))
    inner = outline(WALL, Z_FLOOR_TOP, 10.0).cut(leg_keepout(WALL))
    return outer.cut(inner)


def gusset_x(sx, y, x_at_wall, z_top, leg, thick):
    """45 deg triangle under an overhang that sticks out in +/-X from the wall at x_at_wall"""
    g = cq.Workplane("XZ").polyline([(0, 0), (leg, 0), (0, -leg)]).close().extrude(thick / 2, both=True)
    if sx > 0:
        return g.translate((x_at_wall, y, z_top))
    return g.mirror("YZ").translate((-x_at_wall, y, z_top))


# ------------------------------------------------------------------ TRAY
def make_tray(shell=None):
    shell = make_shell() if shell is None else shell
    t = shell.intersect(box(-200, 200, -200, 200, Z_BOTTOM - 1, Z_SPLIT))

    # 1. PIC32 cradle: fence on 3 sides (port end open), crush ribs, screw boss, corner pad,
    #    J1 pedestal (far end), rubber-foot pedestal (port end)
    fy0 = KIT_W / 2 + FENCE_GAP
    fx1 = KIT_X1 + FENCE_GAP
    fence = U(box(KIT_X0, fx1 + FENCE_T, fy0, fy0 + FENCE_T, Z_FLOOR_TOP - 0.1, Z_FENCE),
              box(KIT_X0, fx1 + FENCE_T, -fy0, -fy0 - FENCE_T, Z_FLOOR_TOP - 0.1, Z_FENCE),
              box(fx1, fx1 + FENCE_T, -fy0 - FENCE_T, fy0 + FENCE_T, Z_FLOOR_TOP - 0.1, Z_FENCE))
    for p in RIB_P:
        for sy in (1, -1):
            fence = fence.union(cyl(2 * RIB_R, Z_FLOOR_TOP - 0.1, Z_FENCE, kx(p), sy * (fy0 + RIB_R - RIB_IN)))
    boss = U(cone(9.0, BOSS_D, Z_FLOOR_TOP - 0.1, Z_PCB_BOT - 2.0, HOLE_X, HOLE_Y),
             cyl(BOSS_D, Z_PCB_BOT - 2.0, Z_PCB_BOT, HOLE_X, HOLE_Y))
    pad = box(kx(PAD_P[0]), kx(PAD_P[1]), -fy0, ky(PAD_Q[1]), Z_FLOOR_TOP - 0.1, Z_PCB_BOT)
    j1 = box(kx(J1_P[0]), kx(J1_P[1]), -J1_HALF, J1_HALF, Z_FLOOR_TOP - 0.1, Z_PCB_BOT - J1_H - J1_GAP)
    foot = cyl(FOOT_D + 3.5, Z_FLOOR_TOP - 0.1, Z_PCB_BOT - FOOT_H, kx(FOOT_P), ky(FOOT_Q))
    t = t.union(fence).union(boss).union(pad).union(j1).union(foot)
    t = t.cut(cyl(BOSS_PILOT, Z_PCB_BOT - 7.0, Z_PCB_BOT + 1, HOLE_X, HOLE_Y))

    # 2. port windows in the left end wall + a dip in the floor for the micro-USB plug
    for (y0, y1, z0, z1) in (WIN_A, WIN_B):
        t = t.cut(box(-OUT_X - 1, -OUT_X + WALL + 0.5, y0, y1, z0, z1))
    t = t.cut(box(-OUT_X + WALL - 0.5, KIT_X0 - 0.5, WIN_A[0], -10.0, WIN_A[2], Z_FLOOR_TOP + 0.01))

    # 3. power bay: zip-tie slots in the floor, vent slots in the right end wall
    for zx in ZIP_X:
        for sy in (1, -1):
            t = t.cut(box(zx - 2.6, zx + 2.6, sy * ZIP_Y - 1.0, sy * ZIP_Y + 1.0, Z_BOTTOM - 1, Z_FLOOR_TOP + 1))
    for vy in VENT_Y:
        t = t.cut(box(OUT_X - WALL - 0.5, OUT_X + 1, vy - VENT_W / 2, vy + VENT_W / 2, *VENT_Z))
    t = t.cut(box(OUT_X - WALL - 0.5, OUT_X + 1, XT60_Y - XT60_W / 2, XT60_Y + XT60_W / 2, XT60_Z0, XT60_Z0 + XT60_H))

    # 4. battery strap ears at floor level on the long walls
    for sx in (1, -1):
        for sy in (1, -1):
            ex = sx * STRAP_X
            ear = box(ex - STRAP_W / 2 - 3.5, ex + STRAP_W / 2 + 3.5, sy * (OUT_Y - 0.5), sy * (OUT_Y + 8),
                      Z_BOTTOM, Z_BOTTOM + FLOOR).edges("|Z").fillet(1.5)
            slot = box(ex - STRAP_W / 2 - 1, ex + STRAP_W / 2 + 1,
                       sy * (OUT_Y + 3.2) - 1.75, sy * (OUT_Y + 3.2) + 1.75, Z_BOTTOM - 1, Z_BOTTOM + FLOOR + 1)
            t = t.union(ear).cut(slot)

    # 5. joint tabs (top of the end-wall corners) with a gusset under each, M3 pilot
    for sx in (1, -1):
        for sy in (1, -1):
            tab = box(sx * JT_X0, sx * JT_X1, sy * (JT_Y - JT_HW), sy * (JT_Y + JT_HW),
                      Z_SPLIT - TRAY_TAB_T, Z_SPLIT).edges("|Z").fillet(1.5)
            leg = JT_X1 - (OUT_X - 0.5)
            g = gusset_x(sx, sy * JT_Y, OUT_X - 0.5, Z_SPLIT - TRAY_TAB_T, leg, 6.0)
            t = t.union(tab).union(g)
            t = t.cut(cyl(JT_PILOT, Z_SPLIT - TRAY_TAB_T - 5.0, Z_SPLIT + 1, sx * JT_X, sy * JT_Y))

    # 6. FRONT engraved on the front wire bay
    try:
        pl = cq.Plane(origin=(0, LOBE_Y, Z_SPLIT - 13.0), xDir=(-1, 0, 0), normal=(0, 1, 0))
        txt = cq.Workplane(pl).text("FRONT", 6.0, -0.6, combine=False, halign="center", valign="center")
        t = t.cut(txt)
    except Exception as e:
        print("text skipped:", e)
    return t


# ------------------------------------------------------------------ COLLAR
def make_collar(shell=None):
    shell = make_shell() if shell is None else shell
    c = shell.intersect(box(-200, 200, -200, 200, Z_SPLIT, 0.0))

    # 1. end plates on the plate tabs: key into each tab slot + M3 bolt + nut trap, 45 deg gussets
    for sx in (1, -1):
        x0, x1 = sorted((sx * (OUT_X - WALL), sx * EAR_X1))
        c = c.union(box(x0, x1, -TAB_HALF_W, TAB_HALF_W, -EAR_T, 0).edges("|Z").fillet(2.0))
        for gy in (GUS_Y_R if sx > 0 else GUS_Y_L):
            c = c.union(gusset_x(sx, gy, OUT_X - 1, -EAR_T, EAR_X1 - OUT_X - 1, 3.0))
        for sy in (1, -1):
            kx0, kx1 = sorted((sx * KEY_X0, sx * KEY_X1))
            c = c.union(box(kx0, kx1, sy * SLOT_Y - KEY_W / 2, sy * SLOT_Y + KEY_W / 2, 0, KEY_H))
            c = c.cut(cyl(BOLT_D, -20, 20, sx * BOLT_X, sy * SLOT_Y))
            c = c.cut(cq.Workplane("XY").polygon(6, NUT_AF / 0.866).extrude(NUT_DEPTH)
                      .translate((sx * BOLT_X, sy * SLOT_Y, -EAR_T)))

    # 2. (the XT60 window is in the tray's right end wall, just above the power board)

    # 3. joint tabs (bottom of the end-wall corners), M3 clearance
    for sx in (1, -1):
        for sy in (1, -1):
            tab = box(sx * JT_X0, sx * JT_X1, sy * (JT_Y - JT_HW), sy * (JT_Y + JT_HW),
                      Z_SPLIT, Z_SPLIT + COLLAR_TAB_T).edges("|Z").fillet(1.5)
            c = c.union(tab).cut(cyl(JT_CLEAR, Z_SPLIT - 1, Z_SPLIT + 10, sx * JT_X, sy * JT_Y))
    # nothing below the joint (the collar prints standing on this face)
    return c.intersect(box(-200, 200, -200, 200, Z_SPLIT, 20))


# ------------------------------------------------------------------ frame + parts (for checks / pictures)
def make_plate():
    d = json.load(open(os.path.join(HERE, "f450_bottom_plate.json")))
    p = cq.Workplane("XY").polyline(d["exterior"][:-1]).close().extrude(PLATE_T)
    for ring in d["interiors"]:
        if len(ring) > 3:
            try:
                p = p.cut(cq.Workplane("XY").polyline(ring[:-1]).close().extrude(PLATE_T + 2).translate((0, 0, -1)))
            except Exception:
                pass
    return p


def make_parts():
    P = {}
    zb, zt = Z_PCB_BOT, Z_PCB_TOP
    P["pcb"] = box(KIT_X0, KIT_X1, -KIT_W / 2, KIT_W / 2, zb, zt).cut(cyl(3.0, zb - 1, zt + 1, HOLE_X, HOLE_Y))
    # top side
    usba = box(KIT_X0 - 0.3, KIT_X0 + 14.0, USBA_Y - 6.55, USBA_Y + 6.55, zt, zt + 7.0)
    mini1 = box(KIT_X0 - 0.3, KIT_X0 + 9.2, MINI1_Y - 3.85, MINI1_Y + 3.85, zt, zt + 3.9)
    mini2 = box(KIT_X0 - 0.3, KIT_X0 + 9.2, MINI2_Y - 3.85, MINI2_Y + 3.85, zt, zt + 3.9)
    j12 = box(kx(KIT_L - 7.2), kx(KIT_L - 1.0), -25.4, 25.4, zt, zt + 2.5)
    pins = box(kx(KIT_L - 6.7), kx(KIT_L - 1.5), -24.9, 24.9, zt + 2.5, zt + 8.5)
    db = box(KIT_X0 - 2.0, KIT_X0 + 26.0, 2.7, 28.0, zt + 7.1, zt + 8.7)
    db_hdr = U(box(KIT_X0 + 1, KIT_X0 + 24, 3.5, 6.0, zt, zt + 7.1), box(KIT_X0 + 1, KIT_X0 + 24, 22.0, 24.5, zt, zt + 7.1))
    rj45 = box(KIT_X0 - 2.5, KIT_X0 + 19.0, 5.6, 21.7, zt + 8.7, zt + 22.2)
    P["kit_top"] = U(usba, mini1, mini2, j12, pins, db, db_hdr, rj45)
    # underside
    j1 = box(kx(57.6), kx(63.5), -24.55, 24.55, zb - J1_H, zb)
    foot = cyl(FOOT_D, zb - FOOT_H, zb, kx(FOOT_P), ky(FOOT_Q))
    j4 = box(KIT_X0 - 0.6, kx(5.9), ky(3.2), ky(11.9), zb - 3.0, zb)
    tails = U(box(kx(65.9), kx(70.3), -25.0, 25.0, zb - 2.0, zb),
              box(KIT_X0 + 0.5, KIT_X0 + 24.5, ky(29.0), ky(31.5), zb - 2.0, zb),
              box(KIT_X0 + 0.5, KIT_X0 + 24.5, ky(47.5), ky(50.0), zb - 2.0, zb))
    P["kit_bot"] = U(j1, foot, j4, tails)
    P["jumpers"] = box(kx(KIT_L - 7.2), kx(KIT_L - 1.0), -25.4, 25.4, zt + 2.5, zt + 16.5)
    P["screw"] = cyl(2.5, zb - 6.4, zt, HOLE_X, HOLE_Y).union(cyl(4.5, zt, zt + 1.8, HOLE_X, HOLE_Y))
    # plugs pushed all the way in (fat over-molds: USB-A 17 x 9, mini 12 x 9, micro 11 x 7.5)
    def plug(y, zc, w, h, x_face):
        return U(box(x_face - 28, x_face, y - w / 2, y + w / 2, zc - h / 2, zc + h / 2),
                 box(x_face - 45, x_face - 28, y - 2, y + 2, zc - 2, zc + 2))
    P["plugs"] = U(plug(USBA_Y, zt + 3.5, 17.0, 9.0, KIT_X0 - 0.3),
                   plug(MINI1_Y, zt + 1.95, 12.0, 9.0, KIT_X0 - 0.3),
                   plug(MINI2_Y, zt + 1.95, 12.0, 9.0, KIT_X0 - 0.3),
                   plug(MICRO_Y, zb - 1.5, 11.0, 7.5, KIT_X0 - 0.6))
    P["pwr"] = box(PWR_X0, PWR_X0 + PWR_L, -PWR_W / 2, PWR_W / 2, Z_FLOOR_TOP, Z_FLOOR_TOP + PWR_H)
    P["xt60"] = box(OUT_X - 10, OUT_X + 8, XT60_Y - 7.75, XT60_Y + 7.75, XT60_Z0 + 0.4, XT60_Z0 + 0.4 + 8.1)
    P["xt60_plug"] = box(OUT_X + 8, OUT_X + 30, XT60_Y - 7.75, XT60_Y + 7.75, XT60_Z0 + 0.4, XT60_Z0 + 0.4 + 8.1)
    P["bat"] = box(-BAT_L / 2, BAT_L / 2, -BAT_W / 2, BAT_W / 2, Z_BOTTOM - BAT_H, Z_BOTTOM)
    P["jt_screws"] = U(*[cyl(3.0, Z_SPLIT - TRAY_TAB_T - 3.0, Z_SPLIT + COLLAR_TAB_T, sx * JT_X, sy * JT_Y)
                         .union(cyl(5.5, Z_SPLIT + COLLAR_TAB_T, Z_SPLIT + COLLAR_TAB_T + 3.0, sx * JT_X, sy * JT_Y))
                         for sx in (1, -1) for sy in (1, -1)])
    return P


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


def main(out):
    os.makedirs(out, exist_ok=True)
    shell = make_shell()
    tray, collar = make_tray(shell), make_collar(shell)
    cq.exporters.export(tray, f"{out}/tray_v3.step")
    cq.exporters.export(collar, f"{out}/collar_v3.step")
    cq.exporters.export(tray.translate((0, 0, -Z_BOTTOM)), f"{out}/tray_v3.stl", tolerance=0.03, angularTolerance=0.1)
    cq.exporters.export(collar.translate((0, 0, -Z_SPLIT)), f"{out}/collar_v3.stl", tolerance=0.03, angularTolerance=0.1)
    coupon = tray.intersect(box(-66, 16.5, -30, 30, Z_BOTTOM - 1, Z_FENCE + 0.01))
    cq.exporters.export(coupon.translate((0, 0, -Z_BOTTOM)), f"{out}/test_cradle_v3.stl", tolerance=0.03, angularTolerance=0.1)

    plate, legs = make_plate(), make_legs_estimate()
    P = make_parts()
    asm = cq.Assembly(name="ELC275_box_v3")
    asm.add(plate, name="F450_bottom_plate", color=cq.Color(0.15, 0.15, 0.15))
    asm.add(legs, name="landing_gear_legs_ESTIMATE", color=cq.Color(0.05, 0.05, 0.05))
    asm.add(tray, name="tray_v3_printed", color=cq.Color(0.9, 0.15, 0.1))
    asm.add(collar, name="collar_v3_printed", color=cq.Color(0.95, 0.95, 0.95))
    asm.add(P["pcb"].union(P["kit_top"]).union(P["kit_bot"]), name="PIC32MZ_EF_starter_kit", color=cq.Color(0.1, 0.5, 0.2))
    asm.add(P["jumpers"], name="J12_dupont_housings", color=cq.Color(0.2, 0.2, 0.2))
    asm.add(P["pwr"], name="power_kill_board_PLACEHOLDER", color=cq.Color(0.9, 0.4, 0.1))
    asm.add(P["xt60"], name="XT60_male", color=cq.Color(0.9, 0.8, 0.1))
    asm.add(P["bat"], name="Battery_3S_2200", color=cq.Color(0.8, 0.6, 0.1))
    asm.save(f"{out}/assembly_v3.step")

    L = []
    L.append(f"box bottom {-Z_BOTTOM:.1f} mm under the plate, battery bottom {BAT_H - Z_BOTTOM:.1f} mm under the plate")
    L.append(f"tray:   {2*OUT_X:.0f} x {2*OUT_Y:.0f} body + front/back bays to y = +/-{LOBE_Y:.0f}, z {Z_BOTTOM:.1f} .. {Z_SPLIT:.1f} "
             f"({Z_SPLIT - Z_BOTTOM:.0f} tall), volume {vol(tray)/1000:.1f} cm3")
    L.append(f"collar: z {Z_SPLIT:.1f} .. 0 ({-Z_SPLIT:.0f} tall) + keys, end plates to x = +/-{EAR_X1:.0f}, volume {vol(collar)/1000:.1f} cm3")
    L.append(f"kit: board underside z = {Z_PCB_BOT:.1f} ({BED_H:.1f} above the floor), top z = {Z_PCB_TOP:.1f}; "
             f"screw boss at ({HOLE_X:.1f}, {HOLE_Y:.1f})")
    L.append(f"tray vs collar overlap: {inter(tray, collar):.3f} mm3")
    L.append(f"kit board vs tray, crush ribs only (intended squeeze): {inter(tray, P['pcb']):.2f} mm3")
    for nm in ("kit_top", "kit_bot", "jumpers", "screw", "plugs", "pwr", "xt60", "xt60_plug", "bat"):
        part = P[nm]
        if nm == "screw":
            part = part.cut(cyl(2.7, Z_PCB_BOT - 9, Z_PCB_BOT + 0.01, HOLE_X, HOLE_Y))
        L.append(f"overlap {nm:9s}: tray {inter(tray, part):8.3f}   collar {inter(collar, part):8.3f} mm3")
    js = P["jt_screws"].cut(U(*[cyl(3.0, Z_SPLIT - TRAY_TAB_T - 3.0, Z_SPLIT, sx * JT_X, sy * JT_Y)
                                for sx in (1, -1) for sy in (1, -1)]))
    L.append(f"overlap joint screws (heads + clearance part): tray {inter(tray, js):.3f}  collar {inter(collar, js):.3f} mm3")
    L.append(f"overlap bottom plate:     tray {inter(tray, plate):.3f}   collar {inter(collar, plate):.3f} mm3")
    L.append(f"overlap legs (photo estimate): tray {inter(tray, legs):.3f}   collar {inter(collar, legs):.3f} mm3")
    K0 = leg_keepout(0.0, clr=0.0)
    L.append(f"overlap legs (conservative model, no margin): tray {inter(tray, K0):.3f}   collar {inter(collar, K0):.3f} mm3")
    # drop-in checks: kit and power board straight down into the open tray
    sweep_kit = U(box(KIT_X0, KIT_X1, -KIT_W / 2, KIT_W / 2, Z_PCB_BOT, Z_SPLIT + 5),
                  box(KIT_X0 - 2.5, KIT_X0 + 26.0, 2.7, 28.0, Z_PCB_TOP + 7.1, Z_SPLIT + 5))
    ribs = U(*[cyl(2 * RIB_R, Z_FLOOR_TOP - 0.1, Z_FENCE, kx(p), sy * (KIT_W / 2 + FENCE_GAP + RIB_R - RIB_IN))
               for p in RIB_P for sy in (1, -1)])
    L.append(f"kit drop-in sweep vs tray (minus crush ribs): {inter(tray.cut(ribs), sweep_kit):.3f} mm3;"
             f" crush-rib squeeze on a {KIT_W:.1f} board: {inter(ribs, sweep_kit):.2f} mm3")
    sweep_pwr = box(PWR_X0, PWR_X0 + PWR_L, -PWR_W / 2, PWR_W / 2, Z_FLOOR_TOP, Z_SPLIT + 5)
    L.append(f"power board drop-in sweep vs tray: {inter(tray, sweep_pwr):.3f} mm3")
    keys = collar.intersect(box(-100, 100, -40, 40, 0, KEY_H))
    L.append(f"keys above z=0: {vol(keys):.1f} mm3, key/plate clash {inter(keys, plate):.3f} mm3")
    open(f"{out}/fit_check_v3.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out")
