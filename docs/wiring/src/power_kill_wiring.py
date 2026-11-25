# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : docs/wiring/src/power_kill_wiring.py
#  Part    : wiring diagrams
#  What    : Draws the wiring diagram (PNG + PDF next to this folder).
#  FUNCTIONS: wire(), dot(), text(), block(), ground(), resistor_h(), resistor_v(), cap_v(),
#    npn(), tag(), spin()
# ==============================================================================================

# Power + Bluetooth emergency-stop wiring sheet (matches the two whiteboards)
# python3 power_kill_wiring.py  ->  power_kill_wiring.png / .pdf
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Arc, Polygon, FancyArrowPatch

RED = "#c62828"      # battery + (always live)
SW = "#8e24aa"       # battery + after the relay (switched)
FIVE = "#ef6c00"     # +5 V from the 7805
SIG = "#1565c0"      # 3.3 V logic
GND = "#212121"
GREY = "#757575"
WARN = "#d32f2f"

fig = plt.figure(figsize=(17.2, 11.0), dpi=150)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 172)
ax.set_ylim(0, 110)
ax.set_aspect("equal")
ax.axis("off")


def wire(pts, c=GND, lw=2.0, ls="-", z=2):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=c, lw=lw, ls=ls, solid_capstyle="round", zorder=z)


def dot(x, y, c=GND):
    ax.add_patch(Circle((x, y), 0.55, color=c, zorder=5))


def text(x, y, s, size=9, ha="left", va="center", c="black", w="normal", **kw):
    ax.text(x, y, s, fontsize=size, ha=ha, va=va, color=c, fontweight=w, zorder=6, **kw)


def block(x, y, w, h, title, fc="#f5f5f5", ec="#424242", tsize=10):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3,rounding_size=1.2",
                                fc=fc, ec=ec, lw=1.6, zorder=1))
    text(x + w / 2, y + h - 2.2, title, size=tsize, ha="center", w="bold")


def ground(x, y, c=GND):
    # y = top of the symbol (wire end)
    wire([(x, y), (x, y - 1.2)], c, 1.8)
    for i, half in enumerate((1.6, 1.0, 0.4)):
        yy = y - 1.2 - i * 0.6
        wire([(x - half, yy), (x + half, yy)], c, 1.8)


def resistor_h(x1, x2, y, c, label, above=True):
    n = 6
    seg = (x2 - x1) * 0.6
    a = x1 + (x2 - x1 - seg) / 2
    pts = [(x1, y), (a, y)]
    for i in range(n):
        pts.append((a + seg * (i + 0.5) / n, y + (0.9 if i % 2 == 0 else -0.9)))
    pts += [(a + seg, y), (x2, y)]
    wire(pts, c, 1.8)
    text((x1 + x2) / 2, y + (2.0 if above else -2.0), label, size=8.5, ha="center")


def resistor_v(x, y1, y2, c, label, left=False):
    n = 6
    seg = (y1 - y2) * 0.6
    a = y1 - (y1 - y2 - seg) / 2
    pts = [(x, y1), (x, a)]
    for i in range(n):
        pts.append((x + (0.9 if i % 2 == 0 else -0.9), a - seg * (i + 0.5) / n))
    pts += [(x, a - seg), (x, y2)]
    wire(pts, c, 1.8)
    text(x - 1.6 if left else x + 1.6, (y1 + y2) / 2, label, size=8.5, ha="right" if left else "left")


def cap_v(x, ytop, ybot, label, c=GND):
    mid = (ytop + ybot) / 2
    wire([(x, ytop), (x, mid + 0.5)], c, 1.8)
    wire([(x - 1.6, mid + 0.5), (x + 1.6, mid + 0.5)], c, 2.2)          # + plate
    ax.add_patch(Arc((x, mid - 1.6), 3.6, 3.0, theta1=40, theta2=140, color=c, lw=2.2, zorder=3))
    wire([(x, mid - 0.1), (x, ybot)], c, 1.8)
    text(x - 1.9, mid + 1.4, "+", size=8, ha="right")
    text(x + 2.2, mid, label, size=8.2)


def npn(cx, cy, c=SIG):
    ax.add_patch(Circle((cx, cy), 3.0, fill=False, ec=GND, lw=1.5, zorder=3))
    wire([(cx - 1.0, cy - 1.8), (cx - 1.0, cy + 1.8)], GND, 2.4)        # base bar
    wire([(cx - 3.6, cy), (cx - 1.0, cy)], c, 1.8)                       # base lead
    wire([(cx - 1.0, cy + 0.8), (cx + 1.2, cy + 2.2), (cx + 1.2, cy + 3.6)], GND, 1.8)   # collector
    wire([(cx - 1.0, cy - 0.8), (cx + 1.2, cy - 2.2), (cx + 1.2, cy - 3.6)], GND, 1.8)   # emitter
    ax.add_patch(FancyArrowPatch((cx - 0.4, cy - 1.18), (cx + 0.9, cy - 2.0), arrowstyle="-|>",
                                 mutation_scale=9, color=GND, lw=1.2, zorder=4))
    return (cx + 1.2, cy + 3.6), (cx + 1.2, cy - 3.6)


def tag(x, y, n):
    ax.add_patch(Circle((x, y), 1.5, fc=WARN, ec="white", lw=1.0, zorder=7))
    ax.text(x, y - 0.05, str(n), fontsize=8.5, ha="center", va="center", color="white", fontweight="bold", zorder=8)


def spin(x, y, r, cw, c):
    # arc arrow seen from ABOVE
    if cw:
        ax.add_patch(Arc((x, y), 2 * r, 2 * r, theta1=-30, theta2=200, color=c, lw=2.0, zorder=4))
        a = math.radians(-30)
        tip = (x + r * math.cos(a), y + r * math.sin(a))
        d = (math.sin(a), -math.cos(a))                                    # clockwise tangent
    else:
        ax.add_patch(Arc((x, y), 2 * r, 2 * r, theta1=-20, theta2=210, color=c, lw=2.0, zorder=4))
        a = math.radians(210)
        tip = (x + r * math.cos(a), y + r * math.sin(a))
        d = (-math.sin(a), math.cos(a))                                    # counter-clockwise tangent
    ax.add_patch(FancyArrowPatch((tip[0] - d[0] * 0.8, tip[1] - d[1] * 0.8), (tip[0] + d[0] * 0.6, tip[1] + d[1] * 0.6),
                                 arrowstyle="-|>", mutation_scale=12, color=c, lw=1.5, zorder=5))


# ------------------------------------------------------------------ title
text(2, 107, "ELC275 quad: power + Bluetooth emergency stop", size=15, w="bold")
text(2, 103.6, "Matches both whiteboards: 7805 (LDO) from Vbatt with input/output caps -> PIC32 kit;  "
     "relay switches Vbatt to the motors;  GPIO -> 470 Ω -> 2N3904 drives the relay.",
     size=9.5, c="#333333")

# ------------------------------------------------------------------ battery + XT60
block(3, 50, 11, 24, "BATTERY", fc="#fffde7")
text(8.5, 62.5, "3S LiPo\n2200 mAh\n11.1 V nom\n12.6 V full", size=8.2, ha="center")
block(15.2, 56, 5.2, 12, "XT60", fc="#eeeeee", tsize=8)
text(17.8, 61.4, "male\non\ndrone", size=7, ha="center", c=GREY)
text(21.4, 67.2, "+", size=11, w="bold", c=RED)
text(19.4, 56.4, "–", size=11, w="bold")

# Vbatt (always live): XT60 + -> up -> rail at y=98 -> relay COM
wire([(20.4, 65.5), (24.5, 65.5), (24.5, 98), (99, 98), (99, 88), (95.3, 88)], RED, 3.4)
text(40, 99.8, "VBATT  (always live, 9.0–12.6 V)   same gauge as the battery lead", size=8.5, c=RED, w="bold")
dot(31, 98, RED)

# GND rail: XT60 - -> rail at y=30 -> plate - pad
wire([(20.4, 58.5), (22.0, 58.5), (22.0, 30), (102, 30), (102, 41), (104, 41)], GND, 3.4)
text(40, 28.0, "BATTERY –  =  common ground (every ground symbol on this sheet ties here)", size=8.5, w="bold")

# ------------------------------------------------------------------ 7805 + caps
block(40, 79, 12, 10, "U1 MC7805CT", fc="#e3f2fd", tsize=8.5)
text(41.0, 83.4, "1 IN", size=7.5)
text(51.0, 83.4, "3 OUT", size=7.5, ha="right")
text(46.0, 80.6, "2 GND", size=7.5, ha="center")
wire([(31, 98), (31, 84), (40, 84)], RED, 2.2)
dot(31, 84, RED)
cap_v(31, 84, 75, "C1\n≥ 0.33 µF\n≥ 25 V", RED)
ground(31, 75)
wire([(46, 79), (46, 75)], GND, 2.0)
ground(46, 75)
wire([(52, 84), (63, 84)], FIVE, 2.6)
dot(53.5, 84, FIVE)
dot(56.5, 84, FIVE)
cap_v(56.5, 84, 75, "C2\n1 µF\n50 V", FIVE)
ground(56.5, 75)
tag(46, 91.4, 2)

# +5 V rail -> relay DC+ and VREF, and down to J12 pin 2
wire([(63, 84), (63, 88), (69, 88)], FIVE, 2.6)
wire([(63, 84), (63, 76), (69, 76)], FIVE, 2.2)
dot(63, 84, FIVE)
text(63.4, 90.0, "+5 V", size=9, c=FIVE, w="bold")
wire([(53.5, 84), (53.5, 63), (42, 63)], FIVE, 2.2)

# ------------------------------------------------------------------ relay module
MX = 69.0
block(MX, 62, 26.3, 31, "K1 relay module (RM1HLE)", fc="#ffebee", tsize=9)
for y, lab in ((88, "DC+"), (82, "DC–"), (76, "VREF"), (70, "CH1")):
    ax.add_patch(Rectangle((MX - 0.9, y - 0.9), 1.8, 1.8, fc="#66bb6a", ec="#2e7d32", zorder=4))
    text(MX + 1.6, y, lab, size=8, w="bold")
RX = MX + 26.3
for y, lab in ((88, "COM"), (80, "NO"), (72, "NC")):
    ax.add_patch(Rectangle((RX - 0.9, y - 0.9), 1.8, 1.8, fc="#66bb6a", ec="#2e7d32", zorder=4))
    text(RX - 1.6, y, lab, size=8, w="bold", ha="right")
cx = MX + 13
for i in range(4):
    ax.add_patch(Arc((cx, 79.8 - i * 1.6), 2.2, 1.6, theta1=-90, theta2=90, color=GND, lw=1.4, zorder=3))
wire([(cx, 81.6), (cx, 83)], GND, 1.2)
wire([(cx, 74.4), (cx, 73)], GND, 1.2)
text(cx, 85.4, "coil 5 V, 70 Ω, 71 mA", size=6.8, ha="center", c=GREY)
wire([(RX - 4.3, 88), (RX - 4.3, 85.0), (RX - 6.7, 81.0)], GND, 1.4)          # COM blade
wire([(RX - 4.3, 80), (RX - 6.3, 80)], GND, 1.4)                               # NO contact
wire([(RX - 4.3, 72), (RX - 6.3, 72), (RX - 7.0, 74.2)], GND, 1.4)             # NC contact
wire([(cx + 1.2, 78.0), (RX - 7.3, 79.2)], GREY, 1.0, ls=":")
text(cx, 67.0, "opto + driver transistor + LED on board", size=6.8, ha="center", c=GREY)
text(cx, 64.6, "VREF on +5 V  →  CH1 is active-LOW", size=6.8, ha="center", c=GREY)
text(RX - 1.0, 75.6, "10 A\ncontacts", size=6.8, ha="right", c=WARN, w="bold")
tag(RX + 1.8, 75.6, 1)

wire([(MX - 1.8, 82), (MX - 3.5, 82), (MX - 3.5, 80.6)], GND, 1.8)
ground(MX - 3.5, 80.6)
tag(MX - 2.0, 92.4, 3)

# switched Vbatt: NO -> plate + pad
wire([(RX, 80), (104, 80)], SW, 3.4)
text(99.8, 83.0, "switched\nVBATT", size=7.5, c=SW, w="bold", ha="center")

# ------------------------------------------------------------------ F450 plate
block(104, 34, 13.2, 55, "F450 plate", fc="#fafafa", tsize=9)
text(110.6, 80.0, "+ pad\n(BACK)", size=8, ha="center", c=SW, w="bold")
text(110.6, 41.0, "– pad\n(BACK)", size=8, ha="center", w="bold")
text(110.6, 61, "built-in\npower board\n\nESC 1–4\nred/black\npower leads\n(already\nsoldered)", size=7.4,
     ha="center", c="#333333")
text(110.6, 36.4, "ESC signal\nplugs → J12", size=6.8, ha="center", c=GREY)
tag(110.6, 91.6, 5)

# ------------------------------------------------------------------ PIC32 kit
block(27, 38, 15, 30, "PIC32MZ EF\nStarter Kit", fc="#e8f5e9", tsize=9)
for y, lab in ((63, "pin 2  +5V_EXT"), (50, "pin 33  RK4"), (44, "pin 34  GND")):
    ax.add_patch(Rectangle((42 - 0.8, y - 0.8), 1.6, 1.6, fc="#424242", zorder=4))
    text(40.6, y, lab, size=7.6, ha="right")
text(34.5, 41.0, "J12 header", size=7.2, ha="center", c=GREY)
wire([(42, 44), (45.5, 44), (45.5, 42.5)], GND, 1.8)
ground(45.5, 42.5)
text(34.5, 34.4, "inside the kit: +5V_EXT → Schottky D10 → 3.3 V LDO;\n"
     "USB debug → Schottky D2 → same LDO.\nBattery + USB plugged in together is OK.",
     size=6.8, ha="center", c=GREY)

# ------------------------------------------------------------------ Q1 driver (whiteboard)
wire([(42, 50), (44.5, 50)], SIG, 1.8)
resistor_h(44.5, 53.0, 50, SIG, "R1 470 Ω")
wire([(53.0, 50), (55.0, 50)], SIG, 1.8)
dot(55.0, 50, SIG)
resistor_v(55.0, 50, 43.5, GND, "R2\n10 kΩ", left=True)
ground(55.0, 43.5)
coll, emit = npn(59.6, 50)
text(63.6, 49.0, "Q1\n2N3904", size=8, w="bold")
wire([coll, (coll[0], 70), (MX, 70)], GND, 1.8)
wire([emit, (emit[0], 44.5)], GND, 1.8)
ground(emit[0], 44.5)
tag(48.8, 54.6, 4)

# ------------------------------------------------------------------ legend (lower left)
text(2.5, 47.0, "Wire colours", size=8.5, w="bold")
for i, (c, lab) in enumerate(((RED, "Vbatt, always live"), (SW, "Vbatt after relay"),
                              (FIVE, "+5 V (7805)"), (SIG, "3.3 V logic"), (GND, "ground / other"))):
    yy = 44.0 - i * 2.7
    wire([(2.8, yy), (6.0, yy)], c, 3.0)
    text(7.0, yy, lab, size=7.6)

# ------------------------------------------------------------------ notes (panel A)
notes = [
    (1, "Relay contacts are 10 A. Hover is about 4 × 4.4 A = 17.6 A (full throttle roughly 2× that). "
        "For FLIGHT use a 30 A (or bigger) version of this module\n(same DC+ / DC– / IN / COM / NO wiring). "
        "This 10 A one is OK only for low-throttle bench tests."),
    (2, "7805 heat: (12.6 – 5.0) V × ~0.25 A ≈ 1.9 W → about 124 °C rise with no heatsink "
        "(65 °C/W), far past the 125 °C rating. Bolt a TO-220 heatsink to the tab. Electrolytic caps: stripe (–) to ground."),
    (3, "Relay coil (module DC+) goes on +5 V from the 7805, never on Vbatt: 12.6 V on a 5 V coil is about 2.3 W vs 0.36 W rated."),
    (4, "RK4 HIGH → Q1 on → CH1 pulled LOW → relay closes → motors powered.  RK4 LOW, reset, or PIC unpowered "
        "→ relay open → motors dead.\nShortcut without Q1/R1/R2: VREF → GND and CH1 straight to pin 33 "
        "(active-HIGH). Test first that 3.3 V clicks the relay."),
    (5, "Before closing the relay, send 1.0 ms pulses so the ESCs arm at zero throttle. All 4 ESC red servo "
        "wires stay unconnected (the 7805 is the only 5 V source)."),
]
y = 24.2
for n, s in notes:
    tag(3.6, y, n)
    text(6.0, y, s, size=8.0, va="center")
    y -= 4.8 if "\n" in s else 3.8

# divider
wire([(120.2, 2), (120.2, 108)], "#bdbdbd", 1.0)

# ------------------------------------------------------------------ panel B: top view
text(122, 107, "Top view (looking down)", size=12.5, w="bold")
text(122, 104.0, "Front = opposite the battery pads (frame maker).\nWhite arms = LEFT,  red arms = RIGHT.",
     size=8.3, c="#333333", va="top")
X0, Y0 = 146, 73
ax.add_patch(FancyArrowPatch((X0, Y0 + 9.5), (X0, Y0 + 24.5), arrowstyle="-|>", mutation_scale=22,
                             color="#1e88e5", lw=3.5, zorder=4))
text(X0, Y0 + 26.6, "FRONT", size=11, c="#1e88e5", w="bold", ha="center")
arms = {  # name: (dx, dy, colour, esc, pin, cw)
    "FL": (-1, 1, "white", 1, 11, False),
    "FR": (1, 1, "red", 2, 12, True),
    "BL": (-1, -1, "white", 3, 21, True),
    "BR": (1, -1, "red", 4, 19, False),
}
for name, (dx, dy, col, esc, pin, cw) in arms.items():
    mx, my = X0 + dx * 13, Y0 + dy * 11.5
    face = "#ffffff" if col == "white" else "#e53935"
    ax.plot([X0 + dx * 4, mx], [Y0 + dy * 4, my], color="#424242", lw=11, solid_capstyle="round", zorder=2)
    ax.plot([X0 + dx * 4, mx], [Y0 + dy * 4, my], color=face, lw=8.5, solid_capstyle="round", zorder=2)
    ax.add_patch(Circle((mx, my), 3.0, fc="#9e9e9e", ec="#424242", lw=1.5, zorder=3))
    spin(mx, my, 4.7, cw, "#1565c0")
    side = "LEFT" if dx < 0 else "RIGHT"
    end = "FRONT" if dy > 0 else "BACK"
    ty = my + (5.6 if dy > 0 else -5.6)
    text(mx, ty, f"{end}-{side}  ({col})\nESC {esc} → J12 pin {pin}\n{'CW' if cw else 'CCW'}",
         size=7.6, ha="center", va="bottom" if dy > 0 else "top")
ax.add_patch(FancyBboxPatch((X0 - 6.5, Y0 - 6.5), 13, 13, boxstyle="round,pad=0.2,rounding_size=1.5",
                            fc="#263238", ec="#000000", lw=1.2, zorder=3))
ax.add_patch(Rectangle((X0 - 2.2, Y0 - 3.2), 4.4, 6.4, fc="#e53935", ec="white", lw=0.8, zorder=4))
ax.add_patch(FancyArrowPatch((X0, Y0 - 1.0), (X0, Y0 + 3.0), arrowstyle="-|>", mutation_scale=9,
                             color="white", lw=1.4, zorder=5))
ax.add_patch(FancyArrowPatch((X0, Y0 - 1.0), (X0 - 3.6, Y0 - 1.0), arrowstyle="-|>", mutation_scale=9,
                             color="white", lw=1.4, zorder=5))
text(X0 + 0.5, Y0 + 2.4, "X", size=6.5, c="white", w="bold")
text(X0 - 4.3, Y0 + 0.2, "Y", size=6.5, c="white", w="bold")
for i, (s, c) in enumerate((("+", RED), ("–", "#212121"))):
    ax.add_patch(Rectangle((X0 - 3.2 + i * 3.6, Y0 - 6.2), 2.6, 1.8, fc="#ffd54f", ec="#8d6e63", lw=0.8, zorder=4))
    text(X0 - 1.9 + i * 3.6, Y0 - 5.3, s, size=7, ha="center", c=c, w="bold")
text(X0, Y0 - 9.0, "battery pads = BACK", size=7.4, ha="center", c="#5d4037")
text(122, 48.4, "IMU: X arrow → FRONT, Y arrow → LEFT. Your red v2 adapter\n"
     "already sits this way: do NOT rotate it.\n"
     "Pitch pairs: FRONT = ESC 1 + 2 (pins 11, 12),\n"
     "BACK = ESC 3 + 4 (pins 21, 19). Each pair is one CW + one CCW,\n"
     "so yaw torque still cancels. The motors spin the same as before;\n"
     "only the names changed. Plug ESC signal leads by THIS picture.",
     size=7.6, va="top", c="#333333")

# ------------------------------------------------------------------ panel C: J12 table
wire([(121.5, 34.6), (171, 34.6)], "#bdbdbd", 1.0)
text(122, 32.4, "J12 connections on this sheet", size=11, w="bold")
rows = [
    ("2", "+5 V in ← 7805 OUT"),
    ("33 (RK4)", "→ R1 → Q1 base  (HIGH = motors ON)"),
    ("34", "GND: the only power-board ground to the kit"),
    ("11 / 12", "ESC 1 front-left / ESC 2 front-right"),
    ("21 / 19", "ESC 3 back-left / ESC 4 back-right"),
    ("14, 20, 25, 30", "ESC black servo wires"),
    ("1, 6, 37, 23, 7, 16, 13", "IMU + Bluefruit cable (unchanged)"),
]
yy = 30.0
for p, g in rows:
    yy -= 2.55
    text(122, yy, p, size=7.6, w="bold")
    text(140.5, yy, g, size=7.6)
text(122, 2.4, "Sources: onsemi MC7805 datasheet; Songle SRD-05VDC-SL-C datasheet;\n"
     "Microchip DS70005230B (Table 2-3, Fig. A-5); relay-module VREF/CH1 notes.\n"
     "Ethan DiMeo, ELC275-501, Nov 18 2026.",
     size=6.4, c=GREY, va="bottom")

for ext in ("png", "pdf"):
    fig.savefig(f"power_kill_wiring.{ext}", dpi=150)
print("saved")
