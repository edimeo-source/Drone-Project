# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : docs/wiring/src/actual_wiring.py
#  Part    : wiring diagrams
#  What    : Draws the wiring diagram (PNG + PDF next to this folder).
#  FUNCTIONS: W(), T(), box(), screw(), hpin(), tag(), yrow(), ecap(), res()
# ==============================================================================================

# Pictorial ("real parts") wiring diagram: power + Bluetooth emergency stop.
# Same circuit as power_kill_wiring.png, drawn as the parts look.
# python3 actual_wiring.py -> actual_wiring.png / .pdf
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, FancyArrowPatch, Polygon, Wedge

# real-ish wire colours
RED = "#e53935"      # battery + (thick)
BLK = "#212121"      # ground
ORG = "#fb8c00"      # +5 V
TEAL = "#00897b"     # RK4 -> R1
GRN = "#43a047"      # Q1 collector -> CH1
WHT = "#f5f5f5"      # ESC signal
BLU = "#1e88e5"
YEL = "#fdd835"
PUR = "#8e24aa"
PNK = "#ec407a"      # 3V3 to the deck (not red, so it is never mixed up with battery +)
GREY = "#616161"
WARN = "#c62828"

fig = plt.figure(figsize=(17.2, 11.0), dpi=150)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 172)
ax.set_ylim(0, 110)
ax.set_aspect("equal")
ax.axis("off")


def W(pts, c, lw=2.2, casing="white", z=3.0):
    """Wire with a casing so crossings read as one wire passing over another."""
    xs, ys = zip(*pts)
    cas = "#757575" if c in (WHT, YEL) else casing
    ax.plot(xs, ys, color=cas, lw=lw + 2.4, solid_capstyle="round", solid_joinstyle="round", zorder=z)
    ax.plot(xs, ys, color=c, lw=lw, solid_capstyle="round", solid_joinstyle="round", zorder=z + 0.01)


def T(x, y, s, size=8, ha="left", va="center", c="black", w="normal", z=9, **kw):
    ax.text(x, y, s, fontsize=size, ha=ha, va=va, color=c, fontweight=w, zorder=z, **kw)


def box(x, y, w, h, fc, ec="#333333", lw=1.4, r=1.0, z=1, alpha=1.0):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}",
                                fc=fc, ec=ec, lw=lw, zorder=z, alpha=alpha))


def screw(x, y, z=6):
    ax.add_patch(Circle((x, y), 1.25, fc="#cfd8dc", ec="#455a64", lw=1.0, zorder=z))
    ax.plot([x - 0.8, x + 0.8], [y, y], color="#455a64", lw=1.1, zorder=z + 0.1)
    ax.plot([x, x], [y - 0.8, y + 0.8], color="#455a64", lw=1.1, zorder=z + 0.1)


def hpin(x, y, z=7):
    ax.add_patch(Rectangle((x - 0.7, y - 0.7), 1.4, 1.4, fc="#212121", ec="#212121", zorder=z))
    ax.add_patch(Rectangle((x - 0.3, y - 0.3), 0.6, 0.6, fc="#ffca28", ec="none", zorder=z + 0.1))


def tag(x, y, n):
    ax.add_patch(Circle((x, y), 1.45, fc=WARN, ec="white", lw=1.0, zorder=12))
    T(x, y - 0.05, str(n), size=8.2, ha="center", c="white", w="bold", z=13)


# ------------------------------------------------------------------ title
T(2, 106.8, "ELC275 quad: power + Bluetooth emergency stop  (real-parts wiring)", size=15, w="bold")
T(2, 103.3, "Same circuit as the schematic sheet, drawn as the parts look. Wire colours are suggestions; "
  "the pin / terminal at each end is what matters.", size=9.2, c="#333333")

# ================================================================== J12 header (left)
box(24, 44.5, 16, 56.5, "#2e7d32", ec="#1b5e20", r=1.5, z=1)
T(32, 99.0, "PIC32MZ EF kit", size=8.6, ha="center", c="white", w="bold")
T(32, 97.0, "J12 (back side)", size=7.6, ha="center", c="#c8e6c9")
box(27.3, 46.6, 9.4, 49.6, "#111111", ec="#000000", r=0.5, z=2)
XO, XE = 29.2, 34.8


def yrow(k):
    return 95.0 - 2.45 * k


PIN = {}
for k in range(20):
    odd, even = 2 * k + 1, 2 * k + 2
    PIN[odd] = (XO, yrow(k))
    PIN[even] = (XE, yrow(k))
    hpin(XO, yrow(k))
    hpin(XE, yrow(k))
    T(30.4, yrow(k), str(odd), size=5.6, c="#bdbdbd", z=8)
    T(33.6, yrow(k), str(even), size=5.6, c="#bdbdbd", ha="right", z=8)
ax.add_patch(Rectangle((XO - 0.95, yrow(0) - 0.95), 1.9, 1.9, fill=False, ec="#ffffff", lw=1.2, zorder=8))
T(32, 45.4, "pin 1 = square pad", size=6.4, ha="center", c="white", z=8)

# ---- left-side stubs (sensor deck + ESC servo leads)
LEFT = [  # pin, colour, label
    (1, PNK, "3V3 → deck hub"),
    (6, BLK, "GND → deck hub"),
    (7, "#2e7d32", "MISO → deck hub"),
    (11, WHT, "ESC 1 white (front-left)"),
    (12, WHT, "ESC 2 white (front-right)"),
    (13, PUR, "CS BLE → deck hub"),
    (14, BLK, "ESC 1 black"),
    (16, "#9e9e9e", "CS IMU → deck hub"),
    (19, WHT, "ESC 4 white (back-right)"),
    (20, BLK, "ESC 2 black"),
    (21, WHT, "ESC 3 white (back-left)"),
    (23, YEL, "SCK → deck hub"),
    (25, BLK, "ESC 3 black"),
    (30, BLK, "ESC 4 black"),
    (37, BLU, "MOSI → deck hub"),
]
for p, c, lab in LEFT:
    x, y = PIN[p]
    if p % 2 == 1:
        pts = [(x, y), (20.5, y)]
        ly = y
    else:
        ly = y + 1.22
        pts = [(x, y), (x - 1.2, ly), (20.5, ly)]
    W(pts, c, lw=1.5, z=4)
    ax.add_patch(Circle((x, y), 0.45, fc=c if c != WHT else "#bdbdbd", ec="#424242", lw=0.6, zorder=8.5))
    T(20.0, ly, f"pin {p}:  {lab}", size=6.5, ha="right")

# ================================================================== power / kill proto board
BX0, BY0, BX1, BY1 = 50, 9, 93, 55.5
box(BX0, BY0, BX1 - BX0, BY1 - BY0, "#1c1c1c", ec="#000000", r=1.0, z=1)
for gx in range(52, 93, 2):
    for gy in range(11, 55, 2):
        ax.add_patch(Circle((gx, gy), 0.22, fc="#8d6e63", ec="none", zorder=1.1, alpha=0.5))
T(71.5, 7.0, "power / kill proto board (your 7805 board + Q1, R1, R2 and header pins)", size=7.2, ha="center",
  c="#333333")

BP = {
    "GNDin": (56.0, 54.0), "VIN": (61.0, 54.0),
    "5Vj": (50.8, 39.0), "RK4": (50.8, 27.0), "GNDj": (50.8, 13.0),
    "CH1": (92.2, 47.0), "5Vr": (92.2, 42.3), "GNDr": (92.2, 37.7),
}
# rails (bare tinned wire / solder bridges on the back)
W([BP["5Vj"], (88.0, 39.0)], ORG, lw=3.2, z=3)
W([BP["GNDj"], (90.5, 13.0)], BLK, lw=3.2, z=3)
T(78.0, 40.4, "+5 V rail", size=6.0, c="#ffcc80", z=9)
T(76.0, 14.4, "GND rail", size=6.0, c="#e0e0e0", z=9)

# 7805, front view, legs into the board: IN  GND  OUT
box(64.5, 44.5, 11, 5.5, "#212121", ec="#000000", r=0.4, z=6)
box(65.3, 50.0, 9.4, 3.3, "#b0bec5", ec="#607d8b", r=0.4, z=5.9)
ax.add_patch(Circle((70, 51.6), 0.85, fc="#1c1c1c", ec="#607d8b", zorder=6.1))
T(70, 47.2, "MC7805CT", size=5.8, ha="center", c="white", z=9)
for x, lab in ((66.5, "IN"), (70.0, "GND"), (73.5, "OUT")):
    ax.add_patch(Rectangle((x - 0.35, 42.0), 0.7, 2.5, fc="#cfd8dc", ec="#90a4ae", lw=0.5, zorder=6))
T(66.5, 45.6, "IN", size=4.8, ha="center", c="#bdbdbd", z=9)
T(73.5, 45.6, "OUT", size=4.8, ha="center", c="#bdbdbd", z=9)
T(77.0, 51.6, "heatsink on tab", size=5.6, c="#ffab91", z=9)
tag(85.0, 51.6, 2)


def ecap(x, y, label, horiz):
    ax.add_patch(Circle((x, y), 1.6, fc="#263238", ec="#90a4ae", lw=0.9, zorder=6))
    if horiz:   # + on the right, stripe (-) on the left
        ax.add_patch(Wedge((x, y), 1.6, 110, 250, fc="#b0bec5", ec="none", zorder=6.1))
        T(x + 0.5, y, "+", size=6, ha="center", c="white", w="bold", z=9)
    else:       # + on top, stripe (-) at the bottom
        ax.add_patch(Wedge((x, y), 1.6, 200, 340, fc="#b0bec5", ec="none", zorder=6.1))
        T(x, y + 0.5, "+", size=6, ha="center", c="white", w="bold", z=9)
    T(x + 2.0, y + (2.2 if horiz else 0), label, size=5.8, c="#ffcc80", z=9)


def res(x, y, horiz, label, lx=0, ly=0):
    if horiz:
        box(x - 2.4, y - 0.75, 4.8, 1.5, "#d7ccc8", ec="#6d4c41", r=0.6, z=6)
    else:
        box(x - 0.75, y - 2.4, 1.5, 4.8, "#d7ccc8", ec="#6d4c41", r=0.6, z=6)
    T(x + lx, y + ly, label, size=5.8, ha="center", c="#ffcc80", z=9)


# VIN -> 7805 IN ; GND in -> 7805 GND ; C1 across them
W([BP["VIN"], (61, 43.2), (66.5, 43.2)], RED, lw=1.6, z=4)
W([BP["GNDin"], (56, 40.6), (70, 40.6), (70, 42.0)], BLK, lw=1.6, z=4)
W([(56, 48.5), (61, 48.5)], GREY, lw=1.0, z=4.5)
ecap(58.5, 48.5, "", horiz=True)
T(58.5, 46.0, "C1 ≥ 0.33 µF", size=5.6, ha="center", c="#ffcc80", z=9)
# 7805 OUT -> +5 V rail
W([(73.5, 42.0), (73.5, 39.0)], ORG, lw=1.6, z=4)
# 7805 GND -> GND rail: insulated jumper over the +5 V rail
W([(70, 40.6), (70, 13.0)], BLK, lw=1.8, z=5.2)
# C2 between the rails
W([(86.5, 39.0), (86.5, 13.0)], GREY, lw=1.0, z=4.5)
ecap(86.5, 26.0, "C2\n1 µF", horiz=False)
# rail ends -> right header
W([(88.0, 39.0), (88.0, 42.3), BP["5Vr"]], ORG, lw=1.6, z=4)
W([(90.5, 13.0), (90.5, 37.7), BP["GNDr"]], BLK, lw=1.6, z=4)

# relay driver: RK4 -> R1 -> B ; R2 B -> GND ; E -> GND ; C -> CH1
W([BP["RK4"], (65.0, 27.0)], TEAL, lw=1.6, z=4)
res(56.6, 27.0, True, "R1 470 Ω", ly=1.9)
W([(62.0, 27.0), (62.0, 13.0)], GREY, lw=1.0, z=4.5)
res(62.0, 20.0, False, "R2\n10 kΩ", lx=-2.6)
W([(65.0, 25.0), (66.8, 25.0), (66.8, 13.0)], BLK, lw=1.4, z=4)
# Q1 2N3904 seen from above, flat face to the right: pins C (top), B, E (bottom)
ax.add_patch(Wedge((66.0, 27.0), 3.2, 90, 270, fc="#212121", ec="#000000", lw=0.8, zorder=6))
ax.add_patch(Rectangle((66.0, 23.8), 1.2, 6.4, fc="#212121", ec="#000000", lw=0.8, zorder=6))
for y, lab in ((29.0, "C"), (27.0, "B"), (25.0, "E")):
    ax.add_patch(Circle((65.0, y), 0.42, fc="#cfd8dc", ec="none", zorder=6.5))
    T(65.0 - 1.0, y, lab, size=4.8, ha="right", c="white", z=9)
T(71.6, 27.0, "Q1 2N3904\n(flat side →)", size=5.6, c="#ffcc80", z=9)
# C -> CH1: insulated jumper over the GND jumper and the +5 V rail
W([(65.0, 29.0), (65.0, 32.0), (84.0, 32.0), (84.0, 47.0), BP["CH1"]], GRN, lw=1.8, z=5.5)
tag(52.6, 32.6, 4)

for name, (x, y) in BP.items():
    hpin(x, y, z=8)
T(56.0, 52.2, "GND", size=5.4, ha="center", c="white", z=9)
T(61.0, 52.2, "VIN", size=5.4, ha="center", c="white", z=9)
T(52.0, 40.6, "+5V", size=5.4, c="white", z=9)
T(52.0, 28.6, "RK4", size=5.4, c="white", z=9)
T(52.0, 14.6, "GND", size=5.4, c="white", z=9)
T(91.4, 48.6, "CH1", size=5.4, ha="right", c="white", z=9)
T(91.4, 43.9, "+5V", size=5.4, ha="right", c="white", z=9)
T(89.6, 36.0, "GND", size=5.4, ha="right", c="white", z=9)

# ================================================================== J12 -> board
W([PIN[2], (46.0, 95.0), (46.0, 39.0), BP["5Vj"]], ORG, lw=2.0, z=5)
W([PIN[33], (30.4, yrow(16) + 1.22), (44.0, yrow(16) + 1.22), (44.0, 27.0), BP["RK4"]], TEAL, lw=2.0, z=5)
W([PIN[34], (42.0, yrow(16)), (42.0, 13.0), BP["GNDj"]], BLK, lw=2.0, z=5)
T(46.8, 86.0, "pin 2  +5V_EXT in", size=6.6, c=ORG, w="bold", rotation=90)
T(44.7, 49.0, "pin 33  RK4", size=6.6, c=TEAL, w="bold", rotation=90)
T(41.1, 35.0, "pin 34  GND", size=6.6, w="bold", rotation=90, ha="right")

# ================================================================== relay module (turned so its terminals face the wires)
RX0, RY0, RX1, RY1 = 98, 27, 126, 53
box(RX0, RY0, RX1 - RX0, RY1 - RY0, "#d32f2f", ec="#7f0000", r=1.2, z=1)
box(RX0 - 0.6, 30.5, 4.6, 19.5, "#43a047", ec="#1b5e20", r=0.4, z=2)
RT = {"CH1": (100.4, 47.0), "VREF": (100.4, 42.3), "DC-": (100.4, 37.7), "DC+": (100.4, 33.0)}
for k, (x, y) in RT.items():
    screw(x, y)
    T(103.4, y, k.replace("-", "–"), size=6.6, c="white", w="bold", z=9)
box(RX1 - 4.0, 30.0, 4.6, 16.5, "#43a047", ec="#1b5e20", r=0.4, z=2)
RP = {"NC": (123.6, 44.0), "COM": (123.6, 38.3), "NO": (123.6, 32.6)}
for k, (x, y) in RP.items():
    screw(x, y)
    T(121.0, y, k, size=6.6, c="white", w="bold", ha="right", z=9)
box(108.6, 34.0, 9.0, 13.0, "#1565c0", ec="#0d47a1", r=0.6, z=2)
T(113.1, 42.6, "SONGLE", size=5.6, ha="center", c="white", z=9)
T(113.1, 40.2, "SRD-05VDC", size=5.6, ha="center", c="white", z=9)
T(113.1, 38.0, "10 A 30 VDC", size=5.6, ha="center", c="#ffeb3b", w="bold", z=9)
T(112.0, 51.2, "relay module (RM1HLE), turned 90°", size=6.4, ha="center", c="white", w="bold", z=9)
T(112.0, 28.6, "NC: leave EMPTY", size=6.2, ha="center", c="#ffeb3b", z=9)
tag(127.6, 51.4, 1)
# jumper on the module: VREF -> DC+
ax.plot([102.0, 105.0, 105.0, 102.0], [42.3, 40.8, 34.5, 33.0], color="#ffffff", lw=4.0, zorder=7.5,
        solid_joinstyle="round")
ax.plot([102.0, 105.0, 105.0, 102.0], [42.3, 40.8, 34.5, 33.0], color=ORG, lw=2.0, zorder=7.6,
        solid_joinstyle="round")
T(105.6, 36.6, "short jumper\nVREF → DC+", size=5.6, c="white", z=9)
tag(97.0, 55.0, 3)

# board -> relay (dupont on the header pin, bare end under the screw)
W([BP["CH1"], RT["CH1"]], GRN, lw=2.0, z=5)
W([BP["5Vr"], RT["VREF"]], ORG, lw=2.0, z=5)
W([BP["GNDr"], RT["DC-"]], BLK, lw=2.0, z=5)

# ================================================================== frame: bottom plate + arms (top view, FRONT up)
FX, FY = 149.0, 83.0
ARMC = {(-1, 1): "white", (1, 1): "red", (-1, -1): "white", (1, -1): "red"}
ESCN = {(-1, 1): ("1", "front-left", "CCW"), (1, 1): ("2", "front-right", "CW"),
        (-1, -1): ("3", "back-left", "CW"), (1, -1): ("4", "back-right", "CCW")}
for (dx, dy), col in ARMC.items():
    mx, my = FX + dx * 18.5, FY + dy * 17.0
    face = "#fafafa" if col == "white" else "#e53935"
    ax.plot([FX + dx * 9, mx], [FY + dy * 7, my], color="#424242", lw=12, solid_capstyle="round", zorder=1.5)
    ax.plot([FX + dx * 9, mx], [FY + dy * 7, my], color=face, lw=9.5, solid_capstyle="round", zorder=1.6)
    ax.add_patch(Circle((mx, my), 3.3, fc="#9e9e9e", ec="#424242", lw=1.3, zorder=1.7))
    n, nm, sp = ESCN[(dx, dy)]
    T(mx, my, n, size=9, ha="center", w="bold", z=9)
    lab = f"ESC {n}: {nm}\n({col}), {sp}"
    if dx < 0 and dy > 0:
        T(mx, my + 5.6, lab, size=6.2, ha="center")
    elif dx < 0:
        T(mx - 4.0, my, lab, size=6.2, ha="right")
    elif dy > 0:
        T(171.6, my + 5.6, lab, size=6.2, ha="right")
    else:
        T(171.6, my - 5.6, lab, size=6.2, ha="right")
box(FX - 12, FY - 9, 24, 18, "#263238", ec="#000000", r=1.5, z=1.8)
ax.add_patch(FancyArrowPatch((FX, FY - 2), (FX, FY + 7.5), arrowstyle="-|>", mutation_scale=16, color="#42a5f5",
                             lw=2.6, zorder=2))
T(FX, FY + 12.2, "FRONT", size=9, ha="center", c="#1e88e5", w="bold", z=9)
T(FX, FY + 10.0, "F450 bottom plate, top view", size=6.0, ha="center", c="#333333", z=9)
T(FX, FY - 4.6, "battery pads = BACK", size=6.0, ha="center", c="#ffd54f", z=9)
PAD = {"+": (FX - 4.0, FY - 7.6), "-": (FX + 4.0, FY - 7.6)}
for k, (x, y) in PAD.items():
    ax.add_patch(Rectangle((x - 1.4, y - 1.0), 2.8, 2.0, fc="#ffd54f", ec="#8d6e63", lw=0.8, zorder=2.2))
    T(x, y, "+" if k == "+" else "–", size=7.5, ha="center", c=RED if k == "+" else BLK, w="bold", z=9)

# ================================================================== battery + XT60
box(129, 3, 40, 12, "#fff59d", ec="#9e9d24", r=1.2, z=1)
T(149, 10.6, "3S LiPo 2200 mAh  (Velcro under the frame)", size=7.4, ha="center", w="bold")
T(149, 6.8, "11.1 V nominal, 12.6 V full", size=6.6, ha="center", c="#555555")
W([(145, 15), (145, 19.5)], RED, lw=3.6, z=3)
W([(153, 15), (153, 19.5)], BLK, lw=3.6, z=3)
box(141.5, 19.5, 15, 4.2, "#fdd835", ec="#827717", r=0.6, z=4)
box(141.5, 23.9, 15, 4.5, "#ffeb3b", ec="#827717", r=0.6, z=4)
T(149, 21.6, "XT60 on battery", size=5.8, ha="center", z=9)
T(149, 26.1, "XT60 male (drone)", size=5.8, ha="center", z=9)
T(143.4, 26.1, "+", size=7, c=RED, w="bold", ha="center", z=9)
T(154.6, 26.1, "–", size=7, w="bold", ha="center", z=9)

# thick battery - : XT60 -> plate - pad
W([(153, 28.4), (153, PAD["-"][1] - 1.0)], BLK, lw=4.2, z=5)
# thick battery + : XT60 -> relay COM (always live)
W([(145, 28.4), (145, 38.3), RP["COM"]], RED, lw=4.2, z=5)
# switched + : relay NO -> plate + pad
W([RP["NO"], (138.5, 32.6), (138.5, 70.0), (PAD["+"][0], 70.0), (PAD["+"][0], PAD["+"][1] - 1.0)],
  "#b71c1c", lw=4.2, z=5.5)
T(139.6, 50.0, "switched +", size=6.8, c="#b71c1c", w="bold", rotation=90)
T(134.5, 40.0, "battery +\n(always live)", size=6.2, c=RED, w="bold", ha="center")
T(154.2, 45.0, "battery –", size=6.8, w="bold", rotation=90)
T(130.0, 30.6, "14 AWG", size=6.0, c="#555555", ha="center")

# thin taps for the 7805 board
W([RP["COM"], (127.6, 39.6), (127.6, 58.0), (61.0, 58.0), BP["VIN"]], RED, lw=1.5, z=6)
W([(PAD["-"][0] - 1.6, PAD["-"][1] - 1.0), (151.4, 62.0), (56.0, 62.0), BP["GNDin"]], BLK, lw=1.5, z=6.2)
T(64.0, 59.3, "thin red (22 AWG): VIN tap, under the COM screw with the battery + lead", size=6.3, c=RED)
T(64.0, 63.3, "thin black (22 AWG): GND tap, soldered on the – pad with the battery – lead", size=6.3)

# servo-lead note + ESC tag
T(157.2, 56.0, "ESC power leads are already\nsoldered to the arm pads.\n\nServo leads → J12:\nwhite = signal pin\nblack = GND pin\nred = tape back,\n          NOT connected",
  size=6.2, va="top", c="#333333")
tag(155.2, 57.4, 5)

# ================================================================== sensor deck hub (left bottom)
box(2.5, 27.0, 26, 14.5, "#eceff1", ec="#90a4ae", r=1.0, z=1)
T(15.5, 39.6, "Sensor deck hub: 7-pin header", size=7.0, ha="center", w="bold")
HUB = [("GND", BLK, 6), ("3V3", PNK, 1), ("MOSI", BLU, 37), ("SCK", YEL, 23), ("MISO", "#2e7d32", 7),
       ("CS IMU", "#9e9e9e", 16), ("CS BLE", PUR, 13)]
for i, (lab, c, p) in enumerate(HUB):
    x = 5.0 + i * 3.4
    hpin(x, 33.6)
    W([(x, 33.6), (x, 36.8)], c, lw=1.4, z=4)
    T(x, 31.2, lab, size=5.0, ha="center")
    T(x, 29.0, f"pin {p}", size=5.2, ha="center", c="#333333", w="bold")

# ================================================================== notes
notes = [
    (1, "10 A relay: bench / low throttle only. Hover ≈ 17.6 A,\nso for flight swap in a 30 A module (same terminals)."),
    (2, "7805 burns ≈ 1.9 W on a full pack: heatsink on the tab,\nkeep it off the PLA."),
    (3, "VREF on +5 V makes CH1 active-LOW: Q1 pulls it down.\nRK4 HIGH = motors powered; reset / RK4 LOW = dead."),
    (4, "Two board wires cross others (7805 GND → GND rail, Q1 C → CH1):\nuse insulated wire for those two."),
    (5, "Servo leads: white → signal pin, black → GND pin, red taped back.\nOnly solder off the board: the plate + and – pads."),
]
y = 23.2
for n, s in notes:
    tag(3.6, y, n)
    T(6.2, y, s, size=6.6)
    y -= 4.3
T(2.5, 1.6, "Before the battery: USB in, meter pin 1 → GND = 3.3 V (finds pin 1).  + and – pads not shorted.  NC empty.",
  size=6.3, c="#333333")

# legend (under the relay)
lx, ly = 98.0, 23.0
T(lx, ly + 2.0, "Wires", size=7.6, w="bold")
for i, (c, lab, lw) in enumerate(((RED, "battery + (always live)", 4.2), ("#b71c1c", "switched + to the plate", 4.2),
                                  (BLK, "ground", 2.2), (ORG, "+5 V", 2.2), (TEAL, "RK4 (relay control)", 2.2),
                                  (GRN, "Q1 collector → CH1", 2.2))):
    yy = ly - 1.4 - i * 2.5
    W([(lx, yy), (lx + 4, yy)], c, lw=lw, z=4)
    T(lx + 5.0, yy, lab, size=6.3)
T(126.0, 1.6, "Ethan DiMeo, ELC275-501, Nov 19 2026", size=6.0, c="#757575", ha="right")

for ext in ("png", "pdf"):
    fig.savefig(f"actual_wiring.{ext}", dpi=150)
print("saved")
