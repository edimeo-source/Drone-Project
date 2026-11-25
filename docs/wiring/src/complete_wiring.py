# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : docs/wiring/src/complete_wiring.py
#  Part    : wiring diagrams
#  What    : Draws the wiring diagram (PNG + PDF next to this folder).
#  FUNCTIONS: W(), T(), box(), screw(), hpin(), tag(), ecap(), res(), yrow()
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


fig = plt.figure(figsize=(20.0, 12.5), dpi=150)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 200)
ax.set_ylim(0, 125)
ax.set_aspect("equal")
ax.axis("off")
MAIN = ax
OX, OY = 22.0, 22.0
ax2 = fig.add_axes([(48 + OX) / 200, (0 + OY) / 125, (172 - 48) / 200, 110 / 125])
ax2.set_xlim(48, 172)
ax2.set_ylim(0, 110)
ax2.set_aspect("equal")
ax2.axis("off")
ax2.patch.set_visible(False)

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


# ===== power section drawn in ax2 (old sheet coordinates)
ax = ax2
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
FX, FY = 149.0, 80.0
ARMC = {(-1, 1): "white", (1, 1): "red", (-1, -1): "white", (1, -1): "red"}
ESCN = {(-1, 1): ("1", "front-left", "CCW"), (1, 1): ("2", "front-right", "CW"),
        (-1, -1): ("3", "back-left", "CW"), (1, -1): ("4", "back-right", "CCW")}
for (dx, dy), col in ARMC.items():
    mx, my = FX + dx * 18.5, FY + dy * 14.0
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


# ======================================================================================
# MAIN SHEET (new coordinates, 200 x 125 units = 20 x 12.5 in)
# ======================================================================================
ax = MAIN

T(2, 121.6, "ELC275 quad: complete wiring  (power, kill switch, IMU + Bluetooth, ESCs)", size=16, w="bold")
T(2, 117.9, "Battery → 7805 → PIC32 kit → IMU + Bluefruit stays powered through a kill. "
  "The relay cuts only the plate's + input, which feeds the 4 ESCs.", size=9.6, c="#333333")

# ---------------------------------------------------------------- sensor deck
box(2, 56, 31.5, 57.5, "#eceff1", ec="#90a4ae", r=1.2, z=0.5)
T(17.7, 111.5, "SENSOR DECK  (centre of the bottom plate)", size=6.8, ha="center", w="bold")

COL = [5.4 + 2.6 * c for c in range(9)]
R0, R1, R2, R3 = 97.0, 93.5, 90.0, 86.0

# IMU board
box(4.5, 99.6, 24.0, 9.6, "#d32f2f", ec="#7f0000", r=0.8, z=2)
T(16.5, 107.4, "SparkFun LSM6DSO IMU", size=6.4, ha="center", c="white", w="bold")
T(16.5, 105.4, "3.3 V only  ·  X arrow → FRONT", size=5.6, ha="center", c="#ffcdd2")
for c, lab in zip(range(2, 8), ("GND", "3V3", "SDA", "SCL", "SDO", "CS")):
    T(COL[c], 102.6, lab, size=4.6, ha="center", c="white", z=9)
    ax.plot([COL[c], COL[c]], [101.2, R0], color="#9e9e9e", lw=1.2, zorder=2.5)
    hpin(COL[c], 101.2, z=3)

# hub strip (perfboard)
box(4.0, 84.6, 26.4, 13.9, "#1c1c1c", ec="#000000", r=0.6, z=1)
for gx in COL:
    for gy in (R0, R1, R2, R3):
        ax.add_patch(Circle((gx, gy), 0.22, fc="#8d6e63", ec="none", zorder=1.1, alpha=0.6))
T(29.0, 89.9, "hub\nstrip", size=4.6, ha="right", c="#bdbdbd", z=9)
for c in range(2, 8):
    hpin(COL[c], R0, z=4)
for c in range(0, 9):
    hpin(COL[c], R3, z=4)
# solder bridges (silver): IMU pin -> header pin beside it; Bluefruit GND -> header GND
for c in range(2, 8):
    ax.plot([COL[c], COL[c]], [R0, R1], color="#cfd8dc", lw=2.4, zorder=4.5, solid_capstyle="round")
ax.plot([COL[2], COL[2]], [R3, R1], color="#cfd8dc", lw=2.4, zorder=4.5, solid_capstyle="round")
# insulated links from the Bluefruit pins to the header
for (cf, ct, colr) in ((0, 3, PNK), (6, 4, BLU), (8, 5, YEL), (7, 6, "#2e7d32"), (5, 8, PUR)):
    W([(COL[cf], R3), (COL[cf], 88.0), (COL[ct], 91.4), (COL[ct], R1)], colr, lw=1.3, z=5)
# 7-pin dupont housing on the header row (pins up through the strip, cable plugs on top)
box(COL[2] - 1.3, R1 - 1.25, COL[8] - COL[2] + 2.6, 2.5, "#111111", ec="#000000", r=0.3, z=6)
for c, lab, colr in ((2, "GND", BLK), (3, "3V3", PNK), (4, "MOSI", BLU), (5, "SCK", YEL), (6, "MISO", "#2e7d32"),
                     (7, "CS I", "#9e9e9e"), (8, "CS B", PUR)):
    ax.add_patch(Rectangle((COL[c] - 1.1, R1 - 1.15), 2.2, 0.55, fc=colr if colr != BLK else "#616161",
                           ec="none", zorder=6.5))
    T(COL[c], R1 + 0.45, lab, size=4.2, ha="center", c="white", z=9)

# Bluefruit board
box(3.5, 58.0, 27.4, 25.0, "#1a237e", ec="#000051", r=0.8, z=2)
for c, lab in enumerate(("VIN", "RST", "GND", "DFU", "IRQ", "CS", "MOSI", "MISO", "SCK")):
    nc = lab in ("RST", "DFU", "IRQ")
    T(COL[c], 81.4, lab, size=4.4, ha="center", c="#9fa8da" if nc else "white", z=9)
    ax.plot([COL[c], COL[c]], [83.0, R3], color="#9e9e9e", lw=1.2, zorder=2.5)
for c in (1, 3, 4):
    T(COL[c], 79.6, "nc", size=4.2, ha="center", c="#9fa8da", z=9)
T(17.2, 73.5, "Adafruit Bluefruit LE", size=6.6, ha="center", c="white", w="bold")
T(17.2, 71.3, "SPI Friend", size=6.6, ha="center", c="white", w="bold")
T(17.2, 67.6, "VIN takes 3.3–16 V: 3.3 V is fine", size=5.4, ha="center", c="#c5cae9")
T(17.2, 65.6, "RST, DFU, IRQ: not connected", size=5.4, ha="center", c="#c5cae9")
T(17.2, 63.6, "(firmware polls, no IRQ)", size=5.4, ha="center", c="#c5cae9")
tag(28.6, 61.0, 6)

# ---------------------------------------------------------------- J12 header
box(39.5, 60.4, 13.0, 54.6, "#2e7d32", ec="#1b5e20", r=1.2, z=1)
T(46.0, 113.5, "PIC32 kit", size=7.0, ha="center", c="white", w="bold")
T(46.0, 112.0, "J12 (back side)", size=5.6, ha="center", c="#c8e6c9")
box(41.3, 62.0, 9.4, 49.6, "#111111", ec="#000000", r=0.5, z=2)
XO, XE = 43.2, 48.8


def yrow(k):
    return 110.0 - 2.45 * k


PIN = {}
for k in range(20):
    o, e = 2 * k + 1, 2 * k + 2
    PIN[o] = (XO, yrow(k))
    PIN[e] = (XE, yrow(k))
    hpin(XO, yrow(k))
    hpin(XE, yrow(k))
    T(44.35, yrow(k), str(o), size=5.2, c="#bdbdbd", z=8)
    T(47.65, yrow(k), str(e), size=5.2, c="#bdbdbd", ha="right", z=8)
ax.add_patch(Rectangle((XO - 0.95, yrow(0) - 0.95), 1.9, 1.9, fill=False, ec="#ffffff", lw=1.2, zorder=8))
T(46.0, 61.2, "pin 1 = square pad", size=5.2, ha="center", c="white", z=8)

# ---------------------------------------------------------------- 7-wire cable (sleeve) hub -> J12
SLX0, SLX1 = 33.6, 38.4
box(COL[8] + 1.3, R1 - 1.2, SLX0 - COL[8] - 1.0, 2.4, "#9e9e9e", ec="#616161", r=0.5, z=3)
box(SLX0, 63.0, SLX1 - SLX0, 49.0, "#9e9e9e", ec="#616161", r=1.0, z=3)
T((SLX0 + SLX1) / 2, 74.0, "7-wire cable (dupont)", size=6.0, ha="center", c="white", w="bold", rotation=90, z=9)
DECK = [(1, PNK, "3V3"), (6, BLK, "GND"), (7, "#2e7d32", "MISO"), (13, PUR, "CS BLE"),
        (16, "#9e9e9e", "CS IMU"), (23, YEL, "SCK"), (37, BLU, "MOSI")]
for p, c, lab in DECK:
    x, y = PIN[p]
    if p % 2 == 1:
        pts = [(SLX1 - 0.2, y), (x, y)]
    else:
        pts = [(SLX1 - 0.2, y + 1.22), (x - 1.2, y + 1.22), (x, y)]
    W(pts, c, lw=1.5, z=4)
    ax.add_patch(Circle((x, y), 0.45, fc=c, ec="#424242", lw=0.6, zorder=8.5))

# ---------------------------------------------------------------- ESC servo leads (right of J12)
ESC = [(11, WHT, "11  ESC 1 white  (front-left)"), (12, WHT, "12  ESC 2 white  (front-right)"),
       (14, BLK, "14  ESC 1 black"), (19, WHT, "19  ESC 4 white  (back-right)"), (20, BLK, "20  ESC 2 black"),
       (21, WHT, "21  ESC 3 white  (back-left)"), (25, BLK, "25  ESC 3 black"), (30, BLK, "30  ESC 4 black")]
for p, c, lab in ESC:
    x, y = PIN[p]
    if p % 2 == 0:
        pts = [(x, y), (53.6, y)]
        ly = y
    else:
        ly = y + 1.22
        pts = [(x, y), (x + 1.2, ly), (53.6, ly)]
    W(pts, c, lw=1.5, z=4)
    ax.add_patch(Circle((x, y), 0.45, fc=c if c != WHT else "#bdbdbd", ec="#424242", lw=0.6, zorder=8.5))
    T(54.2, ly, lab, size=5.3)
T(54.2, 104.6, "ESC servo leads", size=6.4, w="bold")
T(54.2, 102.6, "(red wire taped back)", size=5.6, c="#555555")

# ---------------------------------------------------------------- J12 -> power board (board drawn in ax2)
BP5, BPR, BPG = (72.8, 61.0), (72.8, 49.0), (72.8, 35.0)
W([PIN[2], (68.5, 110.0), (68.5, BP5[1]), BP5], ORG, lw=2.0, z=5)
W([PIN[33], (XO + 1.2, yrow(16) + 1.22), (66.0, yrow(16) + 1.22), (66.0, BPR[1]), BPR], TEAL, lw=2.0, z=5)
W([PIN[34], (64.0, yrow(16)), (64.0, BPG[1]), BPG], BLK, lw=2.0, z=5)
ax.add_patch(Circle(PIN[2], 0.45, fc=ORG, ec="#424242", lw=0.6, zorder=8.5))
ax.add_patch(Circle(PIN[33], 0.45, fc=TEAL, ec="#424242", lw=0.6, zorder=8.5))
ax.add_patch(Circle(PIN[34], 0.45, fc=BLK, ec="#424242", lw=0.6, zorder=8.5))
T(69.3, 104.0, "pin 2  +5V_EXT in", size=6.2, c=ORG, w="bold", rotation=90)
T(66.7, 63.6, "pin 33 RK4", size=6.0, c=TEAL, w="bold", rotation=90)
T(63.3, 52.0, "pin 34 GND", size=6.0, w="bold", rotation=90, ha="right")

# ---------------------------------------------------------------- notes
notes = [
    (1, "10 A relay: bench / low throttle only. Hover ≈ 4 × 4.4 A = 17.6 A, so fly with a 30 A module (same wiring)."),
    (2, "7805 burns ≈ 1.9 W on a full pack: heatsink on the tab, keep it off the PLA."),
    (3, "VREF on +5 V makes CH1 active-LOW; Q1 pulls it down. RK4 HIGH = motors powered, reset / RK4 LOW = cut."),
    (4, "Two board wires cross others (7805 GND → GND rail, Q1 C → CH1): use insulated wire for those two."),
    (5, "ESC leads by position (front = opposite the battery pads, white arms left): white → signal pin, black → GND."),
    (6, "IMU and Bluefruit run from J12 pin 1 (kit 3.3 V), so a kill never turns them off. Never put 5 V on the IMU."),
]
y = 52.0
for n, s in notes:
    tag(3.6, y, n)
    T(6.0, y, s, size=6.3)
    y -= 3.9
T(2.0, 26.6, "Only solder off the boards: the plate + and – pads (battery leads and the thin GND tap).", size=6.3,
  c="#333333")
T(2.0, 23.6, "Before the battery: USB only, meter J12 pin 1 → pin 6 = 3.3 V (finds pin 1). + and – pads not "
  "shorted. Relay NC empty.", size=6.3, c="#333333")

# ---------------------------------------------------------------- legend (under the relay)
lx, ly = 120.0, 45.5
T(lx, ly, "Wires", size=7.6, w="bold")
LEG = [(RED, "battery + (always live)", 4.2), ("#b71c1c", "switched + (motors only)", 4.2), (BLK, "ground", 2.2),
       (ORG, "+5 V", 2.2), (TEAL, "RK4 relay control", 2.2), (GRN, "Q1 collector → CH1", 2.2),
       (PNK, "3.3 V to the deck", 2.0), (BLU, "MOSI", 2.0), (YEL, "SCK", 2.0), ("#2e7d32", "MISO", 2.0),
       ("#9e9e9e", "CS IMU", 2.0), (PUR, "CS BLE", 2.0), (WHT, "ESC signal", 2.0)]
for i, (c, lab, lw) in enumerate(LEG):
    yy = ly - 3.0 - i * 2.35
    W([(lx, yy), (lx + 4, yy)], c, lw=lw, z=4)
    T(lx + 5.0, yy, lab, size=6.0)
T(198.0, 1.6, "Ethan DiMeo, ELC275-501, Nov 19 2026", size=6.0, c="#757575", ha="right")

for ext in ("png", "pdf"):
    fig.savefig(f"complete_wiring.{ext}", dpi=150)
print("saved")
