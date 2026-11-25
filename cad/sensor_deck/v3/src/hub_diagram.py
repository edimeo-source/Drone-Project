# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/sensor_deck/v3/src/hub_diagram.py
#  Part    : sensor_deck
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: uv()
# ==============================================================================================

# Hub strip wiring diagram (top view, FRONT = up). Run: python hub_diagram.py OUT.png
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sensor_deck_v3 as D
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch

def uv(c, r):            # plot coords: left = drone left (+Y), up = front (+X)
    return -(D.ROW_IMU + r * D.PITCH), D.COL0_X - c * D.PITCH

fig, ax = plt.subplots(figsize=(13, 10.5))
ax.set_aspect("equal"); ax.axis("off")
# boards (outlines)
ax.add_patch(Rectangle((-12.7, -12.7), 25.4, 25.4, fc="#f7d9d9", ec="#c0392b", lw=2))          # IMU (only its left part matters)
ax.text(0, -8.5, "IMU (LSM6DSO)\nX arrow -> FRONT", ha="center", va="center", color="#c0392b", fontsize=12, weight="bold")
ax.add_patch(Rectangle((-D.BLE_Y1, D.BLE_XC - D.BLE_W / 2), D.BLE_L, D.BLE_W, fc="#dbe4f7", ec="#1f4fbf", lw=2))
ax.text(-34, D.BLE_XC, "Bluefruit\nSPI Friend\n(module side up,\nantenna end out)", ha="center", va="center", color="#1f4fbf", fontsize=12, weight="bold")
ax.add_patch(Rectangle((-D.HUB_Y1, D.HUB_X0), D.HUB_Y1 - D.HUB_Y0, D.HUB_X1 - D.HUB_X0, fc="#cfe8d4", ec="#1e7a3c", lw=2.5, zorder=2))
ax.text(-(D.HUB_Y0 + D.HUB_Y1) / 2, D.HUB_X1 + 1.6, "HUB STRIP\n9 x 4 holes (23 x 10 mm)", ha="center", va="bottom", color="#1e7a3c", fontsize=11, weight="bold")
# pads
for c in range(9):
    for r in range(4):
        u, v = uv(c, r)
        ax.add_patch(plt.Circle((u, v), 0.75, fc="#e8c46a", ec="#8a6d1f", lw=0.8, zorder=3))
# solder bridges
for (a, b) in D.BRIDGES:
    (u0, v0), (u1, v1) = uv(*a), uv(*b)
    ax.plot([u0, u1], [v0, v1], color="#7f8c8d", lw=7, solid_capstyle="round", zorder=4)
# wire links
cols = {"3V3": "#e74c3c", "MOSI": "#8e44ad", "SCK": "#f39c12", "MISO": "#16a085", "CS BLE": "#2c3e50"}
for (a, b, name) in D.LINKS:
    (u0, v0), (u1, v1) = uv(*a), uv(*b)
    ax.add_patch(FancyArrowPatch((u0, v0), (u1, v1), connectionstyle="arc3,rad=0.25", arrowstyle="-",
                                 color=cols[name], lw=3.2, zorder=5))
# pins and labels
for c, n in D.IMU_PINS.items():
    u, v = uv(c, 0)
    ax.add_patch(Rectangle((u - 0.55, v - 0.55), 1.1, 1.1, fc="#c0392b", zorder=6))
    ax.text(u + 1.4, v, n, va="center", ha="left", fontsize=10.5, color="#c0392b", weight="bold", zorder=7)
for c, n in D.BLE_PINS.items():
    u, v = uv(c, 3)
    ax.add_patch(Rectangle((u - 0.55, v - 0.55), 1.1, 1.1, fc="#1f4fbf", zorder=6))
    ax.text(u - 1.4, v, n, va="center", ha="right", fontsize=10.5, color="#1f4fbf", weight="bold", zorder=7)
for c, (n, j) in D.OUT_PINS.items():
    u, v = uv(c, D.OUT_ROW)
    ax.add_patch(Rectangle((u - 0.6, v - 0.6), 1.2, 1.2, fc="black", zorder=6))
for c, (n, j) in D.OUT_PINS.items():          # cable labels, out to the right past the IMU
    u, v = uv(c, D.OUT_ROW)
    ax.plot([5.2, 15.0], [v, v], color="#999", lw=0.8, ls=":", zorder=1)
    ax.text(15.5, v, f"cable {n:6s} -> J12 pin {j}", fontsize=10.5, va="center", family="monospace")
# legend + notes
ax.plot([], [], color="#7f8c8d", lw=7, label="solder bridge (pad to pad)")
for n, cc in cols.items(): ax.plot([], [], color=cc, lw=3.2, label=f"wire link: {n}")
ax.add_patch(Rectangle((0, 0), 0, 0, fc="black", label="7-pin cable header (plastic UNDER the strip, pins up)"))
ax.legend(loc="upper center", bbox_to_anchor=(0.5, 0.02), fontsize=10, frameon=True, ncol=3)
ax.annotate("", xy=(-44, 22), xytext=(-44, 12), arrowprops=dict(arrowstyle="-|>", lw=2.5, color="black"))
ax.text(-44, 23, "FRONT", ha="center", fontsize=12, weight="bold")
ax.text(-38, 26.5, "Top view, FRONT = up.  Rows from the IMU out: IMU pins / cable header / free / Bluefruit pins.\n"
        "Bluefruit RST, DFU, IRQ: not connected (the firmware polls).  Everything is soldered on the TOP of the strip.",
        fontsize=10.5, va="top")
ax.set_xlim(-50, 46); ax.set_ylim(-14, 28)
plt.savefig(sys.argv[1] if len(sys.argv) > 1 else "hub_wiring.png", dpi=150, bbox_inches="tight")
print("ok")
