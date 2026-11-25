# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/landing_leg/v1_CURRENT/src/drawing.py
#  Part    : landing_leg
#  What    : Makes the dimensioned drawing (PNG + PDF).
#  FUNCTIONS: dim_v(), dim_h()
# ==============================================================================================

# Dimensioned drawing of the replacement leg (side view, bracket from above, leg section).
# Run:  python drawing.py OUTPUT_DIR
import sys, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP, Circle
from shapely.geometry import Polygon
import f450_leg as L
k = L.solve_k()
FL, WB = max(L.MIN_WALL, L.FLANGE * k), max(L.MIN_WALL, L.WEB * k)
out = sys.argv[1]
fig = plt.figure(figsize=(16, 10.5))
ax = fig.add_axes([0.03, 0.05, 0.62, 0.86]); ax.set_aspect("equal"); ax.axis("off")
# ---- side view (u across, z up): bracket + gusset + leg outline
prof = L.profile_pts()
ax.add_patch(MP([(L.BR_NOSE_U, 0), (L.BR_OUT_U, 0), (L.BR_OUT_U, -L.BR_T), (L.BR_NOSE_U, -L.BR_T)], fc="#f39c4a", ec="k", lw=1.2))
ax.add_patch(MP([(L.BR_OUT_U, 0), (L.BR_OUT_U, -L.BR_T), (L.BR_OUT_U + L.BR_T, -L.BR_T)], fc="#f39c4a", ec="k", lw=1.2))
ax.add_patch(MP([(L.BR_NOSE_U, -L.BR_T), (L.SCREW_U + 4.5, -L.DROP), (L.TOP_OUT_U, -L.DROP), (L.TOP_OUT_U, -L.BR_T)], fc="#e8822c", ec="k", lw=0.8))
toe_u = L.edge_u(-L.HEIGHT, True)
pts = prof[:3] + [(toe_u - L.FOOT_R * 1.2, -L.HEIGHT), (toe_u, -L.HEIGHT + L.FOOT_R * 1.2)] + prof[4:]
ax.add_patch(MP(pts, fc="#f7a35c", ec="k", lw=1.5))
# pockets (dashed)
shr = Polygon(prof).buffer(-FL, join_style=2)
ax.plot([L.SCREW_U] * 2, [3, -L.BR_T - 3], "k-.", lw=0.6)
ax.plot([0, 225], [-L.HEIGHT] * 2, color="#777", lw=2); ax.text(0, -L.HEIGHT - 6, "ground", color="#555")
ax.plot([0, 70], [0, 0], color="#555", lw=3); ax.text(0, 3, "bottom plate (underside = 0)", color="#555")
def dim_v(x, z0, z1, txt):
    ax.annotate("", (x, z0), (x, z1), arrowprops=dict(arrowstyle="<->", lw=1)); ax.text(x + 2, (z0 + z1) / 2, txt, va="center", fontsize=11)
def dim_h(z, u0, u1, txt, dy=-6):
    ax.annotate("", (u0, z), (u1, z), arrowprops=dict(arrowstyle="<->", lw=1)); ax.text((u0 + u1) / 2, z + dy, txt, ha="center", fontsize=11)
dim_v(20, 0, -L.HEIGHT, f"{L.HEIGHT:.0f} plate to ground")
dim_v(L.BR_NOSE_U - 6, 0, -L.BR_T, f"{L.BR_T}")
dim_h(8, L.BR_NOSE_U, L.BR_OUT_U, "23.2 bracket", dy=3)
dim_h(-L.HEIGHT - 14, L.SCREW_U, toe_u, f"{toe_u - L.SCREW_U:.0f} screw line to toe")
dim_h(-L.HEIGHT + 8, prof[2][0], prof[3][0], f"{prof[3][0] - prof[2][0]:.1f} foot")
ax.annotate("", (L.SCREW_U - 26, -L.BR_T), (L.SCREW_U - 26, -L.DROP), arrowprops=dict(arrowstyle="<->", lw=1))
ax.text(L.SCREW_U - 47, -L.DROP + 2, f"leg drops\nto {L.DROP:.0f}", fontsize=10)
ang = math.degrees(math.atan(L.SLOPE))
ax.text(140, -45, f"leans out {ang:.1f}° from vertical\n({L.SLOPE} mm out per mm down)\nbeam {L.DEPTH:.0f} deep (square to the leg)\nI-beam faces {FL:.2f}, web {WB:.2f}", fontsize=11)
ax.text(L.SCREW_U + 2, 4, "2 arm screws", fontsize=10)
L_total = L.DROP + (L.HEIGHT - L.DROP) * math.sqrt(1 + L.SLOPE ** 2)
ax.set_xlim(-5, 235); ax.set_ylim(-178, 22)
ax.set_title(f"SIDE VIEW along the arm (mm)  -  length along the leg about {L_total:.0f}", fontsize=13, loc="left")
# ---- bracket from above
b = fig.add_axes([0.66, 0.50, 0.32, 0.40]); b.set_aspect("equal"); b.axis("off")
foot = [(L.BR_NOSE_U, -L.BR_NOSE_HS), (L.BR_CORNER_U, -L.BR_HS), (L.BR_OUT_U, -L.BR_HS), (L.BR_OUT_U, L.BR_HS), (L.BR_CORNER_U, L.BR_HS), (L.BR_NOSE_U, L.BR_NOSE_HS)]
b.add_patch(MP(foot, fc="#f39c4a", ec="k", lw=1.2))
b.add_patch(MP([(L.ROOT_IN_U, -L.LEG_HS), (L.BR_OUT_U + L.BR_T, -L.LEG_HS), (L.BR_OUT_U + L.BR_T, L.LEG_HS), (L.ROOT_IN_U, L.LEG_HS)], fc="none", ec="k", ls="--"))
for s in (-L.SCREW_S, L.SCREW_S):
    b.add_patch(Circle((L.SCREW_U, s), L.HOLE_D / 2, fc="w", ec="k"))
b.annotate("", (L.BR_OUT_U + L.BR_T + 4, -L.BR_HS), (L.BR_OUT_U + L.BR_T + 4, L.BR_HS), arrowprops=dict(arrowstyle="<->")); b.text(L.BR_OUT_U + L.BR_T + 5, 0, "28.5", va="center")
b.plot([L.BR_OUT_U, L.BR_OUT_U + L.BR_T + 6], [L.BR_HS] * 2, "k-", lw=0.4); b.plot([L.BR_OUT_U, L.BR_OUT_U + L.BR_T + 6], [-L.BR_HS] * 2, "k-", lw=0.4)
b.annotate("", (L.SCREW_U, -L.BR_HS - 4), (L.SCREW_U, L.BR_HS + 4), arrowprops=dict(arrowstyle="-", lw=0.4))
b.text(L.SCREW_U - 15, L.BR_HS + 6, f"holes Ø{L.HOLE_D}, {2 * L.SCREW_S:.1f} apart", fontsize=10)
b.text(L.BR_NOSE_U - 1, -L.BR_HS - 8, "nose (toward frame center) 10.6 flat,\n18.5 in from the screw line", fontsize=9)
b.text(L.BR_OUT_U + 1, -L.LEG_HS - 4.5, "leg 14.9 wide\n(dashed)", fontsize=9)
b.set_xlim(L.BR_NOSE_U - 8, L.BR_OUT_U + L.BR_T + 16); b.set_ylim(-L.BR_HS - 16, L.BR_HS + 12)
b.set_title("BRACKET from above (your caliper numbers)", fontsize=12, loc="left")
# ---- leg section
c = fig.add_axes([0.70, 0.08, 0.22, 0.36]); c.set_aspect("equal"); c.axis("off")
D, W = L.DEPTH, 2 * L.LEG_HS
c.add_patch(MP([(0, 0), (W, 0), (W, FL), (W / 2 + WB / 2, FL), (W / 2 + WB / 2, D - FL), (W, D - FL), (W, D), (0, D), (0, D - FL),
                (W / 2 - WB / 2, D - FL), (W / 2 - WB / 2, FL), (0, FL)], fc="#f7a35c", ec="k"))
c.text(W / 2, -2.5, "14.9 wide", ha="center"); c.text(W + 1, D / 2, f"{D:.0f} deep", va="center")
c.set_xlim(-3, W + 10); c.set_ylim(-5, D + 4)
c.set_title("LEG SECTION (I-beam)", fontsize=12, loc="left")
g = round(L.TARGET_G, 1)
fig.text(0.03, 0.955, f"ELC275 F450 replacement landing leg  -  Ethan DiMeo  -  2026-11-25  -  PLA at 100 % infill about {g} g (target: stock leg, about {g} g)", fontsize=14, weight="bold")
fig.savefig(f"{out}/leg_drawing.png", dpi=110); fig.savefig(f"{out}/leg_drawing.pdf")
print("ok")
