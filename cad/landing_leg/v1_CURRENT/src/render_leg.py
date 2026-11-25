# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/landing_leg/v1_CURRENT/src/render_leg.py
#  Part    : landing_leg
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

# Pictures of the replacement leg: alone, on the drone (with the box), and on the print bed.
# Run:  python render_leg.py OUTPUT_DIR
import sys, os, math, vtk
import cadquery as cq
HERE = os.path.dirname(os.path.abspath(__file__))
CAD = next(c for c in (os.path.join(HERE, "../../../06_box_v4_1_tray_CURRENT/src"),
                       "/home/claude/Drone-Project/cad/06_box_v4_1_tray_CURRENT/src") if os.path.isdir(c))
sys.path.insert(0, CAD); here = os.getcwd(); os.chdir(CAD)
import box_v3 as B, collar_v3_1 as C, tray_v4_1 as T
os.chdir(here); sys.path.insert(0, HERE)
import f450_leg as L
out = os.path.abspath(sys.argv[1]); os.makedirs(out, exist_ok=True)
leg = L.make_leg(L.solve_k())
at = lambda sx, sy, p: p.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(sy, sx)))
new = at(-1, 1, leg)                                        # front-left in the box frame (+Y front, +X right)
others = at(1, 1, leg).union(at(1, -1, leg)).union(at(-1, -1, leg))
os.chdir(CAD)
S = {"leg": (leg, (0.95, 0.45, 0.08)), "new": (new, (0.95, 0.45, 0.08)), "others": (others, (0.12, 0.12, 0.12)),
     "plate": (B.make_plate(), (0.25, 0.25, 0.25)), "collar": (C.make_collar_v31(), (0.15, 0.35, 0.85)),
     "tray": (T.make_tray(), (0.86, 0.16, 0.12)),
     "bedleg": (leg.rotate((0, 0, 0), (1, 0, 0), 180).translate((-110, 0, 0)), (0.95, 0.45, 0.08)),
     "bed": (cq.Workplane("XY").box(300, 300, 2).translate((0, 0, -1)), (0.82, 0.82, 0.85))}
os.chdir(here)

def actor(n):
    s, c = S[n]; p = f"/tmp/_leg_{n}.stl"; cq.exporters.export(s, p, tolerance=0.03, angularTolerance=0.1)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); a = vtk.vtkActor(); a.SetMapper(m)
    pr = a.GetProperty(); pr.SetColor(*c); pr.SetSpecular(0.2); pr.SetDiffuse(0.85); pr.SetAmbient(0.25); return a
A = {k: actor(k) for k in S}

def shot(f, names, pos, foc, title, zoom=1.2, up=(0, 0, 1), ortho=False):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up)
    if ortho: c.ParallelProjectionOn()
    r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()

shot("1_leg_iso", ["leg"], (230, -200, 40), (110, 0, -70), "Replacement F450 leg: measured bracket, I-beam leg, 150 mm tall, about 20.8 g in PLA")
shot("2_leg_underside", ["leg"], (60, -90, -160), (90, 0, -60), "From below: hollow bracket underside, 2 screw holes, I-beam pockets, rounded toe", zoom=1.3)
shot("3_on_drone", ["plate", "collar", "tray", "others", "new"], (-330, 330, -60), (-40, 40, -60),
     "On the drone (orange = new leg, front-left): clears collar v3.1, the plate and tray v4.1", zoom=1.15)
shot("4_print_bed", ["bed", "bedleg"], (150, -260, 220), (-40, 0, 60), "Print: bracket face down on the bed, no supports (every overhang 44 deg or steeper)")
print("ok")
