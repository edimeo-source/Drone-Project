# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/03_box_v2/src/render_v2.py
#  Part    : 03_box_v2
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v2 as B
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
bx = B.make_box(); plate = B.make_plate(); Pp = B.make_parts(); heads = B.make_arm_screw_heads()
P = {"plate": (plate, (0.17, 0.17, 0.17), 1.0), "box": (bx, (0.93, 0.93, 0.93), 1.0), "heads": (heads, (0.55, 0.55, 0.6), 1.0),
     "kit": (Pp["kit"].union(Pp["usb"]).union(Pp["j12"]), (0.12, 0.48, 0.23), 1.0),
     "jump": (Pp["jumpers"], (0.25, 0.25, 0.25), 0.55), "screw": (Pp["screw"], (0.55, 0.55, 0.6), 1.0),
     "pwr": (Pp["pwr"], (0.92, 0.45, 0.1), 0.85), "xt60": (Pp["xt60"], (0.95, 0.8, 0.1), 1.0), "bat": (Pp["bat"], (0.83, 0.63, 0.09), 1.0)}
def actor(n):
    s, c, a = P[n]; p = f"/tmp/_b2_{n}.stl"; cq.exporters.export(s, p, tolerance=0.05, angularTolerance=0.1)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetOpacity(a); pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25); return ac
A = {k: actor(k) for k in P}
def shot(f, names, pos, foc, title, zoom=1.25, up=(0, 0, 1)):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up); r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()
inside = ["box", "kit", "jump", "screw", "pwr", "xt60"]
shot("1_box_v2_bare", ["box"], (210, -250, 230), (0, 0, -25), "Box V2: closed walls, center screw post + 4 side lips, power bay at the front, 4 arm-screw pockets")
shot("2_box_v2_with_parts", inside, (210, -250, 230), (0, 0, -25), "PIC32 at the rear (J12 jumpers hang under its front edge), orange = power/kill board 45 x 58")
shot("3_front", inside, (300, -150, 60), (0, 0, -25), "Front: XT60 window above the power board, 4 vent slots by the 7805", zoom=1.3)
shot("4_rear", inside, (-300, -150, 120), (0, 0, -25), "Rear: USB window (J3 program, J11 PuTTY/tuner, J4 power)", zoom=1.3)
shot("5_under_frame", ["plate", "heads", "box", "bat", "xt60"], (260, -330, -260), (0, 0, -20), "Under the plate: keys + 4 x M3 in the tab slots, battery strapped below")
shot("6_top_through_plate", ["plate", "heads", "box", "kit", "pwr"], (0, 0, 600), (0, 0, 0), "Top view (FRONT = up): wires go up through the plate's big front + rear center openings", zoom=1.3, up=(1, 0, 0))
print("ok")
