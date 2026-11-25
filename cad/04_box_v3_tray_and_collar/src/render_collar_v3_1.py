# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/04_box_v3_tray_and_collar/src/render_collar_v3_1.py
#  Part    : 04_box_v3_tray_and_collar
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), edges(), shot()
# ==============================================================================================

import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B
import collar_v3_1 as C

out = sys.argv[1]; os.makedirs(out, exist_ok=True)
collar = C.make_collar_v31()
tray = B.make_tray(B.make_shell())
legs = C.make_legs_measured()
plate = B.make_plate()
S = {"collar": (collar, (0.15, 0.35, 0.85), 1.0), "tray": (tray, (0.86, 0.16, 0.12), 1.0),
     "legs": (legs, (0.05, 0.05, 0.05), 1.0), "plate": (plate, (0.17, 0.17, 0.17), 1.0),
     "collar_x": (collar.translate((0, 0, 30)), (0.15, 0.35, 0.85), 1.0)}


def actor(n):
    s, c, a = S[n]; p = f"/tmp/_c31_{n}.stl"; cq.exporters.export(s, p, tolerance=0.05, angularTolerance=0.1)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetOpacity(a); pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25); return ac


def edges(n, color=(0.95, 0.75, 0.1), width=2.5):
    r = vtk.vtkSTLReader(); r.SetFileName(f"/tmp/_c31_{n}.stl")
    fe = vtk.vtkFeatureEdges(); fe.SetInputConnection(r.GetOutputPort()); fe.BoundaryEdgesOn(); fe.FeatureEdgesOn()
    fe.SetFeatureAngle(30); fe.ManifoldEdgesOff(); fe.NonManifoldEdgesOff()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(fe.GetOutputPort()); m.ScalarVisibilityOff()
    a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*color); a.GetProperty().SetLineWidth(width); return a


A = {k: actor(k) for k in S}
A["plate_edges"] = edges("plate")


def shot(f, names, pos, foc, title, zoom=1.25, up=(0, 0, 1), ortho=False):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up)
    if ortho: c.ParallelProjectionOn()
    r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()


shot("c1_collar_v3_1_iso", ["collar"], (200, -240, 210), (0, 0, -10),
     "Collar v3.1: a straight pocket around each leg bracket (floor at the joint), everything else the same")
shot("c2_view_from_plate", ["collar", "legs", "plate_edges"], (0, 0, 500), (0, 0, -10),
     "Looking down through the plate (yellow = plate outline, black = legs as measured)", zoom=1.3, up=(0, 1, 0), ortho=True)
shot("c3_under_frame", ["plate", "legs", "collar", "tray"], (240, -300, -170), (0, 0, -30),
     "Under the frame: collar v3.1 on the printed tray, legs as measured (bracket 23.2 x 28.5 x 7.3)")
shot("c4_leg_corner", ["plate", "legs", "collar"], (130, -120, -45), (40, -40, -8),
     "One corner: 1 mm gap around the bracket, its screw heads and the leg", zoom=2.3)
shot("c5_on_tray", ["tray", "collar_x"], (230, -260, 170), (0, 0, -30),
     "Same joint as before: 4 corner tabs + rim line up with the tray you already printed")
print("ok")
