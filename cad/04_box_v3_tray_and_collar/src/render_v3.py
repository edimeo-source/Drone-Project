# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/04_box_v3_tray_and_collar/src/render_v3.py
#  Part    : 04_box_v3_tray_and_collar
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), edge_actor(), shot()
# ==============================================================================================

import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B

out = sys.argv[1]; os.makedirs(out, exist_ok=True)
shell = B.make_shell()
tray, collar = B.make_tray(shell), B.make_collar(shell)
plate, legs = B.make_plate(), B.make_legs_estimate()
P = B.make_parts()
kit = P["pcb"].union(P["kit_top"]).union(P["kit_bot"])
EXPL = 34.0

S = {"plate": (plate, (0.17, 0.17, 0.17), 1.0), "plate_t": (plate, (0.17, 0.17, 0.17), 0.35),
     "tray": (tray, (0.86, 0.16, 0.12), 1.0), "collar": (collar, (0.93, 0.93, 0.93), 1.0),
     "collar_t": (collar, (0.93, 0.93, 0.93), 0.45),
     "collar_x": (collar.translate((0, 0, EXPL)), (0.93, 0.93, 0.93), 1.0),
     "collar_g": (collar, (0.72, 0.72, 0.76), 1.0),
     "legs": (legs, (0.05, 0.05, 0.05), 1.0), "kit": (kit, (0.12, 0.48, 0.23), 1.0),
     "jump": (P["jumpers"], (0.25, 0.25, 0.25), 0.6), "screw": (P["screw"], (0.6, 0.6, 0.65), 1.0),
     "plugs": (P["plugs"], (0.15, 0.15, 0.18), 1.0), "pwr": (P["pwr"], (0.92, 0.45, 0.1), 0.85),
     "xt60": (P["xt60"].union(P["xt60_plug"]), (0.95, 0.8, 0.1), 1.0), "bat": (P["bat"], (0.83, 0.63, 0.09), 1.0),
     "jts": (P["jt_screws"], (0.6, 0.6, 0.65), 1.0)}
# print bed: tray and collar as they sit on the K2 Pro plate (300 x 300)
bed = cq.Workplane("XY").box(300, 300, 1).translate((0, 0, -0.5))
t_p = tray.translate((0, 0, -B.Z_BOTTOM)).translate((0, -70, 0))
c_p = collar.translate((0, 0, -B.Z_SPLIT)).translate((0, 72, 0))
S["bed"] = (bed, (0.35, 0.35, 0.38), 1.0); S["t_p"] = (t_p, (0.86, 0.16, 0.12), 1.0); S["c_p"] = (c_p, (0.93, 0.93, 0.93), 1.0)


def actor(n):
    s, c, a = S[n]; p = f"/tmp/_b3_{n}.stl"; cq.exporters.export(s, p, tolerance=0.05, angularTolerance=0.1)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetOpacity(a); pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25); return ac


A = {k: actor(k) for k in S}


def edge_actor(n, color=(0, 0, 0), width=2.0):
    r = vtk.vtkSTLReader(); r.SetFileName(f"/tmp/_b3_{n}.stl")
    fe = vtk.vtkFeatureEdges(); fe.SetInputConnection(r.GetOutputPort()); fe.BoundaryEdgesOn(); fe.FeatureEdgesOn()
    fe.SetFeatureAngle(30); fe.ManifoldEdgesOff(); fe.NonManifoldEdgesOff()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(fe.GetOutputPort()); m.ScalarVisibilityOff()
    a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*color); a.GetProperty().SetLineWidth(width); return a


A["plate_edges"] = edge_actor("plate", (0.05, 0.05, 0.6), 2.5)


def shot(f, names, pos, foc, title, zoom=1.25, up=(0, 0, 1), ortho=False):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up)
    if ortho: c.ParallelProjectionOn()
    r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()


inside = ["tray", "kit", "jump", "screw", "pwr", "xt60"]
shot("1_parts_exploded", ["tray", "collar_x", "jts"], (230, -260, 170), (0, 0, -15),
     "V3 = 2 parts: red TRAY (kit + power board) and white COLLAR (bolts to the frame), 4x M3 x 10 at the corners")
shot("2_tray_with_kit", inside, (150, -210, 230), (-20, 0, -35),
     "Kit right side up, flat on its cradle (screw + crush ribs). J12 on top at the right end of the kit")
shot("3_cradle_closeup", ["tray"], (-60, -120, 120), (-25, 0, -42),
     "Cradle: fence + 6 crush ribs, screw boss, corner pad, J1 pedestal, rubber-foot pedestal, 5 mm relief", zoom=2.0)
shot("4_left_end_ports", ["tray", "collar", "kit", "screw"], (-300, -70, 25), (-60, 0, -30),
     "Left end: window A = USB-A J5 + mini-USB + micro-USB J4 (under the board), window B = 2nd mini-USB", zoom=1.7)
shot("5_under_frame_legs", ["plate", "legs", "tray", "collar", "bat", "xt60"], (260, -330, -200), (0, 0, -25),
     "Under the frame (black legs = traced from your photos): collar walls step in around every leg")
shot("6_leg_corner_closeup", ["plate", "legs", "tray", "collar"], (150, -150, -60), (45, -38, -12),
     "Leg corner: the collar wraps around the leg bracket and its curve, the tray only starts 22 mm down", zoom=2.2)
shot("7_top_through_plate", ["tray", "collar_g", "kit", "pwr", "legs", "plate_edges"], (0, 0, 600), (0, 0, 0),
     "Top view, FRONT = up (blue = plate outline, black = leg brackets): wire bays sit right under the edge openings",
     zoom=1.25, up=(0, 1, 0), ortho=True)
shot("8_print_plate", ["bed", "t_p", "c_p"], (260, -300, 330), (0, 0, 10),
     "K2 Pro bed (300 x 300): tray floor-down, collar joint-face down, both in one print, no supports")
print("ok")
