# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/06_box_v4_1_tray/src/render_v4_1.py
#  Part    : 06_box_v4_1_tray
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

# Pictures of tray V4.1 (relay in the back wire bay, battery-lead slot) - Ethan DiMeo
import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B, collar_v3_1 as C, tray_v4_1 as T

out = sys.argv[1]; os.makedirs(out, exist_ok=True)
tray, collar = T.make_tray(), C.make_collar_v31()
plate, legs = B.make_plate(), C.make_legs_measured()
P, Q = B.make_parts(), T.make_parts()
kit = P["pcb"].union(P["kit_top"]).union(P["kit_bot"])
cut_lobe = B.box(-40, 40, -70, -30, -100, 10)          # for the back-lobe close-up
S = {"tray": (tray, (0.16, 0.36, 0.86), 1.0),
     "tray_cut": (tray.cut(B.box(-0.5, 60, -70, 60, -100, 10)), (0.16, 0.36, 0.86), 1.0),
     "collar": (collar, (0.93, 0.93, 0.93), 1.0), "collar_t": (collar, (0.93, 0.93, 0.93), 0.35),
     "plate": (plate, (0.17, 0.17, 0.17), 1.0), "plate_t": (plate, (0.17, 0.17, 0.17), 0.45),
     "legs": (legs, (0.05, 0.05, 0.05), 1.0),
     "kit": (kit, (0.12, 0.48, 0.23), 1.0), "jump": (P["jumpers"], (0.25, 0.25, 0.25), 0.7),
     "plugs": (P["plugs"], (0.15, 0.15, 0.18), 1.0), "bat": (P["bat"], (0.83, 0.63, 0.09), 1.0),
     "pb": (Q["pb"], (0.10, 0.10, 0.10), 1.0), "pb_parts": (Q["pb_parts"], (0.80, 0.15, 0.15), 0.9),
     "usb": (Q["usb_plug"], (0.35, 0.35, 0.40), 1.0),
     "relay": (Q["relay"], (0.85, 0.12, 0.10), 1.0), "relay_parts": (Q["relay_parts"], (0.20, 0.45, 0.85), 1.0),
     "rscrews": (Q["relay_screws"], (0.6, 0.6, 0.65), 1.0),
     "wires": (Q["wires"], (0.95, 0.10, 0.10), 1.0), "bullets": (Q["bullets"], (0.85, 0.70, 0.20), 1.0)}
bed = cq.Workplane("XY").box(300, 300, 1).translate((0, 0, -0.5))
S["bed"] = (bed, (0.35, 0.35, 0.38), 1.0)
S["t_p"] = (tray.translate((0, 0, -B.Z_BOTTOM)), (0.16, 0.36, 0.86), 1.0)


def actor(n):
    s, c, a = S[n]; p = f"/tmp/_b41_{n}.stl"; cq.exporters.export(s, p, tolerance=0.05, angularTolerance=0.1)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetOpacity(a); pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25); return ac


A = {k: actor(k) for k in S}


def shot(f, names, pos, foc, title, zoom=1.25, up=(0, 0, 1), ortho=False):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up)
    if ortho: c.ParallelProjectionOn()
    r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()


inside = ["kit", "jump", "plugs", "pb", "pb_parts", "usb", "relay", "relay_parts", "rscrews", "wires", "bullets"]
shot("1_tray_v4_1_loaded", ["tray"] + inside, (200, -300, 200), (10, -10, -50),
     "Tray V4.1: relay on edge in the back wire bay, leads (red) out the back slot to the bullets (gold)")
shot("2_top_view", ["tray"] + inside, (0, 0, 600), (5, 0, -50),
     "Top view, FRONT = up: kit, proto board (right bay), relay in the back bay right under the battery pads",
     zoom=1.3, up=(0, 1, 0), ortho=True)
shot("3_back_view_assembled", ["plate", "collar", "tray", "legs", "wires", "bullets", "bat", "usb"], (20, -420, -40), (5, -50, -45),
     "From the back: both bullets hang out of the slot just under the plate's back edge; plug the battery in here", zoom=1.5)
shot("4_back_bay_cutaway", ["tray_cut", "relay", "relay_parts", "rscrews", "wires", "bullets", "kit", "jump"],
     (170, -60, 40), (0, -45, -50),
     "Back bay cut open: relay on 4 sideways posts, COM/NO on top, + lead over the top edge and out the U-slot", zoom=1.8)
shot("5_slot_closeup", ["tray"], (40, -200, 10), (1, -56, -32),
     "U-slot in the back wall (11 wide, open at the top): leads and bullets drop in from above", zoom=2.4)
shot("6_under_frame", ["plate", "legs", "collar", "tray", "bat", "bullets", "usb"], (280, -330, -200), (10, 0, -30),
     "Under the frame: collar v3.1 unchanged, tray V4.1, bullets at the back, legs clear")
shot("7_print_plate", ["bed", "t_p"], (260, -300, 330), (0, 0, 20),
     "K2 Pro bed (300 x 300): tray V4.1 floor-down, no supports")
print("ok")
