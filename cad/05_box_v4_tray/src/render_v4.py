# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/05_box_v4_tray/src/render_v4.py
#  Part    : 05_box_v4_tray
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

# Pictures of tray V4 with collar v3.1, the kit, the 7805 proto board and the relay. Ethan DiMeo
import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_v3 as B, collar_v3_1 as C, tray_v4 as T

out = sys.argv[1]; os.makedirs(out, exist_ok=True)
tray, collar = T.make_tray_v4(), C.make_collar_v31()
tray3 = B.make_tray(B.make_shell())
plate, legs = B.make_plate(), C.make_legs_measured()
P, Q = B.make_parts(), T.make_parts_v4()
kit = P["pcb"].union(P["kit_top"]).union(P["kit_bot"])

S = {"tray": (tray, (0.16, 0.36, 0.86), 1.0), "tray_t": (tray, (0.16, 0.36, 0.86), 0.35),
     "tray3_t": (tray3, (0.86, 0.16, 0.12), 0.30),
     "collar": (collar, (0.93, 0.93, 0.93), 1.0), "collar_t": (collar, (0.93, 0.93, 0.93), 0.40),
     "plate": (plate, (0.17, 0.17, 0.17), 1.0), "legs": (legs, (0.05, 0.05, 0.05), 1.0),
     "kit": (kit, (0.12, 0.48, 0.23), 1.0), "jump": (P["jumpers"], (0.25, 0.25, 0.25), 0.7),
     "plugs": (P["plugs"], (0.15, 0.15, 0.18), 1.0), "bat": (P["bat"], (0.83, 0.63, 0.09), 1.0),
     "pb": (Q["pb"], (0.10, 0.10, 0.10), 1.0), "pb_parts": (Q["pb_parts"], (0.80, 0.15, 0.15), 0.9),
     "usb": (Q["usb_plug"], (0.35, 0.35, 0.40), 1.0),
     "relay": (Q["relay"], (0.85, 0.12, 0.10), 1.0), "relay_parts": (Q["relay_parts"], (0.20, 0.45, 0.85), 1.0),
     "rscrews": (Q["relay_screws"], (0.6, 0.6, 0.65), 1.0),
     "xt60": (Q["xt60"].union(Q["xt60_plug"]), (0.95, 0.8, 0.1), 1.0)}
bed = cq.Workplane("XY").box(300, 300, 1).translate((0, 0, -0.5))
S["bed"] = (bed, (0.35, 0.35, 0.38), 1.0)
S["t_p"] = (tray.translate((0, 0, -B.Z_BOTTOM)), (0.16, 0.36, 0.86), 1.0)


def actor(n):
    s, c, a = S[n]; p = f"/tmp/_b4_{n}.stl"; cq.exporters.export(s, p, tolerance=0.05, angularTolerance=0.1)
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


boards = ["kit", "jump", "plugs", "pb", "pb_parts", "usb", "relay", "relay_parts", "rscrews", "xt60"]
shot("1_tray_v4_loaded", ["tray"] + boards, (230, -250, 220), (15, 0, -50),
     "Tray V4 (blue): kit left, 7805 proto board back-right, relay front-right, XT60 in the new end wall")
shot("2_top_view", ["tray"] + boards, (0, 0, 600), (5, 0, -50),
     "Top view, FRONT = up: proto board (black) with its USB socket at the back wall, relay (red) along the front",
     zoom=1.3, up=(0, 1, 0), ortho=True)
# shot("3_right_bay_closeup", ["tray", "pb", "pb_parts", "relay", "relay_parts", "rscrews", "xt60"], (190, 120, 120), (50, 0, -60),
#     (replaced by the cutaways below)
shot("4_back_wall_usb", ["tray", "pb", "pb_parts", "usb", "kit", "plugs"], (120, -320, -40), (40, -30, -65),
     "Back wall: the USB-A plug goes in from outside through its window; the cable runs to the kit's port end", zoom=1.6)
shot("5_v3_vs_v4", ["tray3_t", "tray_t", "collar_t"], (0, 0, 600), (5, 0, -50),
     "V3 tray (red) vs V4 (blue), top view: the right bay is 19.5 mm longer and 6.75 mm wider per side",
     zoom=1.3, up=(0, 1, 0), ortho=True)
shot("6_under_frame", ["plate", "legs", "collar", "tray", "bat", "xt60", "usb"], (280, -330, -200), (10, 0, -30),
     "Under the frame: your printed collar v3.1 (white) unchanged, tray V4 (blue) under it, legs clear")
shot("7_roof_section", ["tray"], (260, 40, -10), (60, 0, -55),
     "Right end: 45 deg roof from the collar joint down to the end wall, so it prints with no supports", zoom=1.6)
shot("8_print_plate", ["bed", "t_p"], (260, -300, 330), (0, 0, 20),
     "K2 Pro bed (300 x 300): tray V4 floor-down, no supports")
print("ok")

# cutaways of the right bay (roof and walls above 30 mm removed so the floor features show)
cut = B.box(13.0, 100, -60, 60, B.Z_FLOOR_TOP + 30.0, 0)
S["tray_cut"] = (tray.cut(cut), (0.16, 0.36, 0.86), 1.0)
A["tray_cut"] = actor("tray_cut")
shot("3a_right_bay_empty_cutaway", ["tray_cut"], (150, -150, 140), (50, 0, -78),
     "Right bay floor (cut away above 30 mm): proto board rails + 4 corner lips, 4 relay posts, zip slots", zoom=1.9)
shot("3b_right_bay_loaded_cutaway", ["tray_cut", "pb", "pb_parts", "usb", "relay", "relay_parts", "rscrews", "xt60"],
     (150, -150, 140), (50, 0, -78),
     "Same cutaway with the boards: proto board on its rails, relay on its posts, USB plug from outside", zoom=1.9)
print("ok cutaways")
