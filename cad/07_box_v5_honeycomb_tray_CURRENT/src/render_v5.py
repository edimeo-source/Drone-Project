# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/07_box_v5_honeycomb_tray_CURRENT/src/render_v5.py
#  Part    : 07_box_v5_honeycomb_tray_CURRENT
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor_file(), actor(), shot()
# ==============================================================================================

# Pictures of tray V5 (honeycomb, flat bottom, bigger) - Ethan DiMeo
# Run:  python render_v5.py RENDER_DIR [path/to/tray_v4_1.stl for the size comparison]
import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tray_v5 as T
import box_v3 as B, collar_v3_1 as C

out = sys.argv[1]; os.makedirs(out, exist_ok=True)
v41 = sys.argv[2] if len(sys.argv) > 2 else None
tray, collar, plate = T.make_tray(), C.make_collar_v31(), B.make_plate()
P, Q = B.make_parts(), T.make_parts()
kit = P["pcb"].union(P["kit_top"]).union(P["kit_bot"])
BLUE = (0.16, 0.36, 0.86)
S = {"tray": (tray, BLUE, 1.0),
     "tray_cut": (tray.cut(B.box(-200, 200, -0.5, 200, -200, 10)), BLUE, 1.0),
     "collar": (collar, (0.93, 0.93, 0.93), 1.0),
     "plate": (plate, (0.17, 0.17, 0.17), 1.0),
     "kit": (kit, (0.12, 0.48, 0.23), 1.0), "jump": (P["jumpers"], (0.25, 0.25, 0.25), 0.8),
     "plugs": (P["plugs"], (0.15, 0.15, 0.18), 1.0),
     "pb": (Q["pb"], (0.10, 0.10, 0.10), 1.0), "pb_parts": (Q["pb_parts"], (0.80, 0.15, 0.15), 0.9),
     "usb": (Q["usb_plug"], (0.35, 0.35, 0.40), 1.0),
     "relay": (Q["relay"], (0.85, 0.12, 0.10), 1.0), "relay_parts": (Q["relay_parts"], (0.20, 0.45, 0.85), 1.0),
     "rscrews": (Q["relay_screws"], (0.6, 0.6, 0.65), 1.0),
     "wires": (Q["wires"], (0.95, 0.10, 0.10), 1.0), "bullets": (Q["bullets"], (0.85, 0.70, 0.20), 1.0),
     "bat": (Q["bat_top"], (0.83, 0.63, 0.09), 1.0)}
bed = cq.Workplane("XY").box(300, 300, 1).translate((0, 0, -0.5))
S["bed"] = (bed, (0.35, 0.35, 0.38), 1.0)
S["t_p"] = (tray.translate((0, 0, -T.ZB)), BLUE, 1.0)
ground = cq.Workplane("XY").box(420, 420, 1).translate((0, 0, T.ZB - 0.5))
S["ground"] = (ground, (0.80, 0.80, 0.78), 1.0)


def actor_file(p, c, a):
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetOpacity(a); pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25); return ac


def actor(n):
    s, c, a = S[n]; p = f"/tmp/_b5_{n}.stl"; cq.exporters.export(s, p, tolerance=0.05, angularTolerance=0.1)
    return actor_file(p, c, a)


A = {k: actor(k) for k in S}
if v41 and os.path.exists(v41):
    a41 = actor_file(v41, (0.95, 0.35, 0.10), 0.55)
    a41.SetPosition(0, 0, -86.1)                     # its STL sits on z = 0; V4.1 bottom was 86.1 under the plate
    A["v41"] = a41


def shot(f, names, pos, foc, title, zoom=1.25, up=(0, 0, 1), ortho=False):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names:
        if n in A: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up)
    if ortho: c.ParallelProjectionOn()
    r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()


inside = ["kit", "jump", "plugs", "pb", "pb_parts", "usb", "relay", "relay_parts", "rscrews", "wires", "bullets"]
shot("1_tray_v5", ["tray"], (230, -320, 160), (10, -5, -60),
     "Tray V5: honeycomb walls, 45 deg roof up to the collar joint, flat bottom")
shot("2_tray_v5_loaded", ["tray"] + inside, (230, -320, 220), (10, -5, -60),
     "Loaded: kit (green), 7805 proto board (black), relay in the back wire bay, battery leads out the slot")
shot("3_top_view", ["tray"] + inside, (0, 0, 600), (8, 0, -60),
     "Top view, FRONT = up: 18 mm of room each side of the kit (V4.1: 6), 40 mm free strip in front of the proto board",
     zoom=1.3, up=(0, 1, 0), ortho=True)
shot("4_size_vs_v4_1", ["tray", "v41"], (260, -330, 150), (10, 0, -60),
     "Size: V5 (blue) vs V4.1 (orange, see-through): wider, 12 mm deeper, same joint at the top")
shot("5_bottom_flat", ["tray"], (170, -230, -330), (10, 0, -60),
     "Underneath: one flat landing face (no strap ears), honeycomb only where nothing mounts")
shot("6_on_the_drone", ["ground", "plate", "collar", "tray", "bat", "bullets", "wires", "usb"], (300, -380, 40), (5, 0, -40),
     "On the drone: plate, collar v3.1 (white, unchanged), tray V5, battery on top; it sits flat on the ground", zoom=2.3)
shot("7_back_view", ["plate", "collar", "tray", "bullets", "wires", "usb", "bat"], (20, -420, -50), (5, -50, -55),
     "From the back: bullets hang out of the slot under the plate's back edge (as V4.1)", zoom=1.4)
shot("8_cutaway", ["tray_cut", "kit", "jump", "plugs", "pb", "pb_parts", "relay", "relay_parts", "wires"],
     (40, 400, 60), (5, 0, -60), "Cut open along the middle: kit cradle, proto board rails, relay in the back bay", zoom=1.3)
shot("9_print_plate", ["bed", "t_p"], (260, -300, 330), (0, 0, 25),
     "K2 Pro bed (300 x 300): tray V5 floor-down, no supports")
print("ok")
