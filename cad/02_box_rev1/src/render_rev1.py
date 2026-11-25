# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/02_box_rev1/src/render_rev1.py
#  Part    : 02_box_rev1
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

# Preview renders for box rev 1 (VTK offscreen). Run: python render_rev1.py OUT_DIR
import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_rev1 as B
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
bx = B.make_box(); plate = B.make_plate(); Pp = B.make_parts(); adapter, imu = B.make_imu_on_plate()
P = {"plate": (plate, (0.17, 0.17, 0.17), 1.0), "box": (bx, (0.93, 0.93, 0.93), 1.0),
     "kit": (Pp["kit"].union(Pp["usb"]).union(Pp["j12"]), (0.12, 0.48, 0.23), 1.0),
     "jump": (Pp["jumpers"], (0.25, 0.25, 0.25), 0.55), "screw": (Pp["screw"], (0.55, 0.55, 0.6), 1.0),
     "ble": (Pp["ble"], (0.15, 0.28, 0.71), 1.0), "relay": (Pp["relay"], (0.92, 0.45, 0.1), 0.85),
     "xt60": (Pp["xt60"], (0.95, 0.8, 0.1), 1.0), "bat": (Pp["bat"], (0.83, 0.63, 0.09), 1.0),
     "adapter": (adapter, (0.97, 0.97, 0.97), 1.0), "imu": (imu, (0.78, 0.16, 0.16), 1.0)}


def actor(name):
    s, col, a = P[name]; path = f"/tmp/_r1_{name}.stl"
    cq.exporters.export(s, path, tolerance=0.05, angularTolerance=0.1)
    rd = vtk.vtkSTLReader(); rd.SetFileName(path)
    nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(rd.GetOutputPort()); nm.SetFeatureAngle(35)
    mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(nm.GetOutputPort())
    ac = vtk.vtkActor(); ac.SetMapper(mp); pr = ac.GetProperty(); pr.SetColor(*col); pr.SetOpacity(a)
    pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25)
    return ac


A = {k: actor(k) for k in P}


def shot(fname, names, pos, focal, up=(0, 0, 1), title="", zoom=1.0, size=(1200, 850)):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    cam = r.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
    r.ResetCamera(); cam.Zoom(zoom)
    if title:
        t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20)
        t.GetTextProperty().SetColor(0.1, 0.1, 0.1); t.SetPosition(15, size[1] - 35); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(*size); w.SetMultiSamples(8)
    w.Render(); f = vtk.vtkWindowToImageFilter(); f.SetInput(w); f.Update()
    p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{fname}.png"); p.SetInputConnection(f.GetOutputPort()); p.Write()


inside = ["box", "kit", "jump", "screw", "ble", "relay", "xt60"]
shot("1_box_bare", ["box"], (210, -250, 230), (0, 0, -25),
     title="Rev 1 box: center screw post, 4 side lips, Bluefruit card slot, PWM slots, XT60 window", zoom=1.25)
shot("2_box_with_parts", inside, (210, -250, 230), (0, 0, -25),
     title="PIC32 on the post (1 x M2.5), Bluefruit in its slot, orange = PLACEHOLDER power/kill board", zoom=1.25)
shot("3_rear", inside, (-300, -150, 120), (0, 0, -25),
     title="Rear: USB window (J3 program, J11 PuTTY, J4 power under the board)", zoom=1.3)
shot("4_under_frame", ["plate", "box", "bat", "adapter", "imu", "xt60"], (260, -330, -260), (0, 0, -20),
     title="Under the bottom plate: keys + 4 x M3 in the tab slots, battery strapped below", zoom=1.25)
shot("5_top_view", ["box", "kit", "jump", "ble", "relay", "xt60", "screw"], (0, 0, 600), (0, 0, 0), up=(1, 0, 0),
     title="Top view (front = up): kit at the rear, power bay + Bluefruit at the front", zoom=1.3)
shot("6_side", ["plate", "box", "bat", "adapter", "imu", "xt60"], (0, -600, 0), (0, 0, -20),
     title="Side (front = right): box + battery centered under the frame", zoom=1.3)
print("ok")
