# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/sensor_deck/v3_1_CURRENT/src/render_v3_1.py
#  Part    : sensor_deck
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

# Renders for sensor deck v3.1. Run: python render_v3_1.py OUT_DIR
import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sensor_deck_v3_1 as D
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
R = D.rot
deck = D.make_deck(); imu = R(D.make_imu()); imu_h = R(D.make_imu_headers()); ble = R(D.make_ble())
ble_h, ble_m = (R(x) for x in D.make_ble_parts())
hub, out_h = (R(x) for x in D.make_hub()); pins = R(D.make_pins()); links = R(D.make_links()); heads = D.make_heads()
# the IMU's printed X arrow (as on the board), so the renders show which way it points
xarrow = R(cq.Workplane("XY").polyline([(-6, -0.6), (3, -0.6), (3, -2.0), (7, 0), (3, 2.0), (3, 0.6), (-6, 0.6)])
           .close().extrude(0.15).translate((0, -4, D.BASE_T + D.BOARD_T)))
bed = D.box(-50, 30, -20, 20, -0.6, 0)
P = {"plate": (D.make_plate(), (0.17, 0.17, 0.17)), "deck": (deck, (0.25, 0.42, 0.85)),
     "imu": (imu, (0.8, 0.12, 0.12)), "imuh": (imu_h, (0.12, 0.12, 0.12)), "xarrow": (xarrow, (1, 1, 1)),
     "ble": (ble, (0.1, 0.18, 0.55)), "bleh": (ble_h, (0.12, 0.12, 0.12)), "blem": (ble_m, (0.75, 0.75, 0.72)),
     "hub": (hub, (0.1, 0.4, 0.18)), "outh": (out_h, (0.12, 0.12, 0.12)), "pins": (pins, (0.9, 0.78, 0.3)),
     "links": (links, (0.98, 0.5, 0.08)), "heads": (heads, (0.35, 0.35, 0.35)), "bed": (bed, (0.55, 0.6, 0.65)),
     "printed": (D.for_print(deck), (0.25, 0.42, 0.85))}
def actor(n):
    s, c = P[n]; p = f"/tmp/_d31_{n}.stl"; cq.exporters.export(s, p, tolerance=0.02, angularTolerance=0.05)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetSpecular(0.15); pr.SetAmbient(0.25); return ac
A = {k: actor(k) for k in P}
def shot(f, names, pos, foc, title, zoom=1.2, up=(0, 0, 1), labels=()):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up); r.ResetCamera(); c.Zoom(zoom)
    for (txt, x, y, size) in ((title, 15, 815, 20),) + tuple(labels):
        t = vtk.vtkTextActor(); t.SetInput(txt); t.GetTextProperty().SetFontSize(size); t.GetTextProperty().SetColor(.1, .1, .1)
        t.GetTextProperty().SetBold(size > 20); t.SetPosition(x, y); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()
parts = ["deck", "imu", "imuh", "xarrow", "ble", "bleh", "blem", "hub", "outh", "pins", "links"]
shot("1_top_view", ["plate"] + parts, (0, 0, 300), (-12, 0, 0),
     "v3.1 top view, TRUE FRONT = up. IMU X arrow -> front, Bluefruit on the LEFT (white arms side)", zoom=2.0, up=(0, 1, 0),
     labels=(("FRONT", 560, 760, 28), ("LEFT (white arms)", 20, 420, 24), ("RIGHT (red arms)", 960, 420, 24), ("BACK (battery pads)", 470, 40, 24)))
shot("2_assembled", ["plate"] + parts, (-150, -120, 140), (-14, 2, 6),
     "Sensor deck v3.1 on the bottom plate, seen from the back-left", zoom=1.9)
shot("3_hub_closeup", parts, (-70, -55, 75), (-15, 1, 9),
     "Hub strip on top of both pin rows: IMU GND...CS row and the Bluefruit row, 7-pin cable header between", zoom=2.6)
shot("4_deck_bare", ["deck", "heads"], (-90, -100, 110), (-10, 0, 2),
     "Deck alone (screw heads shown sunk flush): keys into the 3 slots, pilots for IMU (4) + Bluefruit (2)", zoom=1.4)
shot("5_print_orientation", ["bed", "printed"], (-100, -110, 80), (-10, 0, 2),
     "PRINT: flat top on the bed, keys up, no supports (STL is already flipped)", zoom=1.4)
print("ok")
