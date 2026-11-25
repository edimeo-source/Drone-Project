# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/sensor_deck/v3/src/render_v3.py
#  Part    : sensor_deck
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sensor_deck_v3 as D
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
deck = D.make_deck(); imu = D.make_imu(); imu_h = D.make_imu_headers(); ble = D.make_ble(); ble_h, ble_m = D.make_ble_parts()
hub, out_h = D.make_hub(); pins = D.make_pins(); links = D.make_links()
bed = D.box(-45, 45, -30, 60, -0.6, 0)
P = {"plate": (D.make_plate(), (0.17, 0.17, 0.17), 1), "deck": (deck, (0.94, 0.94, 0.94), 1),
     "imu": (imu, (0.8, 0.12, 0.12), 1), "imuh": (imu_h, (0.12, 0.12, 0.12), 1),
     "ble": (ble, (0.12, 0.25, 0.75), 1), "bleh": (ble_h, (0.12, 0.12, 0.12), 1), "blem": (ble_m, (0.75, 0.75, 0.72), 1),
     "hub": (hub, (0.1, 0.4, 0.18), 1), "outh": (out_h, (0.12, 0.12, 0.12), 1), "pins": (pins, (0.9, 0.78, 0.3), 1),
     "links": (links, (0.98, 0.5, 0.08), 1), "bed": (bed, (0.55, 0.6, 0.65), 1),
     "printed": (D.for_print(deck), (0.94, 0.94, 0.94), 1)}
def actor(n):
    s, c, a = P[n]; p = f"/tmp/_d3_{n}.stl"; cq.exporters.export(s, p, tolerance=0.02, angularTolerance=0.05)
    r = vtk.vtkSTLReader(); r.SetFileName(p); nm = vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); ac = vtk.vtkActor(); ac.SetMapper(m)
    pr = ac.GetProperty(); pr.SetColor(*c); pr.SetOpacity(a); pr.SetSpecular(0.15); pr.SetAmbient(0.25); return ac
A = {k: actor(k) for k in P}
def shot(f, names, pos, foc, title, zoom=1.2, up=(0, 0, 1)):
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1)
    for n in names: r.AddActor(A[n])
    c = r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up); r.ResetCamera(); c.Zoom(zoom)
    t = vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1, .1, .1); t.SetPosition(15, 815); r.AddActor2D(t)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1200, 850); w.SetMultiSamples(8); w.Render()
    g = vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p = vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()
parts = ["deck", "imu", "imuh", "ble", "bleh", "blem", "hub", "outh", "pins", "links"]
shot("1_deck_assembled", ["plate"] + parts, (110, -120, 130), (0, 14, 6), "Sensor deck v3: IMU in the middle, Bluefruit to the left, hub strip on top of both pin rows", zoom=1.9)
shot("2_hub_closeup", parts, (55, -45, 70), (1, 15, 9), "Hub strip: IMU pins + Bluefruit pins come up through it, 7-pin cable header, 5 orange wire links", zoom=2.6)
shot("3_top_view", ["plate"] + parts, (0, 0, 300), (0, 12, 0), "Top view (FRONT = up). The IMU's X arrow must point up (front)", zoom=2.2, up=(1, 0, 0))
shot("4_deck_bare", ["deck"], (80, -90, 110), (0, 14, 2), "Deck alone: flat top, keys into the 3 center slots, pilots for IMU (4) + Bluefruit (2)", zoom=1.4)
shot("5_print_orientation", ["bed", "printed"], (90, -110, 80), (0, 14, 2), "PRINT: flat top on the bed, keys up, no supports (STL is already flipped)", zoom=1.4)
print("ok")
