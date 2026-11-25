# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/01_case_v0/src/render_vtk.py
#  Part    : 01_case_v0
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

# Preview renders with VTK (offscreen). Same parts and colors as the assembly.
import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import case as C
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
case = C.make_case(); mount = C.make_imu_mount(); plate = C.make_plate()
kit, ble, bat, imu, top = C.make_parts_for_preview()
P = {"plate":(plate,(0.17,0.17,0.17),1.0), "top":(top,(0.17,0.17,0.17),0.35), "case":(case,(0.93,0.93,0.93),1.0),
     "mount":(mount,(0.97,0.97,0.97),1.0), "kit":(kit,(0.12,0.48,0.23),1.0), "ble":(ble,(0.15,0.28,0.71),1.0),
     "bat":(bat,(0.83,0.63,0.09),1.0), "imu":(imu,(0.78,0.16,0.16),1.0)}
def actor(name):
    s,col,a=P[name]; path=f"/tmp/_{name}.stl"
    cq.exporters.export(s,path,tolerance=0.05,angularTolerance=0.1)
    rd=vtk.vtkSTLReader(); rd.SetFileName(path)
    nm=vtk.vtkPolyDataNormals(); nm.SetInputConnection(rd.GetOutputPort()); nm.SetFeatureAngle(35)
    mp=vtk.vtkPolyDataMapper(); mp.SetInputConnection(nm.GetOutputPort())
    ac=vtk.vtkActor(); ac.SetMapper(mp); pr=ac.GetProperty(); pr.SetColor(*col); pr.SetOpacity(a)
    pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.25)
    return ac
A={k:actor(k) for k in P}
def shot(fname, names, pos, focal, up=(0,0,1), title="", zoom=1.0, size=(1100,800)):
    r=vtk.vtkRenderer(); r.SetBackground(1,1,1)
    for n in names: r.AddActor(A[n])
    cam=r.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
    r.ResetCamera(); cam.Zoom(zoom)
    if title:
        t=vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(0.1,0.1,0.1)
        t.SetPosition(15,size[1]-35); r.AddActor2D(t)
    w=vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(*size); w.SetMultiSamples(8)
    w.Render(); f=vtk.vtkWindowToImageFilter(); f.SetInput(w); f.Update()
    p=vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{fname}.png"); p.SetInputConnection(f.GetOutputPort()); p.Write()
allp=list(P)
shot("1_assembly_from_below", allp, (260,-330,-260), (0,0,-15), title="Case under the bottom plate, battery strapped under it (+X = front)", zoom=1.25)
shot("2_side", allp, (0,-600,0), (0,0,-10), title="Side: IMU on the top plate center; case + battery centered under the frame", zoom=1.3)
shot("3_case_with_parts", ["case","kit","ble"], (220,-260,220), (0,0,-25), title="Case: PIC32 kit on 4 corner towers (J12 jumpers hang below), Bluefruit pad at the front", zoom=1.3)
shot("4_case_bare", ["case"], (-230,-260,230), (0,0,-25), title="Case alone: keys + M3 bolt holes on the end plates, rear USB window, strap ears", zoom=1.3)
shot("5_imu_mount", ["mount","imu"], (90,-110,110), (5,0,45), title="IMU mount (top plate center, tape underneath, arrow = FRONT)", zoom=1.4)
shot("6_top_view_keys", ["plate","case"], (0,0,600), (0,0,0), up=(1,0,0), title="Top view through the plate: case keys sit in the 4 tab slots", zoom=1.3)
print("ok")
