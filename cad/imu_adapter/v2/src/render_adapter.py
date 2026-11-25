# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : cad/imu_adapter/v2/src/render_adapter.py
#  Part    : imu_adapter
#  What    : Makes the preview pictures in renders/.
#  FUNCTIONS: actor(), shot()
# ==============================================================================================

import sys, os, vtk, cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_adapter as A
out=sys.argv[1]; os.makedirs(out, exist_ok=True)
bed=A.box(-40,40,-28,28,-0.6,0)
parts={"plate":(A.make_plate_patch(),(0.15,0.15,0.15)),"adapter":(A.make_adapter(),(0.94,0.94,0.94)),
       "imu":(A.make_imu(),(0.8,0.12,0.12)),"printed":(A.for_print(A.make_adapter()),(0.94,0.94,0.94)),
       "bed":(bed,(0.55,0.6,0.65))}
def actor(n):
    s,c=parts[n]; p=f"/tmp/_{n}.stl"; cq.exporters.export(s,p,tolerance=0.02,angularTolerance=0.05)
    r=vtk.vtkSTLReader(); r.SetFileName(p); nm=vtk.vtkPolyDataNormals(); nm.SetInputConnection(r.GetOutputPort()); nm.SetFeatureAngle(35)
    m=vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort()); a=vtk.vtkActor(); a.SetMapper(m)
    pr=a.GetProperty(); pr.SetColor(*c); pr.SetSpecular(0.15); pr.SetAmbient(0.25); return a
A_={k:actor(k) for k in parts}
def shot(f,names,pos,foc,title,zoom=1.2,up=(0,0,1)):
    r=vtk.vtkRenderer(); r.SetBackground(1,1,1)
    for n in names: r.AddActor(A_[n])
    c=r.GetActiveCamera(); c.SetPosition(*pos); c.SetFocalPoint(*foc); c.SetViewUp(*up); r.ResetCamera(); c.Zoom(zoom)
    t=vtk.vtkTextActor(); t.SetInput(title); t.GetTextProperty().SetFontSize(20); t.GetTextProperty().SetColor(.1,.1,.1); t.SetPosition(15,765); r.AddActor2D(t)
    w=vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(1100,800); w.SetMultiSamples(8); w.Render()
    g=vtk.vtkWindowToImageFilter(); g.SetInput(w); g.Update(); p=vtk.vtkPNGWriter(); p.SetFileName(f"{out}/{f}.png"); p.SetInputConnection(g.GetOutputPort()); p.Write()
shot("adapter_top",["adapter"],(60,-80,90),(0,0,1),"v2 top: one flat face (IMU sits here), 4 pilot holes, edge reliefs, FRONT arrow")
shot("adapter_bottom",["adapter"],(50,-70,-90),(0,0,0),"Underside: 2 outer keys (ride the outer slot walls, 20.3 mm) + middle key")
shot("adapter_on_plate",["plate","adapter","imu"],(70,-95,80),(0,0,2),"On the bottom plate's 3 center slots, IMU flat on top (square to the frame)")
shot("print_orientation",["bed","printed"],(70,-95,60),(0,0,2),"HOW TO PRINT: flat top face on the bed, keys pointing up - no supports")
print("ok")
