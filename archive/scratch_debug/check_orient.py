import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom

usd_path = 'arcproLab/assets/robot/F1Tenth_Metric.usd'
stage = Usd.Stage.Open(usd_path)
for prim in stage.Traverse():
    if 'Wheel' in prim.GetName() and prim.GetTypeName() == 'Xform':
        xform = UsdGeom.Xform(prim)
        print(f"{prim.GetName()}:")
        for op in xform.GetOrderedXformOps():
            print(f"  {op.GetOpName()}: {op.Get()}")
simulation_app.close()
