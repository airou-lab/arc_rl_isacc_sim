import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics

usd_path = 'arcproLab/assets/robot/F1Tenth_Metric.usd'
stage = Usd.Stage.Open(usd_path)
for prim in stage.Traverse():
    if 'Wheel' in prim.GetName() and prim.GetTypeName() == 'Xform':
        print("Wheel:", prim.GetPath())
        for child in prim.GetChildren():
            print("  Child:", child.GetPath(), child.GetTypeName())
            if child.HasAPI(UsdPhysics.CollisionAPI):
                print("    HAS COLLISION API!")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            print("  WHEEL HAS COLLISION API!")
simulation_app.close()
