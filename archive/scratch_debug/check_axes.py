import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics

usd_path = 'arcproLab/assets/robot/F1Tenth_Metric.usd'
stage = Usd.Stage.Open(usd_path)
if stage:
    for prim in stage.Traverse():
        if prim.GetTypeName() == 'PhysicsRevoluteJoint':
            joint = UsdPhysics.RevoluteJoint(prim)
            axis = joint.GetAxisAttr().Get()
            print(f"{prim.GetName()} axis: {axis}")
simulation_app.close()
