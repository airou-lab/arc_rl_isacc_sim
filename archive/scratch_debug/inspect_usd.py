from pxr import Usd, UsdGeom, UsdPhysics
stage = Usd.Stage.Open("arcproLab/assets/robot/F1Tenth_Metric.usd")
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.CollisionAPI):
        print(f"Collision on: {p.GetPath()}")
    if p.IsA(UsdPhysics.Joint):
        print(f"Joint: {p.GetPath()} Type: {p.GetTypeName()}")
