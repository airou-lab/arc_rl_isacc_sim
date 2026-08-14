from pxr import Usd, UsdPhysics
stage = Usd.Stage.Open("arcproLab/assets/robot/F1Tenth_Metric.usd")
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Joint):
        print(f"{prim.GetPath()}: {prim.GetProperty('physics:localPos0').Get()}")
