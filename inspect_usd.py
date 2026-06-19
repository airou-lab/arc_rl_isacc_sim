
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open('arcproLab/assets/robot/F1Tenth_Metric.usd')
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Xformable):
        print(prim.GetPath(), prim.GetTypeName())

