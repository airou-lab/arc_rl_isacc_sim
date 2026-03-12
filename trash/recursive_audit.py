from pxr import Usd, UsdGeom, Gf
import sys

usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(usd_path)

print(f"--- Recursive Hierarchy Audit: {usd_path} ---")
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
        size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
        print(f"Mesh: {prim.GetPath()} | Size: {size[0]*1000:.2f} x {size[1]*1000:.2f} x {size[2]*1000:.2f} mm")

# Find sensor-like Xforms
for prim in stage.Traverse():
    if "Lidar" in prim.GetName() or "Camera" in prim.GetName():
        xform = UsdGeom.Xformable(prim)
        pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
        print(f"Sensor Found: {prim.GetPath()} | World Pos: {pos}")
