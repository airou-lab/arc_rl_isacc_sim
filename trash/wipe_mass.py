from pxr import Usd, UsdGeom, Gf, UsdPhysics
import os

baked_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(baked_usd)

print("--- Stripping existing Mass APIs ---")
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.MassAPI):
        print(f"Removing Mass from: {prim.GetPath()}")
        prim.RemoveAPI(UsdPhysics.MassAPI)

# 1. Apply Chassis Mass (3.592)
chassis_prim = stage.GetPrimAtPath("/Full_Car/Rigid_Bodies/Chassis/Chassis")
if chassis_prim.IsValid():
    UsdPhysics.MassAPI.Apply(chassis_prim).GetMassAttr().Set(3.592)
    print(f"Applied 3.592kg to {chassis_prim.GetPath()}")

# 2. Apply Wheel Mass (0.125 each)
wheels = [
    "/Full_Car/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left",
    "/Full_Car/Rigid_Bodies/Wheel_Front_Right/Wheel_Front_Right",
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Left/Wheel_Rear_Left",
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Right/Wheel_Rear_Right"
]
for w in wheels:
    w_prim = stage.GetPrimAtPath(w)
    if w_prim.IsValid():
        UsdPhysics.MassAPI.Apply(w_prim).GetMassAttr().Set(0.125)
        print(f"Applied 0.125kg to {w_prim.GetPath()}")

stage.GetRootLayer().Save()
print("Mass calibration finalized.")
