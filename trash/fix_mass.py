from pxr import Usd, UsdGeom, Gf, UsdPhysics
import os

baked_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(baked_usd)

# 1. Correct Chassis Mass (3.592)
chassis_prim = stage.GetPrimAtPath("/Full_Car/Rigid_Bodies/Chassis/Chassis")
if chassis_prim.IsValid():
    UsdPhysics.MassAPI.Apply(chassis_prim).GetMassAttr().Set(3.592)

# 2. Correct Wheel Mass (0.125 each)
wheels = [
    "/Full_Car/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left",
    "/Full_Car/Rigid_Bodies/Wheel_Front_Right/Wheel_Front_Right",
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Left/Wheel_Rear_Left",
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Right/Wheel_Rear_Right"
]
for w in wheels:
    w_prim = stage.GetPrimAtPath(w)
    if w_prim.IsValid():
        # Clean existing mass first to avoid accumulation if any
        if w_prim.HasAPI(UsdPhysics.MassAPI):
            w_prim.RemoveAPI(UsdPhysics.MassAPI)
        UsdPhysics.MassAPI.Apply(w_prim).GetMassAttr().Set(0.125)

stage.GetRootLayer().Save()
print("Mass calibration finalized at 4.092kg.")
