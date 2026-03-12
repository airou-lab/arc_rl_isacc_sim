from pxr import Usd, UsdGeom, Gf, UsdPhysics
import sys

usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(usd_path)

def get_world_pos(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid(): return None
    xform = UsdGeom.Xformable(prim)
    return xform.ComputeLocalToWorldTransform(0).ExtractTranslation()

print(f"--- FINAL ARCPro AUDIT ---")

# A. Kinematics
fl = get_world_pos("/Full_Car/Rigid_Bodies/Wheel_Front_Left")
rl = get_world_pos("/Full_Car/Rigid_Bodies/Wheel_Rear_Left")
fr = get_world_pos("/Full_Car/Rigid_Bodies/Wheel_Front_Right")

if fl and rl and fr:
    wb = abs(fl[0] - rl[0])
    tw = abs(fl[1] - fr[1])
    print(f"Wheelbase: {wb*100:.2f} cm (Target: 25 cm)")
    print(f"Track Width: {tw*100:.2f} cm (Target: 24 cm)")

# B. Sensors
lidar = get_world_pos("/Full_Car/Rigid_Bodies/Chassis/Lidar")
cam = get_world_pos("/Full_Car/Rigid_Bodies/Chassis/Camera_Left")
print(f"LIDAR Offset: {lidar}")
print(f"Camera Offset: {cam}")

# C. Mass
chassis = stage.GetPrimAtPath("/Full_Car/Rigid_Bodies/Chassis/Chassis")
total_mass = 0
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.MassAPI):
        total_mass += UsdPhysics.MassAPI(prim).GetMassAttr().Get()
print(f"Total Robot Mass: {total_mass:.3f} kg (Target: 4.092 kg)")
