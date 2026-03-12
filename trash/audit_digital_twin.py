from pxr import Usd, UsdGeom, Gf
import sys

usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(usd_path)

def get_bounds(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid(): return None
    bbox = UsdGeom.Imageable(prim).ComputeLocalBound(0, "default")
    return bbox.GetRange().GetMin(), bbox.GetRange().GetMax()

def get_world_pos(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid(): return None
    xform = UsdGeom.Xformable(prim)
    return xform.ComputeLocalToWorldTransform(0).ExtractTranslation()

print(f"--- ARCPro Digital Twin Audit: {usd_path} ---")

# 1. Total Length
bmin, bmax = get_bounds("/Full_Car")
total_length = (bmax - bmin)[0]
print(f"Total Length: {total_length*1000:.2f} mm (Target: 403 mm)")

# 2. Wheel Dimensions
w_min, w_max = get_bounds("/Full_Car/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left")
w_size = w_max - w_min
print(f"Wheel Radius: {w_size[2]*500:.2f} mm (Target: 50 mm)") # Radius is half height
print(f"Wheel Width: {w_size[1]*1000:.2f} mm (Target: 20 mm)")

# 3. Kinematics
fl = get_world_pos("/Full_Car/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left")
rl = get_world_pos("/Full_Car/Rigid_Bodies/Wheel_Rear_Left/Wheel_Rear_Left")
fr = get_world_pos("/Full_Car/Rigid_Bodies/Wheel_Front_Right/Wheel_Front_Right")
print(f"Wheelbase: {abs(fl[0]-rl[0])*100:.2f} cm (Target: 25 cm)")
print(f"Track Width: {abs(fl[1]-fr[1])*100:.2f} cm (Target: 24 cm)")

# 4. Sensor Offsets
lidar_pos = get_world_pos("/Full_Car/Rigid_Bodies/Chassis/Lidar")
cam_pos = get_world_pos("/Full_Car/Rigid_Bodies/Chassis/Camera_Left")
chassis_pos = get_world_pos("/Full_Car/Rigid_Bodies/Chassis/Chassis")

print(f"\nLIDAR Pos: {lidar_pos}")
print(f"Camera Pos: {cam_pos}")
print(f"Chassis Pos: {chassis_pos}")
