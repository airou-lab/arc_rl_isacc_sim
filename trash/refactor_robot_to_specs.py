import os
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

def refactor_robot():
    robot_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth.usd"
    backup_usd = robot_usd + ".hardware_refactor.bak"
    
    if not os.path.exists(robot_usd):
        print(f"ERROR: {robot_usd} not found")
        return

    # Create Backup
    import shutil
    shutil.copy2(robot_usd, backup_usd)
    print(f"Backup created at: {backup_usd}")

    stage = Usd.Stage.Open(robot_usd)
    print(f"Refactoring robot to hardware specs...")

    # 1. MASS REFACTOR (Target: 4.092 kg)
    # Chassis: 3.0 kg
    # Wheels: 4 x 0.15 kg = 0.6 kg
    # Remainder (Misc/Arms): 0.492 kg / ~25 parts approx 0.02 kg each
    
    chassis_path = "/Full_Car/Rigid_Bodies/Chassis/Chassis"
    wheel_paths = [
        "/Full_Car/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left",
        "/Full_Car/Rigid_Bodies/Wheel_Front_Right/Wheel_Front_Right",
        "/Full_Car/Rigid_Bodies/Wheel_Rear_Left/Wheel_Rear_Left",
        "/Full_Car/Rigid_Bodies/Wheel_Rear_Right/Wheel_Rear_Right"
    ]

    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            
            if path == chassis_path:
                mass_api.GetMassAttr().Set(3.0)
                print(f"  - Set Chassis mass to 3.0 kg")
            elif path in wheel_paths:
                mass_api.GetMassAttr().Set(0.15)
                print(f"  - Set Wheel mass to 0.15 kg ({path.split('/')[-1]})")
            else:
                # Every other part gets a small minimum mass to avoid ZeroDivision
                mass_api.GetMassAttr().Set(0.02)

    # 2. SENSOR OFFSETS (Hardware Specs)
    # LIDAR: 235mm L (X), 0mm W (Y), 265.23 mm H (Z)
    # Realsense: 145 mm L (X), 0 W (Y), 195 mm H (Z)
    # NOTE: Units in USD are likely Meters, so 235mm = 0.235m
    
    lidar_path = "/Full_Car/Rigid_Bodies/Chassis/Lidar"
    # Using Camera_Left as Realsense proxy
    realsense_path = "/Full_Car/Rigid_Bodies/Chassis/Camera_Left" 
    
    def set_offset(path, x, y, z):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            # Find or create Translate op
            translate_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            
            if not translate_op:
                translate_op = xform.AddTranslateOp()
            
            translate_op.Set(Gf.Vec3d(x, y, z))
            print(f"  - Moved {path.split('/')[-1]} to [{x}, {y}, {z}]")

    set_offset(lidar_path, 0.235, 0.0, 0.26523)
    set_offset(realsense_path, 0.145, 0.0, 0.195)

    # 3. JOINT DAMPING & COLLISION FIX
    print("  - Sanitizing joints and collision meshes...")
    for prim in stage.Traverse():
        # Reset extreme damping (found 100M in diagnostic)
        if "Joint" in prim.GetTypeName():
            for prop in prim.GetProperties():
                if "damping" in prop.GetName():
                    val = prop.Get()
                    if val is not None and val > 1000:
                        prop.Set(50.0) # Sane default
        
        # Force Convex Hull for wheels/chassis to stop triangle-snagging
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
            if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                PhysxSchema.PhysxCollisionAPI.Apply(prim)
            
            physx_api = PhysxSchema.PhysxCollisionAPI(prim)
            # 'convexHull' is much more stable for suspension
            if not physx_api.GetApproximationAttr().HasValue() or physx_api.GetApproximationAttr().Get() == "none":
                physx_api.GetApproximationAttr().Set("convexHull")

    stage.GetRootLayer().Save()
    print("\nRefactor Complete. Robot is now Hardware-Aligned and Stabilized.")

if __name__ == "__main__":
    refactor_robot()
