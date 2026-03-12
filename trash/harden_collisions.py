from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, UsdPhysics
import omni.usd

baked_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(baked_usd)

print(f"[Phase 2.5] Hardening Collisions for {baked_usd}...")

# 1. Target Meshes for Collision
collision_paths = [
    "/Full_Car/Rigid_Bodies/Chassis/Chassis",
    "/Full_Car/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left",
    "/Full_Car/Rigid_Bodies/Wheel_Front_Right/Wheel_Front_Right",
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Left/Wheel_Rear_Left",
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Right/Wheel_Rear_Right"
]

from omni.physx.scripts import utils

for path in collision_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        # Ensure it has CollisionAPI
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
        
        # Apply MeshCollisionAPI for approximation setting
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision.CreateApproximationAttr().Set("convexHull")
        
        print(f"  Set Convex Hull Collision: {path}")

stage.GetRootLayer().Save()
print("\n[Phase 2.5] Collision Hardening Complete.")

simulation_app.close()
