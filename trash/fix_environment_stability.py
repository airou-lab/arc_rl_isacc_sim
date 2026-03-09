import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.kit.commands
import numpy as np
import shutil
import os

def apply_fixes(stage, usd_path):
    print(f"\n--- Executing Stability Fixes for {usd_path} ---")
    
    # 1. Increase PhysX Solver Iterations
    # Look for the PhysicsScene in typical locations
    physics_scene_paths = ["/World/PhysicsScene", "/PhysicsScene"]
    scene_prim = None
    for path in physics_scene_paths:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            scene_prim = prim
            break
    
    if scene_prim:
        print(f"[1/4] Beefing up PhysicsScene at {scene_prim.GetPath()}")
        # Check if the PhysxSceneAPI exists, if not apply it
        if not scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        
        physx_scene_api = PhysxSchema.PhysxSceneAPI(scene_prim)
        physx_scene_api.GetContactPatchSmoothingEnabledAttr().Set(True)
        
        # 16 pos, 4 vel iterations for 26-joint stability
        # Some attributes might need to be created if they don't exist
        pos_attr = scene_prim.GetAttribute("physxScene:positionIterationCount")
        if not pos_attr:
            pos_attr = scene_prim.CreateAttribute("physxScene:positionIterationCount", Usd.SdfValueTypeNames.Int)
        pos_attr.Set(16)
        
        vel_attr = scene_prim.GetAttribute("physxScene:velocityIterationCount")
        if not vel_attr:
            vel_attr = scene_prim.CreateAttribute("physxScene:velocityIterationCount", Usd.SdfValueTypeNames.Int)
        vel_attr.Set(4)
        print(f"      - Set Iterations: 16 pos, 4 vel. Enabled ContactPatchSmoothing.")
    else:
        print("[ERROR] PhysicsScene not found!")

    # 2. Identify Road/Ground and Replace Collision
    road_found = False
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if prim.IsA(UsdGeom.Mesh) and ("road" in name or "street" in name or "ground" in name):
            road_found = True
            print(f"[2/4] Found road-like mesh: {prim.GetPath()}")
            
            # Disable collision on the complex visual mesh
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            
            collision_api = UsdPhysics.CollisionAPI(prim)
            collision_api.GetCollisionEnabledAttr().Set(False)
            print(f"      - Disabled complex mesh collision.")

            # Calculate average Z-height to place the floor
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            if points and len(points) > 0:
                pts = np.array(points)
                z_height = np.mean(pts[:, 2])
                z_min = np.min(pts[:, 2])
                z_max = np.max(pts[:, 2])
                print(f"      - Z-Height stats: Mean={z_height:.4f}m, Min={z_min:.4f}m, Max={z_max:.4f}m")
                
                # Check for hills
                if (z_max - z_min) > 0.05:
                    print(f"      - [WARNING] Hills detected (Delta={z_max-z_min:.2f}m)! Plane fix may fail.")
                
                # Create a simple PhysicsPlane at the mean height
                plane_path = f"/World/PhysicsFloor_{prim.GetName()}"
                if not stage.GetPrimAtPath(plane_path).IsValid():
                    print(f"      - Creating flat PhysicsPlane at {plane_path} (Height={z_height:.4f}m)")
                    # We can't use Isaac core GroundPlane here without a running simulation_app, 
                    # but we can create the USD prims directly.
                    plane_prim = stage.DefinePrim(plane_path, "Plane")
                    UsdGeom.Plane(plane_prim).GetSizeAttr().Set(1000.0) # Massive
                    UsdGeom.Xformable(plane_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, z_height))
                    
                    # Apply physics to the plane
                    UsdPhysics.CollisionAPI.Apply(plane_prim)
                    PhysxSchema.PhysxCollisionAPI.Apply(plane_prim)
                    # Set approximation to 'None' for a perfect mathematical plane
                    PhysxSchema.PhysxCollisionAPI(plane_prim).GetApproximationAttr().Set("none")
            else:
                print(f"      - [ERROR] Could not retrieve vertices for {prim.GetPath()}.")

    if not road_found:
        print("[WARNING] No road-like mesh found to fix.")

    # Save the modified USD (creating a backup first)
    if os.path.exists(usd_path):
        shutil.copy2(usd_path, usd_path + ".bak")
        print(f"[4/4] Backup created: {usd_path}.bak")
        omni.usd.get_context().save_stage()
        print(f"      - Stage saved successfully.")

if __name__ == "__main__":
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    if os.path.exists(usd_path):
        omni.usd.get_context().open_stage(usd_path)
        stage = omni.usd.get_context().get_stage()
        apply_fixes(stage, usd_path)
    else:
        print(f"ERROR: {usd_path} not found")
    
    simulation_app.close()
