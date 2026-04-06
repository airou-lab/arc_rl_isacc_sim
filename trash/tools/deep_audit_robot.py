from isaaclab.app import AppLauncher
import argparse
import sys
import os

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

# Add arcproLab to path
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
sys.path.append(ROOT_DIR)
sys.path.append(ARCPRO_LAB_DIR)

import torch
import numpy as np
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedEnv
from pxr import UsdGeom, Usd, UsdPhysics, PhysxSchema

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedEnv(cfg=env_cfg)

    print("\n--- DEEP PHYSICS AUDIT ---")
    
    # Check for PhysicsScene
    stage = env.sim.stage
    scene_found = False
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            print(f"[Physics] Found PhysicsScene at: {prim.GetPath()}")
            scene_found = True
    if not scene_found:
        print("[Physics] [!] WARNING: No UsdPhysics.Scene found in stage!")

    print("\n[Stage Prims (First 50)]")
    for i, prim in enumerate(stage.Traverse()):
        if i >= 50: break
        print(f"  {prim.GetPath()} ({prim.GetTypeName()})")

    # 1. Inspect Robot Articulation
    robot = env.scene["robot"]
    print(f"\n[Robot: {robot.cfg.prim_path}]")
    print(f"  Total Mass: {torch.sum(robot.data.default_mass).item():.3f} kg")
    print(f"  Scale: {robot.cfg.spawn.scale}")
    
    # Check Joint Gains (Are they strong enough for an 8x robot?)
    stiffnesses = robot.root_physx_view.get_dof_stiffnesses()
    dampings = robot.root_physx_view.get_dof_dampings()
    armatures = robot.root_physx_view.get_dof_armatures()
    
    for i, joint_name in enumerate(robot.joint_names):
        print(f"  Joint '{joint_name}': P={stiffnesses[0, i]:.1f}, D={dampings[0, i]:.1f}, Arm={armatures[0, i]:.3f}")

    # 2. Inspect Track Surface Properties
    print("\n[Track Surface Audit]")
    stage = env.sim.stage
    count = 0
    for prim in stage.Traverse():
        if "Track" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                count += 1
                if count <= 5: # Only show first 5 for brevity
                    coll = UsdPhysics.MeshCollisionAPI(prim)
                    approx = coll.GetApproximationAttr().Get()
                    
                    # Check Material
                    mat_path = ""
                    if prim.HasAPI(UsdPhysics.MaterialAPI):
                        mat_binding = UsdPhysics.MaterialAPI(prim).GetPhysicsMaterialBindingRel().GetTargets()
                        if mat_binding: mat_path = mat_binding[0]
                    
                    print(f"  Mesh: {prim.GetPath().name} | Approx: {approx} | Mat: {mat_path}")

    import omni.physx
    query = omni.physx.get_physx_scene_query_interface()
    # Raycast down from robot pos
    pos = robot.data.root_pos_w[0]
    start = (float(pos[0]), float(pos[1]), 50.0)
    direction = (0.0, 0.0, -1.0)
    hit = query.raycast_closest(start, direction, 100.0)
    
    if hit["hit"]:
        print(f"\nTrack found at robot XY: Z = {hit['position'][2]:.4f}")
        hit_path = hit.get("rigid_body") or hit.get("collision_path") or "Unknown"
        print(f"  Body hit: {hit_path}")
    else:
        print(f"\n[!] Track NOT FOUND at robot XY ({pos[0]:.2f}, {pos[1]:.2f})")
        # Check if we hit ANYTHING else
        hit_any = query.raycast_closest(start, direction, 1000.0)
        if hit_any["hit"]:
            any_path = hit_any.get("rigid_body") or hit_any.get("collision_path") or "Unknown"
            print(f"  Note: Found SOMETHING far below at Z={hit_any['position'][2]:.2f} (Path: {any_path})")

    # 3. Step Simulation and Check Contacts
    print("\n[Simulation Stage Traversal - Active Colliders]")
    coll_count = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            coll_count += 1
            if coll_count <= 20: # Show more to find the road
                xform = UsdGeom.Xformable(prim)
                world_pos = (0,0,0)
                if xform:
                    world_pos = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                print(f"  Collider: {prim.GetPath()} | Pos: {world_pos}")
    print(f"  Total Colliders in Sim: {coll_count}")

    print("\n[Contact Force Test - 10 Steps]")
    env.reset()
    for step in range(10):
        env.step(torch.zeros_like(env.action_manager.action))
        
        # Check robot Z velocity and position
        z_pos = robot.data.root_pos_w[0, 2].item()
        z_vel = robot.data.root_lin_vel_w[0, 2].item()
        
        print(f"  Step {step:02d} | Z-Pos: {z_pos:.3f} | Z-Vel: {z_vel:.3f}")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
