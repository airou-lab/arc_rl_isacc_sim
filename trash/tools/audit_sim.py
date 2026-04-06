from isaaclab.app import AppLauncher
import argparse
import sys
import os

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

# Add parent and arcproLab to path
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
sys.path.append(ROOT_DIR)
sys.path.append(ARCPRO_LAB_DIR)

import torch
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedEnv
from pxr import UsdGeom, Usd, UsdPhysics
import omni.physx

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedEnv(cfg=env_cfg)

    print("\n--- Simulation Audit ---")
    stage = env.sim.stage
    
    # 1. Check Robot Scale and Position
    robot = env.scene["robot"]
    pos = robot.data.root_pos_w[0]
    quat = robot.data.root_quat_w[0]
    print(f"Robot Root Pos: {pos.cpu().numpy()}")
    
    # Check individual colliders
    for prim in stage.Traverse():
        if "Robot" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim)
            r = bbox.GetRange()
            print(f"Robot Mesh: {prim.GetPath()}")
            print(f"  World Range: {r.GetMin()} to {r.GetMax()}")
            print(f"  Approx Size: {r.GetMax()[0]-r.GetMin()[0]:.3f}m")

    # 2. Check Track height at Robot Position
    query = omni.physx.get_physx_scene_query_interface()
    # Raycast down from robot pos
    start = (float(pos[0]), float(pos[1]), 20.0)
    direction = (0.0, 0.0, -1.0)
    hit = query.raycast_closest(start, direction, 100.0)
    if hit["hit"]:
        print(f"\nTrack found at robot XY: Z = {hit['position'][2]:.4f}")
        print(f"  Body hit: {hit['rigid_body']}")
    else:
        print(f"\n[!] Track NOT FOUND at robot XY ({pos[0]:.2f}, {pos[1]:.2f})")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
