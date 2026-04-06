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
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedEnv
from pxr import UsdGeom, Usd

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedEnv(cfg=env_cfg)

    print("\n--- SPATIAL AUDIT ---")
    stage = env.sim.stage
    
    # 1. Find all "drivable" meshes and print their world-space bounding boxes
    print("\n[Road Mesh World Locations]")
    found_road = False
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.IsA(UsdGeom.Mesh) and any(k in path.lower() for k in ["drivable", "pavement", "road"]):
            bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim)
            r = bbox.GetRange()
            print(f"Road Prim: {path}")
            print(f"  BBox: {r.GetMin()} to {r.GetMax()}")
            found_road = True
            # Just show first few to avoid spam
            if "piece_10" in path: # This was one near your spawn
                 print(f"  [TARGET] Found piece_10 at {r.GetMin()} to {r.GetMax()}")

    if not found_road:
        print("[!] No road meshes found in the simulation stage!")

    # 2. Check robot default spawn
    robot = env.scene["robot"]
    pos = robot.data.root_pos_w[0]
    print(f"\n[Robot Spawn]")
    print(f"  Current World Pos: {pos.cpu().numpy()}")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
