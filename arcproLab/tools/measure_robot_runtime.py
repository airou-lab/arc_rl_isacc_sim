import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Measure robot dimensions at runtime.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from isaaclab.envs import ManagerBasedEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    # setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.__post_init__() 
    
    # setup environment
    env = ManagerBasedEnv(cfg=env_cfg)
    robot = env.scene["robot"]
    
    # reset environment to initialize data
    env.reset()
    
    # Find all bodies
    body_names = robot.data.body_names
    print(f"\nAll robot bodies ({len(body_names)}):")
    # print(body_names)
    
    # Look for wheel bodies specifically
    wheel_bodies = [name for name in body_names if "Wheel" in name and "Mesh" not in name]
    print(f"\nWheel bodies identified: {wheel_bodies}")
    
    # Get indices
    indices, resolved_names = robot.find_bodies(wheel_bodies)
    
    # World positions
    pos_w = robot.data.body_pos_w[0, indices] # (4, 3)
    
    # Print positions
    body_map = {}
    for i, name in enumerate(resolved_names):
        p = pos_w[i].cpu().numpy()
        print(f"Body '{name}' pos: {p}")
        body_map[name] = pos_w[i]

    # Calculate dimensions
    try:
        # Expected names:
        # Wheel_Front_Left, Wheel_Front_Right, Wheel_Rear_Left, Wheel_Rear_Right
        fl = body_map.get("Wheel_Front_Left")
        fr = body_map.get("Wheel_Front_Right")
        rl = body_map.get("Wheel_Rear_Left")
        rr = body_map.get("Wheel_Rear_Right")
        
        if all(v is not None for v in [fl, fr, rl, rr]):
            wheelbase = (torch.norm(fl[0] - rl[0]) + torch.norm(fr[0] - rr[0])) / 2.0
            track_front = torch.norm(fl[1] - fr[1])
            track_rear = torch.norm(rl[1] - rr[1])
            
            print("\n--- Dimensions (at current scale) ---")
            print(f"Wheelbase:   {wheelbase.item():.4f} m")
            print(f"Track Front: {track_front.item():.4f} m")
            print(f"Track Rear:  {track_rear.item():.4f} m")
            print(f"Wheel FL Z:  {fl[2].item():.4f} m")
            
            # Root height
            root_pos = robot.data.root_pos_w[0]
            print(f"Root Pos:    {root_pos.cpu().numpy()} m")
        else:
            print("\nCould not find all 4 wheel bodies by exact name.")
            # Fallback to index-based if they are in order
            if len(pos_w) >= 4:
                 print("Using first 4 bodies found for estimation...")
                 p1, p2, p3, p4 = pos_w[0], pos_w[1], pos_w[2], pos_w[3]
                 # Estimate wheelbase as max X diff
                 xs = pos_w[:, 0]
                 ys = pos_w[:, 1]
                 wb = torch.max(xs) - torch.min(xs)
                 tw = torch.max(ys) - torch.min(ys)
                 print(f"Estimated Wheelbase: {wb.item():.4f} m")
                 print(f"Estimated Track Width: {tw.item():.4f} m")

    except Exception as e:
        print(f"Error calculating dimensions: {e}")
    
    app_launcher.app.close()

if __name__ == "__main__":
    main()
