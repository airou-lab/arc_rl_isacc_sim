import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test termination by driving straight.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys

# Add arcproLab to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # Disable cameras for headless test
    env_cfg.observations.visual = None
    env_cfg.scene.tiled_camera = None
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("STRAIGHT LINE TERMINATION TEST")
    print("Goal: Drive straight and hit the boundary.")
    print("="*60)
    
    obs_dict, _ = env.reset()
    
    # Action: [steer, throttle, brake]
    action = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)
    
    for i in range(500):
        obs_dict, rewards, terminated, truncated, info = env.step(action)
        
        if terminated[0]:
            print(f"\nSUCCESS: Robot terminated at step {i}!")
            break
            
        if i % 10 == 0:
            speed = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
            pos = env.scene["robot"].data.root_pos_w[0]
            from mdp.track_manager import get_track_manager
            tm = get_track_manager(device=env.device)
            env_origins = env.scene.env_origins
            local_pos = env.scene["robot"].data.root_pos_w - env_origins
            dist_y, dist_w, dist_g = tm.compute_marker_distances(local_pos)
            
            print(f"Step {i:3d} | Speed: {speed:.2f} m/s | Pos: ({pos[0]:.2f}, {pos[1]:.2f}) | Dist_W: {dist_w[0]:.3f} | Dist_Y: {dist_y[0]:.3f}")

    if not terminated[0]:
        print("\nFAILURE: Robot reached 500 steps without termination.")
        
    simulation_app.close()

if __name__ == "__main__":
    main()
