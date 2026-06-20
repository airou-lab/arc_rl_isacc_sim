
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
import math

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
    # Force 0.0 steering and high throttle
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("STRAIGHT LINE TERMINATION TEST (DIAGNOSTIC)")
    print("Goal: Drive straight and hit the boundary.")
    print("="*60)
    
    obs_dict, _ = env.reset()
    
    # Action: [steer, throttle, brake]
    action_move = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)
    action_stop = torch.tensor([[0.0, 0.0, 0.0]], device=env.device)
    
    for i in range(2000):
        # Apply zero throttle until the car lands, then full throttle
        if i < 10:
            curr_action = action_stop.clone()
        else:
            curr_action = action_move.clone()
            
        obs_dict, rewards, terminated, truncated, info = env.step(curr_action)
        
        if i < 10 or i % 50 == 0:
            speed = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
            pos = env.scene["robot"].data.root_pos_w[0]
            quat = env.scene["robot"].data.root_quat_w[0]
            w, x, y, z = quat[0], quat[1], quat[2], quat[3]
            pitch = math.atan2(2*(w*y + x*z), 1 - 2*(y*y + z*z))
            print(f"Step {i:3d} | Speed: {speed:5.2f} | X: {pos[0]:.2f} | Y: {pos[1]:.2f} | Z: {pos[2]:.3f} | Pitch: {math.degrees(pitch):5.1f}°")

        if terminated[0]:
            print(f"\nTERMINATED at step {i}! Final Pos: X={env.scene['robot'].data.root_pos_w[0,0]:.2f}, Y={env.scene['robot'].data.root_pos_w[0,1]:.2f}")
            break

    simulation_app.close()

if __name__ == "__main__":
    main()
