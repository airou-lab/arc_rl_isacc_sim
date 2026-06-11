import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test termination triggers.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
import mdp.terminations as mdp_done

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.observations.visual = None
    env_cfg.scene.tiled_camera = None
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n" + "="*60)
    print("TERMINATION DIAGNOSTIC TEST")
    print("="*60)
    
    obs_dict, _ = env.reset()
    action = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)
    
    for i in range(20):
        obs_dict, rewards, terminated, truncated, info = env.step(action)
        
        # Manually check termination conditions
        contact = mdp_done.white_line_contact(env, threshold=0.05).item()
        height = mdp_done.height_termination(env).item()
        stagnation = mdp_done.stagnation_termination(env).item()
        
        z = env.scene["robot"].data.root_pos_w[0, 2].item()
        speed = torch.norm(env.scene["robot"].data.root_lin_vel_b[0, :2]).item()
        
        print(f"Step {i:2d} | Z: {z:.3f} | Spd: {speed:.2f} || Contact: {contact} | Height: {height} | Stag: {stagnation}")

        if terminated[0]:
            print(f"\nTERMINATED at step {i}!")
            break

    simulation_app.close()

if __name__ == "__main__":
    main()
