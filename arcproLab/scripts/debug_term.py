import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

import torch
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, ROOT_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    robot = env.scene["robot"]
    sensor = env.scene["ground_contact"]

    print("\n--- SIMULATION STEP TRACE ---")
    action = torch.zeros((1, 2), device=env.device)
    
    for i in range(30):
        if i > 10:
            action[0, 1] = 1.0
            
        env.step(action)
        vel = robot.data.root_lin_vel_b[0, 0].item()
        
        # Check max force
        max_force = torch.max(torch.norm(sensor.data.net_forces_w, dim=-1), dim=-1)[0].item()
        term_flag = env.termination_manager.terminated[0].item()
        
        print(f"Step {i:2d} | FwdVel(LocalX): {vel:.3f} | GroundForce: {max_force:.1f} N | Term: {term_flag}")
            
    env.close()

if __name__ == "__main__":
    main()
