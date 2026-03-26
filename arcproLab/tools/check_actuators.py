# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Find which joints move when commanded.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys

# Add arcproLab to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from isaaclab.envs import ManagerBasedEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.__post_init__() 
    
    env = ManagerBasedEnv(cfg=env_cfg)
    
    obs, _ = env.reset()
    
    # 4 phases, each commanding ONE throttle joint
    for phase in range(4):
        print(f"\n--- Testing Throttle Joint {phase} (Action Index {2 + phase}) ---")
        for step in range(100):
            actions = torch.zeros((1, 6), device=env.device)
            actions[0, 2 + phase] = 20.0 # Command one wheel
            
            env.step(actions)
            
            if step == 99:
                # Check root velocity
                vel = env.scene["robot"].data.root_lin_vel_w[0]
                print(f"  Root Vel: {vel.cpu().numpy()}")
                
                # Check ALL 26 joints
                jv = env.scene["robot"].data.joint_vel[0]
                moving_indices = torch.where(torch.abs(jv) > 1.0)[0]
                for idx in moving_indices:
                    name = env.scene["robot"].data.joint_names[idx.item()]
                    print(f"  Joint {idx.item()} '{name}' is moving: {jv[idx.item()].item():.2f} rad/s")
    
    env.close()

if __name__ == "__main__":
    main()
