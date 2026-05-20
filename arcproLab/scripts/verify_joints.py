import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify each joint direction.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
from isaaclab.envs import ManagerBasedRLEnv
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    joints = ["Joint_Drive_RL", "Joint_Drive_RR", "Joint_Drive_FL", "Joint_Drive_FR"]
    scale = -40.0
    
    for joint_name in joints:
        print(f"\nTesting {joint_name} with scale {scale}...")
        env.reset()
        
        # Manually set joint velocity target
        # steering (2) + drive (4)
        targets = torch.zeros((1, 6), device=env.device)
        joint_idx = joints.index(joint_name) + 2
        targets[0, joint_idx] = 1.0 * scale
        
        for i in range(100):
            # We use set_joint_velocity_target directly on the asset to bypass ActionManager
            env.scene["robot"].set_joint_velocity_target(targets)
            env.sim.step()
            
        lin_vel = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
        print(f"Result Body Vel X: {lin_vel:.4f}")
        
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
