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
    
    # OVERRIDE DRIVE ACTION TO POSITIVE TO TEST FORWARD
    from arcproLab.mdp.actions import GroupedJointVelocityActionCfg
    env_cfg.actions.b_drive = GroupedJointVelocityActionCfg(
        asset_name="robot", 
        joint_names=["Joint_Drive_.*"], 
        scale=20.0, 
        offset=20.0
    )
    
    env_cfg.__post_init__()

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    robot = env.scene["robot"]

    print("\n--- SIMULATION STEP TRACE ---")
    action = torch.zeros((1, 2), device=env.device)
    action[0, 1] = 1.0 # This will now map to +40.0 rad/s

    for i in range(100):
        env.step(action)
        pos = robot.data.root_pos_w[0]
        vel = robot.data.root_lin_vel_b[0]
        
        print(f"Step {i:2d} | Fwd_Vel: {vel[0]:.3f} | Lat_Vel: {vel[1]:.3f}")

    env.close()

if __name__ == "__main__":
    main()
