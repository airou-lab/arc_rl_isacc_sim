import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
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

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.observations.visual = None
    env_cfg.scene.tiled_camera = None
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    env.reset()
    action = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)
    
    print("\n" + "="*60)
    for i in range(10):
        env.step(action)
        pos = env.scene["robot"].data.root_pos_w[0]
        quat = env.scene["robot"].data.root_quat_w[0]
        vel_w = env.scene["robot"].data.root_lin_vel_w[0]
        vel_b = env.scene["robot"].data.root_lin_vel_b[0]
        
        # Calculate forward vector from quaternion
        # In w,x,y,z format
        w, x, y, z = quat
        forward_x = 1 - 2*(y*y + z*z)
        forward_y = 2*(x*y + w*z)
        forward_z = 2*(x*z - w*y)
        
        print(f"Step {i}")
        print(f"  Pos (World): {pos.cpu().numpy()}")
        print(f"  Quat (wxyz): {quat.cpu().numpy()}")
        print(f"  Fwd Vector : [{forward_x:.2f}, {forward_y:.2f}, {forward_z:.2f}]")
        print(f"  Vel (World): {vel_w.cpu().numpy()}")
        print(f"  Vel (Local): {vel_b.cpu().numpy()}")
        
    simulation_app.close()

if __name__ == "__main__":
    main()
