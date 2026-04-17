import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Lane Calibration Script")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from isaaclab.envs import ManagerBasedRLEnv
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False # Disable cameras
    env_cfg.scene.tiled_camera = None # Force remove from scene
    
    # Disable all observation groups that might use the camera
    if hasattr(env_cfg.observations, "visual"):
        env_cfg.observations.visual = None
    if hasattr(env_cfg.observations.policy, "visual"):
        env_cfg.observations.policy.visual = None
        
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # We want to measure the lat_err at different points
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # Spawn point
    spawn_x, spawn_y = -16.25375, 5.56
    yaw = torch.tensor([1.5708], device=env.device) # North
    
    print("\n" + "="*60)
    print("LANE CALIBRATION (1.0x Metric)")
    print("="*60)
    
    # Sweep X from -25 to -10 (crossing the road)
    for x in np.linspace(-22.0, -10.0, 25):
        pos = torch.tensor([[x, spawn_y, 0.1]], device=env.device)
        lat_err, _ = tm.compute_errors(pos, yaw)
        print(f"X: {x:.3f} | LatErr: {lat_err[0].item():.3f}m")
    
    print("="*60)
    env.close()
    simulation_app.close()

import numpy as np
if __name__ == "__main__":
    main()
