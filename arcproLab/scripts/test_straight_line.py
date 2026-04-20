import os
import sys
import torch
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = True 
env_cfg.__post_init__() 

env = ManagerBasedRLEnv(cfg=env_cfg)
obs, info = env.reset()

# Action: steer=0.0, throttle=1.0, brake=0.0
action = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)

print("Starting straight line test...")
for i in range(100):
    obs, rewards, terminated, truncated, info = env.step(action)
    
    # Extract telemetry
    lat_err = env.extras.get("lat_err", torch.tensor([0.0]))[0].item()
    
    # Isaac Lab linear velocity (local X is forward speed)
    lin_vel = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
    
    if i % 10 == 0:
        print(f"Step {i:3d} | LatErr: {lat_err:6.3f}m | Speed: {lin_vel:5.2f}m/s | Terminated: {terminated.item()}")
    
    if terminated.item():
        print(f"Terminated at step {i}! Reason: {info.get('termination_reason', 'unknown')}")
        break

env.close()
app_launcher.app.close()
