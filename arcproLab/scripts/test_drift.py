import argparse
import torch
import os
import sys

# Add arcproLab directory to sys path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from isaaclab.app import AppLauncher

# Launch Isaac Sim
app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False 
    
    env_cfg.__post_init__() 

    env = ManagerBasedRLEnv(cfg=env_cfg)

    # Reset
    obs, _ = env.reset()

    print("Initial Position:", env.scene["robot"].data.root_pos_w.cpu().numpy()[0])
    
    final_pos = env.scene["robot"].data.root_pos_w.cpu().numpy()[0]
    
    # Run 200 steps
    for step in range(200):
        actions = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)
        obs, rew, done, reset, extras = env.step(actions)
        if done[0]:
            print(f"Terminated early at step {step}!")
            break
        final_pos = env.scene["robot"].data.root_pos_w.cpu().numpy()[0].copy()

    print("Final Position (before reset):", final_pos)
    print("Distance Traveled (Y):", final_pos[1] - 4.92 )
    print("Drift in X:", final_pos[0] - (-15.73))

    simulation_app.close()

if __name__ == "__main__":
    main()
