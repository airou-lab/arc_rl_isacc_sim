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
    
    # Disable terminations
    env_cfg.terminations.stagnation = None
    env_cfg.terminations.roadmark_contact = None
    env_cfg.terminations.driving_blind = None
    env_cfg.terminations.height = None
    
    env_cfg.__post_init__() 

    env = ManagerBasedRLEnv(cfg=env_cfg)

    # Test various offsets
    offsets = [0.0, -0.005, -0.01, -0.015, -0.02, 0.005, 0.01]
    
    for offset in offsets:
        env.action_manager._terms["steering"].cfg.offset = offset
        obs, _ = env.reset()
        
        start_pos = env.scene["robot"].data.root_pos_w.cpu().numpy()[0].copy()
        start_heading = env.scene["robot"].data.heading_w.cpu().numpy()[0]
        
        # Run 100 steps
        for _ in range(100):
            actions = torch.tensor([[0.0, 1.0, 0.0]], device=env.device)
            obs, rew, done, reset, extras = env.step(actions)
            if done[0]:
                break
            
        final_pos = env.scene["robot"].data.root_pos_w.cpu().numpy()[0]
        
        drift_x = final_pos[0] - start_pos[0]
        travel_y = final_pos[1] - start_pos[1]
        
        # Which way is it drifting?
        print(f"Offset: {offset:.3f} | Travel Y: {travel_y:.3f}m | Drift X: {drift_x:.6f}m")

    simulation_app.close()

if __name__ == "__main__":
    main()
