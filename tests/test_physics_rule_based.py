import torch
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Rule based physics test")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # Ensure camera flag propagates properly from AppLauncher's built-in argument
    env_cfg.enable_cameras = args_cli.enable_cameras
    # Re-run post-init to strip cameras if they are disabled
    env_cfg.__post_init__()
    # Increase episode length so we can observe it
    env_cfg.episode_length_s = 100.0
    
    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="human")
    
    obs, _ = env.reset()
    
    print("Testing rule-based driving...")
    print("Driving straight for 100 steps...")
    
    actions = torch.zeros((1, 2), device=env.device)
    
    # 0 = steering, 1 = drive
    for i in range(100):
        actions[:, 0] = 0.0 # straight
        actions[:, 1] = 0.5 # half throttle (approx 20 rad/s = 1 m/s)
        env.step(actions)
        
    print("Chassis pos after straight:", env.scene["robot"].data.root_pos_w)
    
    print("Turning right gently for 100 steps...")
    for i in range(100):
        actions[:, 0] = -0.5 # half steer right
        actions[:, 1] = 0.5 # half throttle
        env.step(actions)
        
    print("Chassis pos after turn:", env.scene["robot"].data.root_pos_w)
    print("Simulation complete. The physics are correct if it moved forward and then turned right.")

if __name__ == "__main__":
    main()
    simulation_app.close()
