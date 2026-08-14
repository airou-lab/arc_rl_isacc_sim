import sys
import os
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True, "num_envs": 1})
simulation_app = app_launcher.app

from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False
    env_cfg.__post_init__()
    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="rgb_array")
    
    obs, info = env.reset()
    
    from arcproLab.mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    
    # Run 20 steps forward
    for i in range(20):
        action = torch.zeros((1, 2), device=env.device)
        action[:, 1] = 1.0 # Drive
        env.step(action)
        
        pos = env.scene["robot"].data.root_pos_w
        quat = env.scene["robot"].data.root_quat_w
        
        # calculate yaw
        yaw = torch.atan2(2.0 * (quat[:, 0] * quat[:, 3] + quat[:, 1] * quat[:, 2]), 1.0 - 2.0 * (quat[:, 2]**2 + quat[:, 3]**2))
        
        dists = torch.cdist(pos[:, :2], tm.waypoints[:, :2])
        closest_idx = torch.argmin(dists, dim=1)
        wp_yaw = tm.waypoints[closest_idx, 2]
        
        head_err = yaw - wp_yaw
        
        track_dir = env.extras.get("track_dir", torch.zeros(1))[0].item()
        wps_cum = env.extras.get("cumulative_wp_index", torch.zeros(1))[0].item()
        
        print(f"Step {i}: idx={closest_idx[0].item()} yaw={yaw[0].item():.2f} wp_yaw={wp_yaw[0].item():.2f} err={head_err[0].item():.2f} dir={track_dir} wps_cum={wps_cum}")

if __name__ == "__main__":
    main()
    simulation_app.close()
