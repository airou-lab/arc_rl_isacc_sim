import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check initial lateral error.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.mdp.track_manager import get_track_manager

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    tm = get_track_manager(device=env.device)
    asset = env.scene["robot"]
    
    # Run a few steps to let reset happen
    for _ in range(2):
        env.step(torch.zeros((1, 6), device=env.device))
        
    pos = asset.data.root_pos_w
    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))
    
    lat_err, head_err = tm.compute_errors(pos, yaw)
    
    print("\n" + "="*60)
    print(f"INITIAL POSE: X={pos[0,0]:.4f}, Y={pos[0,1]:.4f}, Z={pos[0,2]:.4f}")
    print(f"LATERAL ERROR: {lat_err[0].item():.4f} meters")
    print(f"HEADING ERROR: {head_err[0].item():.4f} radians")
    print("="*60 + "\n")
    
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
