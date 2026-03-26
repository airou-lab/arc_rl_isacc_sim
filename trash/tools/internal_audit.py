from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.enable_cameras = False
    env_cfg.scene.tiled_camera = None # Completely remove camera sensor
    env_cfg.observations.visual = None # Disable visual observations
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("\n--- INTERNAL PHYSICS AUDIT (Phase 7: Ultra-Stable) ---")
    print(f"Robot Spawn Z: {env_cfg.scene.robot.init_state.pos[2]}")
    print(f"Track Elevation Z: {env_cfg.scene.track.init_state.pos[2]}")
    
    # Check if the environment thinks the robot is fixed
    print(f"Is Fixed Base: {env.scene['robot'].is_fixed_base}")
    
    env.reset()
    
    for i in range(300):
        # Step simulation
        obs, reward, terminated, truncated, info = env.step(torch.zeros(env.action_manager.action.shape, device=env.device))
        
        # Get raw root state from the robot asset
        root_pos = env.scene["robot"].data.root_pos_w[0]
        root_vel = env.scene["robot"].data.root_lin_vel_w[0]
        
        if i % 20 == 0:
            print(f"Step {i:3d} | Z-Pos: {root_pos[2]:.4f}m | Z-Vel: {root_vel[2]:.4f}m/s | Terminated: {terminated.item()}")
            
    final_z = env.scene["robot"].data.root_pos_w[0, 2].item()
    print(f"\nAudit Finished. Final Z: {final_z:.4f}m")
    
    if final_z < 0:
        print("CRITICAL: Robot has fallen through the map into the void!")
    elif final_z < 15.0 and final_z > 9.0:
        print("SUCCESS: Robot appears to have landed on the track (Z~10-15m).")
    else:
        print(f"STATUS: Robot is currently at Z={final_z:.2f}m (Still falling or stuck).")

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
