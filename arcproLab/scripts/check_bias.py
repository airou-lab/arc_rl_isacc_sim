import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check for steering/physical bias.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
# Add arcproLab to path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import euler_xyz_from_quat
from arcpro_env_cfg import ARCProEnvCfg

def main():
    log_file = "bias_report.txt"
    with open(log_file, "w") as f:
        f.write("Starting Bias Report\n")
        f.flush()
        
        print("[Bias Check] Starting environment creation...")
        env_cfg = ARCProEnvCfg()
        env_cfg.scene.num_envs = 1
        
        try:
            env = ManagerBasedRLEnv(cfg=env_cfg)
            print("[Bias Check] Environment created.")
        except Exception as e:
            f.write(f"FAILED to create environment: {e}\n")
            print(f"[Bias Check] FAILED to create environment: {e}")
            simulation_app.close()
            return

        f.write("="*60 + "\n")
        f.write("PHYSICAL BIAS AUDIT\n")
        f.write("="*60 + "\n")
        
        print("[Bias Check] Resetting environment...")
        env.reset()
        
        # Action [Steer, Throttle, (Unused)]
        action = torch.zeros((1, 3), device=env.device)
        action[0, 1] = 0.5 
        
        f.write(f"Applying action: {action}\n")
        
        for i in range(101):
            env.step(action)
            
            if i % 10 == 0:
                root_pos = env.scene["robot"].data.root_pos_w[0]
                quat = env.scene["robot"].data.root_quat_w[0]
                _, _, yaw = euler_xyz_from_quat(quat.unsqueeze(0))
                yaw_deg = yaw.item() * 180.0 / 3.14159
                out = f"Step {i:3d} | Pos: {root_pos[0].item():8.4f}, {root_pos[1].item():8.4f}, {root_pos[2].item():8.4f} | Yaw: {yaw_deg:8.4f} deg\n"
                f.write(out)
                f.flush()
                print(out, end="")

        final_y = env.scene["robot"].data.root_pos_w[0, 1].item()
        f.write("-"*60 + "\n")
        if abs(final_y) > 0.05:
            direction = "RIGHT" if final_y < 0 else "LEFT"
            f.write(f"BIAS DETECTED: Car drifts {direction} (Y pos: {final_y:.4f})\n")
        else:
            f.write(f"NO SIGNIFICANT BIAS: Y position is {final_y:.4f}\n")
        
        f.write("="*60 + "\n")
        f.flush()

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
