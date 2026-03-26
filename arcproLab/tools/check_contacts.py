import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check robot contacts.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from isaaclab.envs import ManagerBasedEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    # setup configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.__post_init__() 
    
    # setup environment
    env = ManagerBasedEnv(cfg=env_cfg)
    robot = env.scene["robot"]
    
    # reset environment to initialize data
    env.reset()
    
    print("\nRunning 100 steps to observe settling...")
    for s in range(100):
        env.step(torch.zeros((1, 6), device=env.device))
        if s % 20 == 0:
            root_z = robot.data.root_pos_w[0, 2].item()
            print(f"Step {s:3d} | Root Z: {root_z:.4f} m")
    
    # Check body heights
    body_pos_w = robot.data.body_pos_w[0]
    body_names = robot.data.body_names
    
    print("\n--- Final Body Heights (Z) ---")
    for i, name in enumerate(body_names):
        print(f"{name:30s}: {body_pos_w[i, 2].item():.4f} m")

    # Check contact forces
    forces = robot.data.net_contact_forces[0]
    print("\n--- Net Contact Forces (N) ---")
    found_contact = False
    for i, name in enumerate(body_names):
        f_norm = torch.norm(forces[i]).item()
        if f_norm > 0.1:
            print(f"{name:30s}: {f_norm:.2f} N")
            found_contact = True
    if not found_contact:
        print("No significant contact forces detected.")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
