from isaaclab.app import AppLauncher
import argparse
import sys
import os

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

# Add arcproLab to path
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
sys.path.append(ROOT_DIR)
sys.path.append(ARCPRO_LAB_DIR)

import torch
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedEnv

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # We need to ensure physics is initialized to check contacts
    env = ManagerBasedEnv(cfg=env_cfg)
    
    robot = env.scene["robot"]
    print("\n--- STUCK STATE VERIFICATION ---")
    
    # 1. Check Mass Properties in the actual simulation backend
    print("\n[Simulation Backend Mass Properties]")
    # Get properties from the PhysX view
    masses = robot.root_physx_view.get_masses()[0]
    coms = robot.root_physx_view.get_coms()[0]
    inertias = robot.root_physx_view.get_inertias()[0]
    
    for i, body_name in enumerate(robot.body_names):
        print(f"Body: {body_name}")
        print(f"  Sim Mass: {masses[i].item():.3f} kg")
        print(f"  Sim CoM: {coms[i].cpu().numpy()}")
        # Check if CoM is valid (not NaN or Inf)
        if torch.isinf(coms[i]).any() or torch.isnan(coms[i]).any():
            print("  [!] CRITICAL: CoM is INVALID (Inf/NaN). Physics will explode.")
    # 2. Check for Movement (Root velocity check)
    print("\n[Movement Audit - 10 Steps]")
    env.reset()
    for step in range(10):
        # We must step simulation to populate data buffers
        env.step(torch.zeros_like(env.action_manager.action))
        
        z_pos = robot.data.root_pos_w[0, 2].item()
        vel = torch.norm(robot.data.root_lin_vel_w[0]).item()
        print(f"Step {step} | Z-Pos: {z_pos:.3f} | Vel: {vel:.2f} m/s")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
