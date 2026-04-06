from isaaclab.app import AppLauncher
import argparse
import sys
import os

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

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
    env = ManagerBasedEnv(cfg=env_cfg)
    
    robot = env.scene["robot"]
    print("\n--- ROBOT JOINT AUDIT ---")
    print(f"Total Joints: {robot.num_joints}")
    print(f"Joint Names: {robot.joint_names}")
    
    # Check which joints are actually 'actuated'
    # We can look at the actuator groups
    for name, group in robot.actuators.items():
        print(f"\nActuator Group: {name}")
        # Group is an instance of ActuatorBase
        # We can find which joints it's handling
        indices, _ = robot.find_joints(group.cfg.joint_names_expr)
        matched_names = [robot.joint_names[i] for i in indices]
        print(f"  Matches: {matched_names}")
        print(f"  Effort Limit: {group.cfg.effort_limit_sim}")
        print(f"  Stiffness: {group.cfg.stiffness}")
        print(f"  Damping: {group.cfg.damping}")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
