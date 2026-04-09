import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="List all joints for the robot.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    robot = env.scene["robot"]
    print("\n" + "="*60)
    print("ROBOT JOINT AUDIT")
    print("="*60)
    
    joint_names = robot.joint_names
    print(f"Total Joints: {len(joint_names)}")
    for i, name in enumerate(joint_names):
        print(f"Index {i}: {name}")
    
    print("\nAction Terms Mapping:")
    for term_name, term in env.action_manager._action_terms.items():
        print(f"  {term_name}: {term.joint_indices}")

    print("="*60 + "\n")
    
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
