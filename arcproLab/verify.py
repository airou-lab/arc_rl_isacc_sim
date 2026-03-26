"""
Verification script for ARCPro RL Isaac Lab environment.
This script loads the environment and runs a simple loop to verify spawning and physics.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Verify ARCPro RL Environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest of the imports"""
import torch
import gymnasium as gym
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

# Manual registration
gym.register(
    id="ARCPro-RL-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ARCProEnvCfg,
    },
)

def main():
    # parse configuration
    env_cfg: ARCProEnvCfg = parse_env_cfg("ARCPro-RL-v1", device=args_cli.device, num_envs=args_cli.num_envs)
    
    # create environment
    env = ManagerBasedRLEnv(cfg=env_cfg)

    print("[INFO]: Environment created successfully.")

    # reset environment
    obs, _ = env.reset()
    
    # simulate
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # sample random actions
            actions = 2.0 * torch.rand(env.num_envs, env.action_manager.total_action_dim, device=env.device) - 1.0
            # apply actions
            obs, rewards, terminations, truncations, infos = env.step(actions)
            
            count += 1
            if count % 100 == 0:
                print(f"Step {count} completed.")
                
            if count > 500:
                break

    # close the environment
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
