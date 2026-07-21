import torch
import sys
import os
import numpy as np
import gymnasium as gym

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from skrl.envs.wrappers.torch import IsaacLabWrapper, Wrapper

from agents.skrl_wrappers import SKRLFlattenWrapper

def main():
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 4
    env_cfg.enable_cameras = False
    env_cfg.__post_init__()
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)
    
    obs, _ = env.reset()
    
    for i in range(20):
        actions = torch.rand(4, 3, device=env.device) * 2.0 - 1.0
        obs, rew, terminated, truncated, extras = env.step(actions)
        
        if terminated.any():
            print(f"Step {i}: TERMINATED={terminated.flatten().tolist()}")
            print("Termination triggers:")
            # Unpack the wrappers to get to the original env
            base_env = env._env._env
            for term, val in base_env.termination_manager._term_dones.items():
                if val.any():
                    print(f" - {term}: {val.flatten().tolist()}")
                    
    simulation_app.close()

if __name__ == "__main__":
    main()
