import torch
import os
import sys

# Add the parent directory to sys.path to find arcproLab modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def test_env():
    print("Initializing environment...")
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 2
    env_cfg.headless = True
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    print("Environment initialized successfully.")
    
    obs, info = env.reset()
    print(f"Initial observation keys: {obs.keys()}")
    
    for i in range(5):
        print(f"Step {i}...")
        # Random actions: (num_envs, action_dim) -> (2, 3) [steering, throttle, brake]
        actions = torch.rand((2, 3), device=env.device) * 2.0 - 1.0
        obs, reward, terminated, truncated, info = env.step(actions)
        print(f"  Reward: {reward}")
    
    print("Test completed successfully.")
    env.close()

if __name__ == "__main__":
    test_env()
