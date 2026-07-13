import torch
import os
import sys
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.__post_init__()
env = ManagerBasedRLEnv(cfg=env_cfg)

obs, _ = env.reset()
for i in range(50):
    # Random actions to see what happens to velocity and terminations
    action = torch.rand((1, 3), device=env.device) * 2 - 1.0 # [-1, 1]
    obs, rew, term, trunc, info = env.step(action)
    
    pos = env.scene["robot"].data.root_pos_w[0].cpu().numpy()
    vel = torch.norm(env.scene["robot"].data.root_lin_vel_b[0, :2]).item()
    print(f"Step {i}: Pos: {pos}, Vel: {vel:.3f} m/s, Term: {term[0].item()}, Rew: {rew[0].item():.3f}")

env.close()
