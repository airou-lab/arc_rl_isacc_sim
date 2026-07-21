import os
import sys
import torch
import numpy as np

# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True, "num_envs": 1})
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = False
env_cfg.__post_init__()

env = ManagerBasedRLEnv(cfg=env_cfg)
obs = env.reset()

# Step 10 times to let it fall and settle
for _ in range(10):
    env.step(torch.zeros((1, 2), device=env.device))

# Now apply full throttle (action=1.0) and see which way it moves
print("Applying action = [0.0, 1.0]...")
actions = torch.tensor([[0.0, 1.0]], device=env.device)
for _ in range(50):
    env.step(actions)

vel = env.scene["robot"].data.root_lin_vel_b[0]
print(f"Velocity after 50 steps of action=1.0: X={vel[0]:.3f}, Y={vel[1]:.3f}, Z={vel[2]:.3f}")

# Now apply negative throttle
print("Applying action = [0.0, -1.0]...")
actions = torch.tensor([[0.0, -1.0]], device=env.device)
for _ in range(50):
    env.step(actions)

vel = env.scene["robot"].data.root_lin_vel_b[0]
print(f"Velocity after 50 steps of action=-1.0: X={vel[0]:.3f}, Y={vel[1]:.3f}, Z={vel[2]:.3f}")

env.close()
simulation_app.close()
