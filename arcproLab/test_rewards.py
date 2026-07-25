import torch
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../"))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
sys.path.append(SCRIPT_DIR)

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True, "num_envs": 1})
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
cfg = ARCProEnvCfg()
cfg.scene.num_envs = 1
cfg.enable_cameras = False # Make it load fast
env = ManagerBasedRLEnv(cfg)

actions = torch.zeros((1, 2), device="cuda:0")
actions[:, 0] = 0.0 # Steer straight
actions[:, 1] = -1.0 # Brake (output -1.0 translates to 0.0 rad/s)

for _ in range(50):
    env.step(actions)

print("EPISODE SUMS (at 50 steps):")
for name, val in env.reward_manager.episode_sums.items():
    print(f"{name}: {val.item()}")

simulation_app.close()
