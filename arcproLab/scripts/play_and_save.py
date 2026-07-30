import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play SKRL ARCPro Policy and Save Frames")
parser.add_argument("--checkpoint", type=str, default=None)
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys, os, cv2, numpy as np, torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

if args_cli.checkpoint is None:
    log_dir = os.path.join(ROOT_DIR, "logs", "ppo_skrl")
    c = []
    for r, d, f in os.walk(log_dir):
        for n in f:
            if n.endswith(".pt"): c.append(os.path.join(r, n))
    args_cli.checkpoint = max(c, key=os.path.getmtime) if c else None

from isaaclab.envs import ManagerBasedRLEnv
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from skrl.envs.wrappers.torch import IsaacLabWrapper
from agents.skrl_wrappers import SKRLFlattenWrapper

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = True
env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="rgb_array")

import gymnasium as gym
obs_raw_dict = {}
class FrameCaptureWrapper(gym.Wrapper):
    def step(self, action):
        o, r, d, t, i = self.env.step(action)
        obs_raw_dict["camera"] = o["policy"]["tiled_camera"].clone()
        return o, r, d, t, i
    def reset(self, **kwargs):
        o, i = self.env.reset(**kwargs)
        obs_raw_dict["camera"] = o["policy"]["tiled_camera"].clone()
        return o, i

env = FrameCaptureWrapper(env)
env = IsaacLabWrapper(env)
env = SKRLFlattenWrapper(env)

models = {
    "policy": ARCProActor(env.observation_space, env.action_space, "cuda:0"),
    "value": ARCProCritic(getattr(env, "state_space", env.observation_space), env.action_space, "cuda:0")
}
agent_cfg = PPO_DEFAULT_CONFIG.copy()
agent_cfg["experiment"]["write_interval"] = 0
agent_cfg["experiment"]["checkpoint_interval"] = 0
agent = PPO(models=models, memory=None, cfg=agent_cfg, observation_space=env.observation_space, action_space=env.action_space, device="cuda:0")
agent.load(args_cli.checkpoint)
agent.set_mode("eval")

obs, info = env.reset()
os.makedirs("debug_frames", exist_ok=True)
for step in range(200):
    with torch.no_grad():
        action = agent.act(obs, timestep=0, timesteps=0)[0]
    obs, r, d, t, i = env.step(action)
    
    if step % 20 == 0:
        img = obs_raw_dict["camera"].cpu().numpy()[0]
        img_bgr = cv2.cvtColor((np.clip(img, 0, 1) * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
        cv2.imwrite(f"debug_frames/eval_frame_{step}.png", img_bgr)
        print(f"Saved eval_frame_{step}.png")

simulation_app.close()
