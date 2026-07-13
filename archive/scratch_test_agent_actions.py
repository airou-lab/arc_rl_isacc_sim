import sys
sys.path.insert(0, "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim")

import torch
from isaaclab.envs import ManagerBasedRLEnv
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from arcproLab.arcpro_env_cfg import ARCProEnvCfg
from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from arcproLab.scripts.train_skrl import SKRLFlattenWrapper, IsaacLabWrapper
from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = False # No need for cameras just to test telemetry output speed
env_cfg.__post_init__()

env = ManagerBasedRLEnv(cfg=env_cfg)
env = IsaacLabWrapper(env)
env = SKRLFlattenWrapper(env)

state_space = getattr(env, "state_space", env.observation_space)
models = {}
models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")
models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")

agent_cfg = PPO_DEFAULT_CONFIG.copy()
agent = PPO(models=models, memory=None, cfg=agent_cfg, observation_space=env.observation_space, action_space=env.action_space, device="cuda:0")

agent.load("logs/ppo_skrl/20260701-192503/26-07-01_19-25-03-529362_PPO/checkpoints/best_agent.pt")
agent.set_mode("eval")

obs, info = env.reset()
for i in range(10):
    with torch.no_grad():
        # format as SKRL dict to be 100% safe
        action = agent.act({"states": obs}, timestep=i, timesteps=10)[0]
    
    print(f"Step {i} | Action: {action.cpu().numpy().round(3)}")
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"  -> Reward: {reward.item():.3f} | Terminated: {terminated.item()}")

env.close()
simulation_app.close()
