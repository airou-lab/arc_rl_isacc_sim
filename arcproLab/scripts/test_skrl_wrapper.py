import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg
from skrl.envs.wrappers.torch import IsaacLabWrapper

env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = True 
env_cfg.__post_init__() 

env = ManagerBasedRLEnv(cfg=env_cfg)
wrapped_env = IsaacLabWrapper(env)

print("Original Obs Space:", env.observation_space)
print("Wrapped Obs Space:", wrapped_env.observation_space)

obs, _ = wrapped_env.reset()
print("Wrapped Obs Type:", type(obs))
if isinstance(obs, dict):
    for k, v in obs.items():
        if isinstance(v, dict):
            print(f"  {k}: Dict with keys {list(v.keys())}")
        else:
            print(f"  {k}: {v.shape}")
else:
    print("Wrapped Obs Shape:", obs.shape)

app_launcher.app.close()
