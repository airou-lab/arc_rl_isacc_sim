import gymnasium as gym
from .arcpro_env_cfg import ARCProEnvCfg

gym.register(
    id="ARCPro-RL-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "cfg_entry_point": ARCProEnvCfg,
    },
)
