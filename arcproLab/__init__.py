import gymnasium as gym
import warnings
from .arcpro_env_cfg import ARCProEnvCfg

# Silence the 'Gym unmaintained' warning coming from external libraries like SB3/Isaac Lab.
# We explicitly target the legacy gym namespace warning while keeping Gymnasium warnings.
warnings.filterwarnings("ignore", category=UserWarning, message=".*Gym has been unmaintained since 2022.*")

# Explicit registration via Gymnasium
gym.register(
    id="ARCPro-RL-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "cfg_entry_point": ARCProEnvCfg,
    },
)
