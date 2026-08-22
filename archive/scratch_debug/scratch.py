import sys
import os
sys.path.append(os.path.join(os.getcwd(), 'arcproLab'))
from arcpro_env_cfg import ARCProEnvCfg
cfg = ARCProEnvCfg()
print(f"Stationary weight: {cfg.rewards.stationary.weight}")
print(f"Progress weight: {cfg.rewards.progress_reward.weight}")
