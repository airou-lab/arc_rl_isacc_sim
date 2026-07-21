import torch
import glob
from tensorboard.backend.event_processing import event_accumulator

log_dirs = sorted(glob.glob('/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs/ppo_skrl/*/*'))
ea = event_accumulator.EventAccumulator(log_dirs[-1])
ea.Reload()
rew_tot = ea.Scalars('Reward / Total reward (mean)')[-1].value
rew_inst = ea.Scalars('Reward / Instantaneous reward (mean)')[-1].value
len_mean = ea.Scalars('Episode / Total timesteps (mean)')[-1].value

print(f"Total Reward (mean): {rew_tot}")
print(f"Instantaneous Reward (mean): {rew_inst}")
print(f"Episode Length (mean): {len_mean}")
print(f"Inst * Len = {rew_inst * len_mean}")
