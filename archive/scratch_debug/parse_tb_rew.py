import sys
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import glob
files = glob.glob("logs/ppo_skrl/20260727-205933/*/*tfevents*")
ea = EventAccumulator(files[0])
ea.Reload()
events = ea.scalars.Items("Reward / Total reward (mean)")
for e in events[-20:]:
    print(f"Step {e.step}: {e.value}")
