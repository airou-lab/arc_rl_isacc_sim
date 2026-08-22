import sys, glob
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
files = glob.glob("logs/ppo_skrl/*/*/*tfevents*")
files.sort(key=lambda x: x)
ea = EventAccumulator(files[-1])
ea.Reload()
try:
    events = ea.scalars.Items("Episode / Total timesteps (mean)")
    for e in events[-5:]:
        print(f"Step {e.step}: {e.value}")
except Exception as e:
    print(e)
