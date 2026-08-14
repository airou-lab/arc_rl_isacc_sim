import sys, glob
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
files = glob.glob("logs/ppo_skrl/*/*/*tfevents*")
files.sort(key=lambda x: x)
ea = EventAccumulator(files[-1])
ea.Reload()
events = ea.scalars.Items("Policy / Standard deviation")
for e in events[-5:]:
    print(f"Step {e.step}: {e.value}")
