import sys, glob
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
files = glob.glob("logs/ppo_skrl/*/*/*tfevents*")
if not files:
    print("No files found.")
    sys.exit(1)
files.sort(key=lambda x: x)
ea = EventAccumulator(files[-1])
ea.Reload()
print("\n".join(ea.scalars.Keys()))
