import sys, glob
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
files = glob.glob("logs/ppo_skrl/*/*/*tfevents*")
files.sort(key=lambda x: x)
ea = EventAccumulator(files[-1])
ea.Reload()

def get_latest(key):
    try:
        events = ea.scalars.Items(key)
        return events[-1].value
    except:
        return None

keys = [
    "Episode / Total timesteps (mean)",
    "Reward / Total reward (mean)",
    "Telemetry / Speed_MPS"
]
for k in keys:
    print(f"{k}: {get_latest(k)}")
