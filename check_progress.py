import os
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

def get_latest_scalars(log_path):
    # Initialize EventAccumulator with a small size to speed up loading
    event_acc = EventAccumulator(log_path, size_guidance={'scalars': 100})
    event_acc.Reload()
    
    tags = event_acc.Tags()['scalars']
    
    # Also check for 'eval/' tags if eval callback is used
    target_tags = [
        'rollout/ep_len_mean', 
        'rollout/ep_rew_mean', 
        'time/fps', 
        'train/learning_rate',
        'eval/mean_ep_length'
    ]
    
    print(f"--- Latest Metrics (Log: {os.path.basename(log_path)}) ---")
    found_any = False
    for tag in target_tags:
        if tag in tags:
            events = event_acc.Scalars(tag)
            if events:
                latest_event = events[-1]
                print(f"{tag:25}: {latest_event.value:>10.2f} (step {latest_event.step})")
                found_any = True
    
    if not found_any:
        print("No metrics found yet. Standard SB3 log frequency is every 2048 steps by default.")
        if tags:
            print(f"Available tags: {', '.join(tags[:10])}...")

if __name__ == "__main__":
    # Dynamically find the latest log directory
    ppo_dir = "logs/ppo"
    dirs = [os.path.join(ppo_dir, d) for d in os.listdir(ppo_dir) if os.path.isdir(os.path.join(ppo_dir, d))]
    if not dirs:
        print(f"No log directories found in {ppo_dir}")
        exit(1)
    
    # Get the latest directory based on name (timestamped)
    latest_dir = max(dirs)
    log_dir = os.path.join(latest_dir, "tb", "RecurrentPPO_1")
    
    if not os.path.exists(log_dir):
        # Fallback for older structures
        log_dir = os.path.join(latest_dir, "tb")
        if not os.path.exists(log_dir):
            log_dir = latest_dir

    files = [f for f in os.listdir(log_dir) if "tfevents" in f]
    if files:
        # Sort by mtime to get the latest
        files.sort(key=lambda x: os.path.getmtime(os.path.join(log_dir, x)))
        log_file = os.path.join(log_dir, files[-1])
        get_latest_scalars(log_file)
    else:
        print(f"No tfevents files found in {log_dir}")
