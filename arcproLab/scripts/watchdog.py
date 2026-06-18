
import time
import os
import subprocess
import re

LOG_FILE = "logs/production_mastery_8_relaunch.log"
DIAGNOSIS_FILE = "logs/WATCHDOG_DIAGNOSIS.md"

def get_latest_metrics():
    if not os.path.exists(LOG_FILE):
        return None
    
    try:
        # Read last 200 lines to find the latest table
        tail = subprocess.check_output(["tail", "-n", "200", LOG_FILE]).decode('utf-8')
        
        timesteps = re.findall(r"total_timesteps\s+\|\s+([-\d\.e\+]+)", tail)
        ep_len = re.findall(r"ep_len_mean\s+\|\s+([-\d\.e\+]+)", tail)
        std = re.findall(r"std\s+\|\s+([-\d\.e\+]+)", tail)
        fps = re.findall(r"fps\s+\|\s+([-\d\.e\+]+)", tail)
        
        return {
            "timesteps": int(float(timesteps[-1])) if timesteps else 0,
            "ep_len": float(ep_len[-1]) if ep_len else 0,
            "std": float(std[-1]) if std else 0,
            "fps": float(fps[-1]) if fps else 0
        }
    except Exception as e:
        print(f"Error parsing log: {e}")
        return None

def main():
    print("Watchdog Active. Monitoring Phase 16 Training...")
    while True:
        metrics = get_latest_metrics()
        
        if metrics:
            t = metrics["timesteps"]
            e = metrics["ep_len"]
            s = metrics["std"]
            f = metrics["fps"]
            
            print(f"Status: T={t} | EpLen={e} | STD={s} | FPS={f}")
            
            # CRITICAL FAILURE CONDITIONS
            failure = None
            
            # 1. Divergence Check (STD Explosion)
            if s > 6.0:
                failure = f"DIVERGENCE: STD exploded to {s}. The neural network has gone insane."
            
            # 2. Stagnation Check (Glass Ceiling)
            elif t > 500000 and e < 200.0:
                failure = f"STAGNATION: Surviving only {e} steps after 500k timesteps. The agent is likely stuck in a crawling local minimum."
            
            # 3. Crash Check (Resource Lock)
            elif t > 10000 and f < 2:
                failure = f"SYSTEM CRASH: FPS dropped to {f}. Simulation is likely frozen or OOM."

            if failure:
                print(f"!!! CRITICAL FAILURE DETECTED: {failure}")
                # 1. Kill the training
                subprocess.run(["tmux", "kill-session", "-t", "training"])
                
                # 2. Write the report
                with open(DIAGNOSIS_FILE, "w") as f_out:
                    f_out.write(f"# WATCHDOG DIAGNOSIS\n\n**Failure Detected at T={t}**\n\n> {failure}\n\n**Action Taken**: Training session 'training' has been killed to save GPU time.\n\n**Next Steps**: Please ask Gemini to analyze the logs and apply a 'Stabilization Patch'.")
                
                print("Training killed. Diagnosis report written. Watchdog exiting.")
                break
        
        # Check every 15 minutes
        time.sleep(900)

if __name__ == "__main__":
    main()
