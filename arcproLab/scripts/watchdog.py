
import time
import os
import subprocess
import re

LOG_FILE = "logs/skrl_phase1.log"
DIAGNOSIS_FILE = "logs/WATCHDOG_DIAGNOSIS.md"

def get_latest_metrics():
    if not os.path.exists(LOG_FILE):
        raise FileNotFoundError(f"Watchdog failed: {LOG_FILE} does not exist.")
    
    try:
        # Read last 200 lines to find the latest table
        tail = subprocess.check_output(["tail", "-n", "200", LOG_FILE]).decode('utf-8')
        
        # Parse custom print format: Step {timestep} | Rew: {rew} | Len: {len} | Spd: {spd} | WPs_cum: {wp_cum} | WPΔ_rew: {wp_step}
        timesteps = re.findall(r"Step (\d+)", tail)
        ep_len = re.findall(r"Len: (\d+)", tail)
        speed = re.findall(r"Spd: ([-\d\.]+)", tail)
        # We don't have std and fps in the custom print, so default to None
        std = []
        fps = []
        
        return {
            "timesteps": int(float(timesteps[-1])) if timesteps else 0,
            "ep_len": float(ep_len[-1]) if ep_len else 0,
            "std": float(std[-1]) if std else None,
            "fps": float(fps[-1]) if fps else None,
            "speed": float(speed[-1]) if speed else 0
        }
    except Exception as e:
        raise ValueError(f"Watchdog failed to parse log: {e}")

def main():
    print("Watchdog Active. Monitoring Phase 16 Training...")
    last_reported_milestone = 0
    while True:
        metrics = get_latest_metrics()
        
        if metrics:
            t = metrics["timesteps"]
            e = metrics["ep_len"]
            s = metrics["std"]
            f = metrics["fps"]
            v = metrics["speed"]
            
            print(f"Status: T={t} | EpLen={e} | STD={s} | FPS={f} | Speed={v}")
            
            # 0. Log Telemetry to SQLite DB (Zero LLM Token Cost)
            try:
                db_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.gsd/archive.db"))
                import sqlite3
                import json
                conn = sqlite3.connect(db_path)
                c = conn.cursor()
                metrics_json = json.dumps({"timesteps": t, "ep_len": e, "std": s, "fps": f, "speed": v})
                c.execute("INSERT INTO telemetry_events (event_type, metrics) VALUES (?, ?)", ("TELEMETRY_HEARTBEAT", metrics_json))
                conn.commit()
                conn.close()
            except Exception as db_e:
                print(f"Failed to log to SQLite: {db_e}")
            
            
            # CRITICAL FAILURE CONDITIONS
            failure = None
            
            # 1. Divergence Check (STD Explosion)
            if s is not None and s > 6.0:
                failure = f"DIVERGENCE: STD exploded to {s}. The neural network has gone insane."
            
            # 2. Stagnation Check (Glass Ceiling)
            elif t > 100000 and v < 0.01:
                failure = f"STAGNATION: Agent stopped moving (speed {v} m/s) after {t} timesteps."
            elif t > 500000 and e < 100.0:
                failure = f"STAGNATION: Surviving only {e} steps after 500k timesteps. The agent is likely stuck in a crawling local minimum."
            
            # 3. Crash Check (Resource Lock - only checked when FPS metric exists)
            elif f is not None and t > 10000 and f < 2:
                failure = f"SYSTEM CRASH: FPS dropped to {f}. Simulation is likely frozen or OOM."

            if failure:
                print(f"!!! CRITICAL FAILURE DETECTED: {failure}")
                
                # 1. Write the report FIRST (so it doesn't kill itself before writing)
                with open(DIAGNOSIS_FILE, "w") as f_out:
                    f_out.write(f"# WATCHDOG DIAGNOSIS\n\n**Failure Detected at T={t}**\n\n> {failure}\n\n**Action Taken**: Training session 'training' has been killed to save GPU time.\n\n**Next Steps**: Please ask Gemini to analyze the logs and apply a 'Stabilization Patch'.")
                
                # ACTUALLY KILL THE TRAINING
                os.system("tmux kill-session -t training 2>/dev/null; pkill -f train_skrl.py; pkill -f train_policy.py")
                
                # 2. Call Antigravity CLI to autonomously analyze, fix, and restart
                print("Summoning Antigravity agent to autofix...")
                prompt = (
                    f"RL Training failed: {failure}. Read logs/WATCHDOG_DIAGNOSIS.md. "
                    "Modify arcproLab/arcpro_env_cfg.py to fix the rewards/penalties, "
                    "then run 'bash start_tmux_training.sh' to restart training."
                )
                
                os.system(f"nohup agy run --prompt '{prompt}' > /dev/null 2>&1 &")
                
                print("Training will be restarted by the agent. Watchdog exiting.")
                break
        
        # Report progress every 300k steps using agy CLI
        if metrics:
            current_milestone = metrics["timesteps"] // 300000
            if current_milestone > last_reported_milestone and metrics["timesteps"] >= 300000:
                print(f"Reporting {current_milestone * 300}k milestone to Antigravity...")
                prompt = f"RL Training Progress Report: Agent reached {metrics['timesteps']} timesteps! Current Speed: {metrics['speed']} m/s, EpLen: {metrics['ep_len']}. The strict target is EpLen >= 600. Please analyze if it is stalling below this target and if we should intervene or let it continue."
                os.system(f"nohup agy run --prompt '{prompt}' > /dev/null 2>&1 &")
                last_reported_milestone = current_milestone

        # Check every 15 minutes
        time.sleep(900)

if __name__ == "__main__":
    main()
