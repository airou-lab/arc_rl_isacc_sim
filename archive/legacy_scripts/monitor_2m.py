import time
import os
import subprocess
import re

LOG_FILE = "logs/curriculum_phase1.log"
TARGET_STEPS = 2000000
EXPECTED_EP_LEN = 400.0
EXPECTED_SPEED = 0.50

def get_latest_metrics():
    if not os.path.exists(LOG_FILE):
        raise FileNotFoundError(f"Monitor failed: {LOG_FILE} does not exist.")
    try:
        tail = subprocess.check_output(["tail", "-n", "300", LOG_FILE]).decode('utf-8')
        t = re.findall(r"total_timesteps\s+\|\s+([-\d\.e\+]+)", tail)
        e = re.findall(r"ep_len_mean\s+\|\s+([-\d\.e\+]+)", tail)
        s = re.findall(r"speed_mps\s+\|\s+([-\d\.e\+]+)", tail)
        
        return {
            "timesteps": int(float(t[-1])) if t else 0,
            "ep_len": float(e[-1]) if e else 0,
            "speed": float(s[-1]) if s else 0,
        }
    except Exception as e:
        raise ValueError(f"Monitor failed to parse log: {e}")

def main():
    print(f"Monitoring training until {TARGET_STEPS} steps...")
    while True:
        metrics = get_latest_metrics()
        if metrics:
            t = metrics["timesteps"]
            e = metrics["ep_len"]
            s = metrics["speed"]
            
            if t >= TARGET_STEPS:
                if e < EXPECTED_EP_LEN or s < EXPECTED_SPEED:
                    print(f"FAILURE_ALERT: At {t} steps, agent is underperforming! EpLen: {e} (Expected >{EXPECTED_EP_LEN}), Speed: {s} m/s (Expected >{EXPECTED_SPEED} m/s)")
                else:
                    print(f"SUCCESS_ALERT: At {t} steps, agent is crushing it! EpLen: {e}, Speed: {s} m/s")
                break
        
        time.sleep(300) # Check every 5 minutes

if __name__ == "__main__":
    main()
