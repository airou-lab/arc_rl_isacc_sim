# Diagnostic to find gate positions
import os
import sys
from isaaclab.app import AppLauncher

# Configuration for headless run
sys.argv.append("--headless")
launcher = AppLauncher()
simulation_app = launcher.app

import torch
import os

# Add project root and arcproLab to path
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(PROJECT_DIR, "arcproLab")

for path in [PROJECT_DIR, ARCPRO_LAB_DIR]:
    if path not in sys.path:
        sys.path.append(path)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from arcproLab.mdp.track_manager import get_track_manager

# 1. Setup environment to load the USD
env_cfg = ARCProEnvCfg()
env_cfg.scene.num_envs = 1
env_cfg.enable_cameras = False # Disable for headless search
env_cfg.__post_init__()
env = ManagerBasedRLEnv(cfg=env_cfg)

# 2. Initialize TrackManager
tm = get_track_manager(device=env.device)
tm.ensure_synced()

if tm.raw_gate_pts is not None:
    print("\n--- DISCOVERED GATES ---")
    for i, pt in enumerate(tm.raw_gate_pts):
        print(f"Gate {i}: {pt}")
else:
    print("No gates found.")

env.close()
simulation_app.close()
