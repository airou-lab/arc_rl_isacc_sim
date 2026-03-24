# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to generate track waypoints from USD stage."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Generate track waypoints from USD stage.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import torch
import numpy as np
import matplotlib.pyplot as plt
import sys

# Add parent directory to sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext, SimulationCfg
from arcpro_env_cfg import ARCProSceneCfg
from mdp.track_manager import get_track_manager

def main():
    # Setup simulation context
    sim_cfg = SimulationCfg(dt=1.0/60.0, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    
    # Setup scene with the track
    scene_cfg = ARCProSceneCfg(num_envs=1, env_spacing=5.0)
    # Disable camera to avoid rendering overhead
    scene_cfg.tiled_camera = None
    scene = InteractiveScene(scene_cfg)
    
    # Reset to ensure everything is spawned
    sim.reset()
    
    # Initialize TrackManager
    tm = get_track_manager()
    
    # Force sampling to ensure we get the latest from USD
    print("[Info] Sampling waypoints from USD stage...")
    tm.sample_waypoints_from_usd(density=0.1) # Higher density for scaled map
    
    # Save waypoints
    wp_path = os.path.join(os.path.dirname(__file__), "mdp", "track_centerline.npy")
    tm.save_waypoints(wp_path)
    
    # Visualization
    wps = tm.waypoints.cpu().numpy()
    plt.figure(figsize=(10, 10))
    plt.scatter(wps[:, 0], wps[:, 1], c=wps[:, 2], cmap='hsv', s=2)
    plt.colorbar(label='Yaw (rad)')
    plt.title(f"Generated Track Waypoints ({len(wps)} points)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis('equal')
    
    plot_path = os.path.join(os.path.dirname(__file__), "track_verification.png")
    plt.savefig(plot_path)
    print(f"[Info] Verification plot saved to {plot_path}")
    
    # Close the simulator
    simulation_app.close()

if __name__ == "__main__":
    main()
