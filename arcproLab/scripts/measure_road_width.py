# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Measure road width by driving laterally until falling.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import numpy as np

# Add arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.append(ARCPRO_LAB_DIR)

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

def main():
    # 1. Setup Environment Configuration (1 env for precision)
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    # Disable cameras and visual observations
    env_cfg.enable_cameras = False
    env_cfg.observations.visual = None
    # Disable most terminations for measurement
    env_cfg.terminations.roadmark_contact = None
    env_cfg.terminations.stagnation = None
    env_cfg.terminations.driving_blind = None
    env_cfg.__post_init__()

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()
    
    # Wait for physics to settle
    for _ in range(50):
        env.sim.step()

    robot = env.scene["robot"]
    
    print("\n--- Road Width Measurement ---")
    print("Driving laterally (+X) until fall...")
    
    max_lat_pos = 0.0
    
    # 3. Measurement Loop (LEFT EDGE / +X)
    print("Testing LEFT (+X)...")
    left_edge = None
    for i in range(500):
        pos = robot.data.root_pos_w.clone()
        pos[0, 0] += 0.01 * i
        
        # Check height after stepping
        # (Teleport doesn't trigger collisions immediately, we check Z height)
        # In a real sim we'd step physics
        
        # Let's use a simpler method: probe Z height using a Raycast or just look at the stage
        # But we'll use the physics engine's ground truth by stepping
        from isaaclab.sim import SimulationContext
        sim = SimulationContext.instance()
        
        # Teleport and step
        robot.write_root_pose_to_sim(torch.cat([pos, robot.data.root_quat_w], dim=-1))
        sim.step()
        robot.update(sim.get_physics_dt())
        
        height = robot.data.root_pos_w[0, 2].item()
        if height < 0.03: # Falling
            left_edge = robot.data.root_pos_w[0, 0].item()
            print(f"LEFT FALL at X: {left_edge:.4f}")
            break
            
    # Reset
    env.reset()
    
    # 4. Measurement Loop (RIGHT EDGE / -X)
    print("\nTesting RIGHT (-X)...")
    right_edge = None
    for i in range(500):
        pos = robot.data.root_pos_w.clone()
        pos[0, 0] -= 0.01 * i
        
        robot.write_root_pose_to_sim(torch.cat([pos, robot.data.root_quat_w], dim=-1))
        env.sim.step()
        robot.update(env.sim.get_physics_dt())
        
        height = robot.data.root_pos_w[0, 2].item()
        if height < 0.03:
            right_edge = robot.data.root_pos_w[0, 0].item()
            print(f"RIGHT FALL at X: {right_edge:.4f}")
            break

    if left_edge and right_edge:
        width = left_edge - right_edge
        center = (left_edge + right_edge) / 2.0
        print(f"\n--- RESULTS ---")
        print(f"Total Road Width: {width:.3f}m")
        print(f"Physical Center X: {center:.4f}")
        print(f"Spawn X: {robot.data.root_pos_w[0, 0].item():.4f}")
        print(f"Spawn Offset from Center: {robot.data.root_pos_w[0, 0].item() - center:.4f}")

    simulation_app.close()

if __name__ == "__main__":
    main()
