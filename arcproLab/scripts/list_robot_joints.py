import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="List all joint names and indices.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg

# Add arcproLab to path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_robot_cfg import ARCPRO_ROBOT_CFG

def main():
    # Reset sim
    sim_dt = 0.01
    sim = SimulationContext(SimulationCfg(dt=sim_dt, device="cuda:0"))
    
    # Load robot
    robot_cfg = ARCPRO_ROBOT_CFG.replace(prim_path="/World/Robot")
    robot = Articulation(robot_cfg)
    
    # Spawn
    robot_cfg.spawn.func(robot_cfg.prim_path, robot_cfg.spawn)
    
    sim.reset()
    # Manual initialization of the view if needed, but robot.data should exist after reset
    
    print("\n" + "="*60)
    print("JOINT AUDIT")
    print("="*60)
    
    # Try getting from data
    joint_names = robot.data.joint_names
    print(f"Total Joints: {len(joint_names)}")
    for i, name in enumerate(joint_names):
        print(f"Index {i}: {name}")
    print("="*60 + "\n")

    simulation_app.close()

if __name__ == "__main__":
    main()
