# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to check mass distribution of the robot articulation.
"""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check robot mass distribution.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg

# Add arcproLab to path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from arcpro_robot_cfg import ARCPRO_ROBOT_CFG

def main():
    # Load robot config
    robot_cfg = ARCPRO_ROBOT_CFG.replace(prim_path="/World/Robot")
    
    # Reset sim
    sim_dt = 0.01
    sim = SimulationContext(SimulationCfg(dt=sim_dt, device="cuda:0"))
    
    # Ground plane
    sim_utils.spawn_ground_plane("/World/GroundPlane", sim_utils.GroundPlaneCfg())
    
    # Spawn robot
    robot_cfg.spawn.func(robot_cfg.prim_path, robot_cfg.spawn, translation=(0.0, 0.0, 1.0))
    
    # Create the articulation object to access body names
    robot = Articulation(robot_cfg)
    
    sim.reset()
    # robot.initialize() - Not needed if sim.reset() handles it or using physx directly
    
    print("\n" + "="*50)
    print("ROBOT MASS DISTRIBUTION AUDIT")
    print("="*50)
    
    # body_names might only be available after initialization, 
    # but we can get them from the prim itself if needed.
    # Let's try getting them from the articulation object first.
    try:
        body_names = robot.body_names
    except:
        # Fallback: manual USD traversal
        import omni.usd
        from pxr import UsdPhysics, Usd
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath("/World/Robot")
        body_names = []
        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                body_names.append(str(prim.GetPath()).replace("/World/Robot/", ""))

    # Link-by-link audit via USD
    from pxr import UsdPhysics
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    total_mass = 0.0
    for i, name in enumerate(body_names):
        prim_path = f"/World/Robot/{name}"
        prim = stage.GetPrimAtPath(prim_path)
        mass = 0.0
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            mass = mass_api.GetMassAttr().Get()
        
        # If mass is 0 in USD, it might be computed from density
        print(f"Body: {name:30} | Mass (USD): {mass:10.4f} kg")
        total_mass += mass

    print("-"*50)
    print(f"Total Static Mass (USD): {total_mass:.4f} kg")
    print("="*50 + "\n")

    simulation_app.close()

if __name__ == "__main__":
    main()
