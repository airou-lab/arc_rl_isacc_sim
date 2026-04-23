# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Visualize DSLaneGate prims in the GUI.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# Ensure GUI is enabled
args_cli.headless = False

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import omni.usd
from pxr import Usd, UsdGeom, Gf
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
import isaaclab.sim as sim_utils

# Add both root and arcproLab to sys.path
import os
import sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

def main():
    # 1. Setup environment
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.__post_init__()
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # 2. Setup Markers
    gate_marker_cfg = VisualizationMarkersCfg(
        prim_path="/World/Visuals/GateMarkers",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.5, # Large enough to see clearly
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)) # Green
            )
        }
    )
    gate_visualizer = VisualizationMarkers(gate_marker_cfg)

    # 3. Find Gates
    stage = omni.usd.get_context().get_stage()
    gate_positions = []
    
    # We need to find the gates in world space
    # In Isaac Lab, envs are usually under /World/envs/env_N
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        # Check for ds_type primvar
        if prim.HasAttribute("primvars:ds_type"):
            ds_type = prim.GetAttribute("primvars:ds_type").Get()
            if ds_type == ["DSLaneGate"]:
                # Get world position
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pos = world_transform.ExtractTranslation()
                gate_positions.append([pos[0], pos[1], pos[2] + 1.0]) # Float it slightly above ground
                print(f"[FOUND GATE] Path: {prim.GetPath()} | Pos: {pos}")

    if not gate_positions:
        print("CRITICAL: No DSLaneGates found in the scene!")
    else:
        gate_tensor = torch.tensor(gate_positions, device="cuda:0")
        print(f"Visualizing {len(gate_positions)} gates...")

    # 4. Loop
    while simulation_app.is_running():
        if gate_positions:
            gate_visualizer.visualize(gate_tensor)
        env.sim.step()
        simulation_app.update()

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
