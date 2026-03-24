# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Permanently scale a USD map.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import os

def main():
    input_usd = "openStreetUSD/arcpro_RL_open_street_sim.usd"
    output_usd = "openStreetUSD/arcpro_RL_open_street_sim_scaled.usd"
    scale_factor = 0.02803 # Target 1.5m road width
    
    print(f"\n[Scaler] Opening: {input_usd}")
    stage = Usd.Stage.Open(input_usd)
    
    # Apply scale to the root Xformable (usually /World)
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim:
        print("[Scaler] ERR: Could not find /World prim.")
        simulation_app.close()
        return

    xform = UsdGeom.Xformable(world_prim)
    # Clear existing ops and set a single scale op
    xform.ClearXformOpOrder()
    xform.AddScaleOp().Set(Gf.Vec3d(scale_factor, scale_factor, scale_factor))
    
    print(f"[Scaler] Applied {scale_factor} scale to /World")
    stage.Export(output_usd)
    print(f"[Scaler] Exported scaled map to: {output_usd}")

    simulation_app.close()

if __name__ == "__main__":
    main()
