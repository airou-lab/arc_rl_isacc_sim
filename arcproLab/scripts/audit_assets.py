# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to list all prims in the stage to find road markers."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="List all prims in the USD stage.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
from pxr import Usd, UsdGeom

def main():
    # Load the stage
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/no_graph_sim_clean_1x.usda"
    omni.usd.get_context().open_stage(usd_path)
    stage = omni.usd.get_context().get_stage()
    
    if stage is None:
        print(f"Failed to load stage at {usd_path}")
        simulation_app.close()
        return
    
    print(f"--- Listing Prims in {usd_path} ---")
    count = 0
    from pxr import UsdShade
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        # Check for roadmarks OR robot parts
        if (("roadmark" in path.lower() or "piece" in path.lower()) and prim.IsA(UsdGeom.Mesh)) or ("robot" in path.lower()):
            # Check material binding (only for meshes)
            mat_name = "N/A"
            if prim.IsA(UsdGeom.Mesh):
                binding_api = UsdShade.MaterialBindingAPI(prim)
                material, _ = binding_api.ComputeBoundMaterial()
                mat_name = str(material.GetPath()) if material else "None"
            
            print(f"[{prim.GetTypeName()}] {path} | Material: {mat_name}")
            count += 1
            if count > 200:
                print("... (Truncated)")
                break
    
    print(f"Total relevant prims found: {count}")
    simulation_app.close()

if __name__ == "__main__":
    main()
