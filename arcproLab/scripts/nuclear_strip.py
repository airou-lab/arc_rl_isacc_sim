# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Nuclear Strip and Flatten of Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def nuclear_strip_flatten(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # FLATTEN into a new stage
    flat_layer = stage.Flatten()
    flat_stage = Usd.Stage.CreateInMemory()
    flat_stage.GetRootLayer().transferContent = flat_layer
    # Wait, simple way:
    flat_layer.Export(output_path)
    print(f"Flattened layer exported to: {output_path}")
    
    # Now open the EXPORTED file and strip it
    print(f"Opening flattened stage for stripping: {output_path}")
    stage = Usd.Stage.Open(output_path)
    
    to_delete = []
    for prim in stage.GetPseudoRoot().GetChildren():
        name = prim.GetName()
        if name == "Full_Car":
            continue
        if name == "Render":
            continue
        print(f"Deleting top-level prim: {prim.GetPath()}")
        to_delete.append(prim.GetPath())

    for path in to_delete:
        stage.RemovePrim(path)

    stage.GetRootLayer().Save()
    print(f"Nuclear-stripped and Flattened USD saved to: {output_path}")

def main():
    nuclear_strip_flatten(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
