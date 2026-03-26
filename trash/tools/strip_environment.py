# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Strip Environment from Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def strip_environment(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    to_delete = []
    for prim in stage.GetPseudoRoot().GetChildren():
        name = prim.GetName()
        # Keep ONLY the robot prim (Full_Car)
        if name != "Full_Car":
            print(f"Marking for deletion: {name}")
            to_delete.append(prim.GetPath())

    for path in to_delete:
        stage.RemovePrim(path)

    # Save as new file
    stage.GetRootLayer().Export(output_path)
    print(f"Stripped USD saved to: {output_path}")

def main():
    strip_environment(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
