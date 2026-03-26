# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Set USD Stage Units.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
parser.add_argument("--mpu", type=float, default=0.01, help="Meters Per Unit.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def set_units(usd_path, output_path, mpu):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # Set Meters Per Unit
    UsdGeom.SetStageMetersPerUnit(stage, mpu)
    print(f"Set Meters Per Unit to: {mpu}")
    
    stage.GetRootLayer().Export(output_path)
    print(f"Updated USD saved to: {output_path}")

def main():
    set_units(args_cli.usd_path, args_cli.output_path, args_cli.mpu)
    simulation_app.close()

if __name__ == "__main__":
    main()
