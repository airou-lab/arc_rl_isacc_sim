# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check USD Units.")
parser.add_argument("usd_path", type=str, help="Path to the USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def check_units(usd_path):
    stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    print(f"\nAsset: {usd_path}")
    print(f"Meters Per Unit: {mpu}")
    print(f"Up Axis: {up}")

def main():
    check_units(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
