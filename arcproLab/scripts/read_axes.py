# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Read joint axes from USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def read_axes(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            rev = UsdPhysics.RevoluteJoint(prim)
            print(f"Joint: {prim.GetPath()}")
            print(f"  Axis: {rev.GetAxisAttr().Get()}")

def main():
    read_axes(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
