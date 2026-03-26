# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check local transform.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def check_local(usd_path):
    stage = Usd.Stage.Open(usd_path)
    prim = stage.GetPrimAtPath("/Full_Car")
    if prim:
        xform = UsdGeom.Xformable(prim)
        print(f"Local transform for {prim.GetPath()}:")
        print(f"  Ops: {xform.GetOrderedXformOps()}")
        for op in xform.GetOrderedXformOps():
            print(f"  Op {op.GetOpName()}: {op.Get()}")
    
    # Check a wheel child too
    wheel = stage.GetPrimAtPath("/Full_Car/Rigid_Bodies/Wheel_Rear_Right")
    if wheel:
        xform = UsdGeom.Xformable(wheel)
        print(f"Local transform for {wheel.GetPath()}:")
        for op in xform.GetOrderedXformOps():
            print(f"  Op {op.GetOpName()}: {op.Get()}")

def main():
    check_local(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
