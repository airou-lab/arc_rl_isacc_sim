# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Nuclear Center Robot Geometry.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def nuclear_center(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # Measured Step 0 offset in Isaac Lab: (-12.43, -14.97, 5.863)
    # This corresponds to CM values: (-1243, -1497, 586)
    # We apply the inverse to the ROOT prim
    root_prim = stage.GetPrimAtPath("/Full_Car")
    xform = UsdGeom.Xformable(root_prim)
    xform.ClearXformOpOrder()
    
    # In USD (CM), we need to move by +1243, +1497, -586
    offset = Gf.Vec3d(1243.0, 1497.0, -586.0)
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(offset)
    
    stage.GetRootLayer().Export(output_path)
    print(f"Nuclear-centered USD saved to: {output_path}")

def main():
    nuclear_center(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
