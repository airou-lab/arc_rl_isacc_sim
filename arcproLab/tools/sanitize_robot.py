# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Sanitize Robot USD Scales.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def sanitize_robot(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. Force MetersPerUnit to 0.01 (centimeters)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    print("Set Meters Per Unit to 0.01")

    # 2. Remove any scale ops from /Full_Car
    root_prim = stage.GetPrimAtPath("/Full_Car")
    if root_prim:
        xform = UsdGeom.Xformable(root_prim)
        new_order = []
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                print(f"Removing scale op from {root_prim.GetPath()}: {op.Get()}")
                continue
            new_order.append(op)
        
        # Correct API to set the order
        xform.SetXformOpOrder(new_order)

    # 3. Check internal prims for scales
    for prim in stage.TraverseAll():
        if prim.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    val = op.Get()
                    if val and (val[0] != 1.0 or val[1] != 1.0 or val[2] != 1.0):
                        print(f"Warning: Internal scale found on {prim.GetPath()}: {val}")

    # 4. Save
    stage.GetRootLayer().Export(output_path)
    print(f"Sanitized USD saved to: {output_path}")

def main():
    sanitize_robot(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
