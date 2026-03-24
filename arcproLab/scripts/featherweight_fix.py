# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Featherweight Fix for Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema, Gf

def featherweight_fix(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        # 1. Set all mass to be very small
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            print(f"  Lightweighting: {prim.GetPath()}")
            # Set tiny mass for all components except chassis
            if "Chassis" in prim.GetName():
                mass_api.GetMassAttr().Set(1.0)
            else:
                mass_api.GetMassAttr().Set(0.1)
            
            # Clear inertia to let it be recomputed
            mass_api.GetDiagonalInertiaAttr().Clear()

        # 2. Force all joints to have NO friction and high effort
        if prim.IsA(UsdPhysics.Joint):
            if prim.HasAPI(PhysxSchema.PhysxJointAPI):
                px_joint = PhysxSchema.PhysxJointAPI(prim)
                px_joint.GetJointFrictionAttr().Set(0.0)
            
            drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive_api.GetMaxForceAttr().Set(10000.0) # OVERPOWERED

    # Save
    stage.GetRootLayer().Export(output_path)
    print(f"Featherweight USD saved to: {output_path}")

def main():
    featherweight_fix(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
