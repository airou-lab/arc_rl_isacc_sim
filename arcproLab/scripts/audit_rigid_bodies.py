# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Audit Rigid Bodies in Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def audit_rigid_bodies(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"\nRigid Body: {prim.GetPath()} ({prim.GetTypeName()})")
            
            # Check for Mass
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
                print(f"  Mass: {mass}")
            else:
                print("  [WARNING] No Mass API found.")

def main():
    audit_rigid_bodies(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
