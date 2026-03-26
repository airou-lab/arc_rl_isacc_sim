# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check joint bodies.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

def check_joint_bodies(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    def traverse(prim):
        name = prim.GetName()
        type_name = prim.GetTypeName()
        if "Joint" in type_name:
            joint = UsdPhysics.Joint(prim)
            body0 = joint.GetBody0Rel().GetTargets()
            body1 = joint.GetBody1Rel().GetTargets()
            print(f"Joint: {prim.GetPath()} ({type_name})")
            print(f"  Body0: {body0[0] if body0 else 'None'}")
            print(f"  Body1: {body1[0] if body1 else 'None'}")
        
        for child in prim.GetChildren():
            traverse(child)

    traverse(stage.GetPseudoRoot())

def main():
    check_joint_bodies(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
