# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Disable Self-Collisions in Robot USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema

def disable_self_collisions(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    root_prim = stage.GetPrimAtPath("/Full_Car")
    if root_prim:
        articulation = UsdPhysics.ArticulationRootAPI.Apply(root_prim)
        physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(root_prim)
        physx_articulation.GetEnabledSelfCollisionsAttr().Set(False)
        print("Disabled self-collisions on /Full_Car")

    # Also, we can disable collisions on nested meshes or rigid bodies
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            # We can use collision groups to isolate components
            pass

    stage.GetRootLayer().Export(output_path)
    print(f"Self-collisions disabled and saved to: {output_path}")

def main():
    disable_self_collisions(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
