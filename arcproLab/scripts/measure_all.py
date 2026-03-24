# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure all meshes in USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def measure_all(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    def traverse(prim):
        if prim.IsA(UsdGeom.Mesh):
            bbox = UsdGeom.Imageable(prim).ComputeLocalBound(Usd.TimeCode.Default(), "default")
            box_range = bbox.GetRange()
            size = box_range.GetMax() - box_range.GetMin()
            
            xformable = UsdGeom.Xformable(prim)
            ops = xformable.GetOrderedXformOps()
            
            print(f"\nMesh: {prim.GetPath()}")
            print(f"  Local Size: {size}")
            
            # Print cumulative scale
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            # Matrix4d has no ExtractScale, but we can compute it from basis vectors
            sx = Gf.Vec3d(world_transform[0][0], world_transform[0][1], world_transform[0][2]).GetLength()
            sy = Gf.Vec3d(world_transform[1][0], world_transform[1][1], world_transform[1][2]).GetLength()
            sz = Gf.Vec3d(world_transform[2][0], world_transform[2][1], world_transform[2][2]).GetLength()
            print(f"  World Scale: ({sx}, {sy}, {sz})")
            print(f"  World Size (units): ({size[0]*sx}, {size[1]*sy}, {size[2]*sz})")
            print(f"  World Translation: {world_transform.ExtractTranslation()}")

        for child in prim.GetChildren():
            traverse(child)

    traverse(stage.GetPseudoRoot())

def main():
    measure_all(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
