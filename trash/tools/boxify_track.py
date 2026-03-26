# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Replace track meshes with box colliders.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, Gf

def boxify_track(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    print("Flattening stage...")
    flat_layer = stage.Flatten()
    flat_stage = Usd.Stage.Open(flat_layer)
    
    count = 0
    for prim in flat_stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName().lower()
            if "terrain" in name:
                print(f"Converting mesh to box collider: {prim.GetPath()}")
                
                # Compute BBox
                bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
                bbox = bbox_cache.ComputeLocalBound(prim, Usd.TimeCode.Default(), "default")
                range = bbox.GetRange()
                size = range.GetMax() - range.GetMin()
                center = range.GetMidpoint()
                
                # Ensure some thickness
                if size[2] < 0.01:
                    size = Gf.Vec3d(size[0], size[1], 0.1) # 10cm thick
                
                # Remove Mesh specifics and turn into Xform
                # Actually, just add CollisionAPI and use 'box' approximation
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_coll.GetApproximationAttr().Set("boundingCube")
                
                count += 1

    flat_stage.GetRootLayer().Export(output_path)
    print(f"Applied box approximation to {count} meshes. Saved to: {output_path}")

def main():
    boxify_track(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
