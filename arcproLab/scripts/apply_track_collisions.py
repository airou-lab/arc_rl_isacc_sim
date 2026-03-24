# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Add robust collisions and static rigid body to track.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema

def apply_track_collisions(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    print("Flattening stage...")
    flat_layer = stage.Flatten()
    flat_stage = Usd.Stage.Open(flat_layer)
    
    count = 0
    for prim in flat_stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName().lower()
            if "terrain" in name or "road" in name or "drivable" in prim.GetPath().pathString:
                print(f"Adding rigid collision to: {prim.GetPath()}")
                
                # Apply Rigid Body (Static)
                rb = UsdPhysics.RigidBodyAPI.Apply(prim)
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).GetKinematicEnabledAttr().Set(True)
                
                # Apply Collision
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_coll.GetApproximationAttr().Set("none")
                
                PhysxSchema.PhysxCollisionAPI.Apply(prim)
                
                count += 1

    flat_stage.GetRootLayer().Export(output_path)
    print(f"Applied rigid collisions to {count} meshes. Saved to: {output_path}")

def main():
    apply_track_collisions(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
