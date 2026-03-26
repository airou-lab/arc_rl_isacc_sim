# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Generate a GSD-grade hardened track.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics

def generate_gsd_track(output_path):
    factor = 2.0
    
    # Create new stage
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    root_prim = UsdGeom.Xform.Define(stage, "/World/Track")
    stage.SetDefaultPrim(root_prim.GetPrim())

    # 1. Visual Reference (Original track, scaled 2x)
    visual_node = UsdGeom.Xform.Define(stage, "/World/Track/Visuals")
    visual_ref = visual_node.GetPrim().GetReferences()
    abs_source = os.path.abspath("openStreetUSD/street_sim_repaired.usd")
    visual_ref.AddReference(abs_source)
    # Scale only the visual node
    scale_op = None
    for op in visual_node.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    if not scale_op:
        scale_op = visual_node.AddScaleOp()
    scale_op.Set(Gf.Vec3f(factor))

    # 2. Physics Hardening (A single, solid Mesh instead of a GroundPlane)
    # This allows direct navigation on the map footprint without falling.
    collision_node = UsdGeom.Mesh.Define(stage, "/World/Track/CollisionMesh")
    
    # Create a large rectangle (2 points per edge)
    # Map is roughly 12x12 at 1x, so at 2x it's 24x24.
    size = 30.0 # meters
    points = [
        Gf.Vec3f(-size, -size, 0),
        Gf.Vec3f( size, -size, 0),
        Gf.Vec3f( size,  size, 0),
        Gf.Vec3f(-size,  size, 0)
    ]
    collision_mesh = UsdGeom.Mesh(collision_node)
    collision_mesh.GetPointsAttr().Set(points)
    collision_mesh.GetFaceVertexCountsAttr().Set([4])
    collision_mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    
    # Apply standard USD Physics
    UsdPhysics.CollisionAPI.Apply(collision_node.GetPrim())
    # Use direct mesh collision
    mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(collision_node.GetPrim())
    mesh_coll.GetApproximationAttr().Set("none")
    
    # Make it invisible
    UsdGeom.Imageable(collision_node).MakeInvisible()

    stage.GetRootLayer().Save()
    print(f"GSD Hardened Track saved to: {output_path}")

def main():
    generate_gsd_track(args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
