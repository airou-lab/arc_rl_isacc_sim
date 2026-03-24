# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="GSD Phase 7: Repair materials and apply high-fidelity 3D collisions.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to save the repaired USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf, UsdPhysics, PhysxSchema

def repair_and_solidify_gsd(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. Ensure absolute paths for materials aren't broken
    # (Optional: we could check for missing textures here)

    for prim in stage.Traverse():
        # --- MATERIAL REPAIR ---
        if prim.IsA(UsdShade.Material):
            mat = UsdShade.Material(prim)
            surface_output = mat.GetSurfaceOutput()
            # If the material has no shader connected, it will render grey.
            if not surface_output or not surface_output.GetConnectedSource():
                print(f"Repairing broken material connection: {prim.GetPath()}")
                
                # Heuristic color based on name
                name = prim.GetName().lower()
                color = (0.1, 0.1, 0.1) # Asphalt
                if "yellow" in name or "marker" in name:
                    color = (1.0, 0.8, 0.0)
                elif "white" in name:
                    color = (0.9, 0.9, 0.9)
                elif "grass" in name or "terrain" in name:
                    color = (0.1, 0.3, 0.1)
                
                shader_path = prim.GetPath().AppendPath("GsdShader")
                shader = UsdShade.Shader.Define(stage, shader_path)
                shader.CreateIdAttr("UsdPreviewSurface")
                shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
                mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # --- PHYSICS HARDENING ---
        if prim.IsA(UsdGeom.Mesh):
            # Apply standard Physics
            UsdPhysics.CollisionAPI.Apply(prim)
            
            # Use Convex Decomposition for accurate 3D terrain interaction (hills, curbs)
            mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_coll.GetApproximationAttr().Set("convexDecomposition")
            
            # Set high-quality decomposition settings
            physx_decomp = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            physx_decomp.GetVoxelResolutionAttr().Set(500000)
            physx_decomp.GetErrorPercentageAttr().Set(1.0)

    print(f"Exporting GSD-grade asset to: {output_path}")
    stage.GetRootLayer().Export(output_path)

def main():
    repair_and_solidify_gsd(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
