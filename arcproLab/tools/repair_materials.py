# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Repair broken material connections in a USD file.")
parser.add_argument("usd_path", type=str, help="Path to the USD file to repair.")
parser.add_argument("output_path", type=str, help="Path to save the repaired USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdShade, Sdf, Gf

def repair_materials(usd_path, output_path):
    print(f"Opening stage to repair: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if prim.IsA(UsdShade.Material):
            mat = UsdShade.Material(prim)
            surface_output = mat.GetSurfaceOutput()
            if not surface_output or not surface_output.GetConnectedSource():
                print(f"Repairing broken material: {prim.GetPath()}")
                
                # Determine color based on name (heuristic)
                name = prim.GetName().lower()
                color = (0.2, 0.2, 0.2) # Default dark grey
                if "yellow" in name or "marker" in name:
                    color = (1.0, 0.8, 0.0)
                elif "white" in name:
                    color = (0.9, 0.9, 0.9)
                elif "grass" in name or "terrain" in name:
                    color = (0.1, 0.3, 0.1)
                
                # Create a fresh shader
                shader_path = prim.GetPath().AppendPath("RepairedShader")
                shader = UsdShade.Shader.Define(stage, shader_path)
                shader.CreateIdAttr("UsdPreviewSurface")
                shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
                
                # Connect it properly using the ConnectableAPI
                mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    stage.GetRootLayer().Export(output_path)
    print(f"Repaired USD saved to: {output_path}")

def main():
    repair_materials(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
