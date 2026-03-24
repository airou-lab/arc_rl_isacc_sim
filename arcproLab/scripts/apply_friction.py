# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Apply High Friction Material to Wheels.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema, UsdShade

def apply_friction(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. Create a Physics Material
    material_path = "/Full_Car/HighFrictionMaterial"
    material = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path, "Material"))
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    
    # Set high friction
    material.GetStaticFrictionAttr().Set(2.0)
    material.GetDynamicFrictionAttr().Set(1.5)
    material.GetRestitutionAttr().Set(0.0)
    
    # 2. Apply to wheels
    for prim in stage.Traverse():
        if "Wheel" in prim.GetName() and prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"Applying high friction to: {prim.GetPath()}")
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(UsdShade.Material(material.GetPrim()), UsdShade.Tokens.weakerThanDescendants, "physics")

    # Save as new file
    stage.GetRootLayer().Export(output_path)
    print(f"Friction-applied USD saved to: {output_path}")

def main():
    apply_friction(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
