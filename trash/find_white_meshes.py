
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdShade

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Searching for White/Bright Meshes ---")
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        print(f"Mesh: {prim.GetPath()}")
        # Check material binding
        rel = prim.GetRelationship("material:binding")
        if rel.IsValid():
            targets = rel.GetTargets()
            if targets:
                mat_path = targets[0]
                print(f"  Bound to Material: {mat_path}")
                mat_prim = stage.GetPrimAtPath(mat_path)
                # Check for emissive in this specific material
                for child in mat_prim.GetChildren():
                    if child.IsA(UsdShade.Shader):
                        emissive = child.GetAttribute("inputs:emissive_color").Get()
                        diffuse = child.GetAttribute("inputs:diffuse_color").Get()
                        print(f"    Shader: {child.GetPath()} | Emissive: {emissive} | Diffuse: {diffuse}")
        else:
            # Check for displayColor if no material
            display_color = prim.GetAttribute("primvars:displayColor").Get()
            print(f"  No Material Binding. DisplayColor: {display_color}")

simulation_app.close()
