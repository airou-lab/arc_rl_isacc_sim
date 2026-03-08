
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdShade

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Inspecting Materials ---")
for prim in stage.Traverse():
    if prim.IsA(UsdShade.Material):
        print(f"Material: {prim.GetPath()}")
        # Check for shaders
        for child in prim.GetChildren():
            if child.IsA(UsdShade.Shader):
                print(f"  Shader: {child.GetPath()}")
                # Check emissive color
                emissive = child.GetAttribute("inputs:emissive_color")
                if emissive.IsValid():
                    print(f"    Emissive Color: {emissive.Get()}")
                
                diffuse = child.GetAttribute("inputs:diffuse_color")
                if diffuse.IsValid():
                    print(f"    Diffuse Color: {diffuse.Get()}")

simulation_app.close()
