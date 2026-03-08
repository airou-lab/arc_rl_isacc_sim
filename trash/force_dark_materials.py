
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdShade, Gf

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Forcing Dark Materials ---")
for prim in stage.Traverse():
    if prim.IsA(UsdShade.Shader):
        # 1. Force Diffuse Color to Dark Gray (0.1, 0.1, 0.1)
        diffuse = prim.GetAttribute("inputs:diffuse_color")
        if diffuse.IsValid():
            print(f"Updating diffuse for {prim.GetPath()}")
            diffuse.Set(Gf.Vec3f(0.1, 0.1, 0.1))
        
        # 2. Check for "base_color" (common in some MDLs)
        base_color = prim.GetAttribute("inputs:base_color")
        if base_color.IsValid():
            print(f"Updating base_color for {prim.GetPath()}")
            base_color.Set(Gf.Vec3f(0.1, 0.1, 0.1))

        # 3. Double check Emissive
        emissive = prim.GetAttribute("inputs:emissive_color")
        if emissive.IsValid():
            emissive.Set(Gf.Vec3f(0.0, 0.0, 0.0))

# Save stage
omni.usd.get_context().save_stage()
print("\nMaterials darkened and stage saved.")

simulation_app.close()
