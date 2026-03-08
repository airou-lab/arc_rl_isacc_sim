
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdLux

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Extreme Lighting Fix ---")
for prim in stage.Traverse():
    # 1. Check for any Lux schema (Light, DomeLight, etc.)
    if "Lux" in prim.GetTypeName():
        print(f"Found Light Prim: {prim.GetPath()} ({prim.GetTypeName()})")
        for attr in prim.GetAttributes():
            if attr.GetName() in ["intensity", "exposure"]:
                print(f"  Setting {attr.GetName()} to 1.0/0.0")
                if attr.GetName() == "intensity":
                    attr.Set(1.0)
                else:
                    attr.Set(0.0)
    
    # 2. Check for manual intensity/exposure attributes on NON-Lux prims
    else:
        intensity_attr = prim.GetAttribute("intensity")
        if intensity_attr.IsValid():
             print(f"Found intensity attr on non-lux prim: {prim.GetPath()}")
             intensity_attr.Set(1.0)

# Save stage
omni.usd.get_context().save_stage()
print("\nLighting zeroed out and stage saved.")

simulation_app.close()
