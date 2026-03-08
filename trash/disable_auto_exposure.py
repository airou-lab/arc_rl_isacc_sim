
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Disabling Auto-Exposure and Bloom ---")
# Post-processing settings are often stored in /Render or /Render/PostProcess
render_prim = stage.GetPrimAtPath("/Render")
if not render_prim.IsValid():
    render_prim = stage.GetPrimAtPath("/RenderSettings")

for prim in stage.Traverse():
    if "PostProcess" in prim.GetName():
        print(f"Checking PostProcess prim: {prim.GetPath()}")
        # Auto-exposure attributes
        for attr in prim.GetAttributes():
            if "exposure" in attr.GetName().lower() or "auto" in attr.GetName().lower():
                print(f"  Found attribute: {attr.GetName()}")
                if "enabled" in attr.GetName().lower():
                    attr.Set(False)
                elif "intensity" in attr.GetName().lower():
                    attr.Set(1.0) # Reset to neutral

# Also use omni.kit.commands to set common render settings
import omni.kit.commands
omni.kit.commands.execute("ChangeProperty", prop_path="/Render/PostProcess/AutoExposure/enabled", value=False, prev=True)
omni.kit.commands.execute("ChangeProperty", prop_path="/Render/PostProcess/Bloom/enabled", value=False, prev=True)

# Save stage
omni.usd.get_context().save_stage()
print("\nPost-processing updated and stage saved.")

simulation_app.close()
