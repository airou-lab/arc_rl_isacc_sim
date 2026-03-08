
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

print("\n--- Fixing Emissive Floor ---")
floor_shader_path = "/FlatGrid/Looks/theGrid/Shader"
shader_prim = stage.GetPrimAtPath(floor_shader_path)

if shader_prim.IsValid():
    print(f"Found floor shader at {floor_shader_path}")
    emissive_attr = shader_prim.GetAttribute("inputs:emissive_color")
    if emissive_attr.IsValid():
        # Set emissive to Black
        from pxr import Gf
        emissive_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        print("  Emissive Color set to (0, 0, 0)")
    else:
        print("  Attribute inputs:emissive_color not found.")
else:
    print(f"Floor shader NOT found at {floor_shader_path}")

# Save stage
omni.usd.get_context().save_stage()
print("\nFloor fixed and stage saved.")

simulation_app.close()
