
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Gf

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Rebuilding Lighting from Scratch ---")

# 1. Delete problematic environment dome and existing lights
paths_to_delete = ["/FlatGrid/Environment", "/FlatGrid/SphereLight", "/World/DistantLight"]
for path in paths_to_delete:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        print(f"Deleting {path}...")
        stage.RemovePrim(path)

# 2. Add ONE clean light
light_path = "/World/CleanLight"
if not stage.GetPrimAtPath(light_path).IsValid():
    print("Adding Clean DistantLight...")
    light = UsdLux.DistantLight.Define(stage, light_path)
    light.CreateIntensityAttr(1000.0)
    light.CreateAngleAttr(0.5)
    xform = UsdGeom.Xformable(light)
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, -45.0, 0.0))

# 3. Force Materials to be non-emissive and neutral
for prim in stage.Traverse():
    if prim.IsA(UsdShade.Shader):
        emissive = prim.GetAttribute("inputs:emissive_color")
        if emissive.IsValid():
            emissive.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        
        diffuse = prim.GetAttribute("inputs:diffuse_color")
        if diffuse.IsValid():
            diffuse.Set(Gf.Vec3f(0.5, 0.5, 0.5))

# Save stage
omni.usd.get_context().save_stage()
print("\nLighting rebuilt and stage saved.")

simulation_app.close()
