
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n--- Forcing Black Floor ---")

# 1. Delete all current floors
paths_to_delete = ["/World/RobustFloor", "/FlatGrid/GroundPlane", "/FlatGrid/Environment"]
for path in paths_to_delete:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        print(f"Deleting {path}...")
        stage.RemovePrim(path)

# 2. Spawn a NEW Black Floor
floor_path = "/World/NewBlackFloor"
if not stage.GetPrimAtPath(floor_path).IsValid():
    print("Creating NEW Black Plane...")
    mesh = UsdGeom.Mesh.Define(stage, floor_path)
    
    # Create a large plane (100x100)
    size = 100.0
    mesh.CreatePointsAttr([(-size, -size, 0), (size, -size, 0), (size, size, 0), (-size, size, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    
    # Set display color to Black
    color_attr = mesh.CreateDisplayColorAttr()
    color_attr.Set([Gf.Vec3f(0.05, 0.05, 0.05)]) # Dark gray

# Save stage
omni.usd.get_context().save_stage()
print("\nBlack floor created and stage saved.")

simulation_app.close()
