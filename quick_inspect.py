
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()

def traverse(prim, depth=0):
    indent = "  " * depth
    name = prim.GetName()
    path = prim.GetPath()
    type_name = prim.GetTypeName()
    
    # Filter for interesting types (Xform, Robot, Articulation, Physics)
    if type_name in ["Xform", "PhysicsScene", "DistantLight", "Camera", "PhysicsFixedJoint", "PhysicsRevoluteJoint"]:
        print(f"{indent}{name} ({type_name}) -> {path}")
    
    # Look specifically for wheels or joints
    if "wheel" in name.lower() or "joint" in name.lower() or "steer" in name.lower() or "throttle" in name.lower():
        print(f"{indent}*** INTERESTING: {name} ({type_name}) -> {path}")

    for child in prim.GetChildren():
        traverse(child, depth + 1)

if stage:
    traverse(stage.GetPseudoRoot())
else:
    print("Failed to load stage.")

simulation_app.close()
