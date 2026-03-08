
import os
from isaacsim import SimulationApp

# Start simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

def inspect_street():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    print(f"Loading World0: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    street_prim = stage.GetPrimAtPath("/World/Street")
    
    if not street_prim.IsValid():
        print("ERROR: /World/Street prim does not exist!")
        return

    print(f"\n--- Inspecting /World/Street ---")
    print(f"Prim Type: {street_prim.GetTypeName()}")
    
    # Check for children
    children = street_prim.GetChildren()
    print(f"Number of direct children: {len(children)}")
    for child in children:
        print(f" - {child.GetPath()} ({child.GetTypeName()})")

    # If it's a reference, it might not show children until fully resolved
    # Let's check for references
    if street_prim.HasAuthoredReferences():
        print("Authored References found.")
    
    # Check visibility
    im_vis = UsdGeom.Imageable(street_prim).GetVisibilityAttr().Get()
    print(f"Visibility: {im_vis}")

    simulation_app.close()

if __name__ == "__main__":
    inspect_street()
