
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf

def find_spawn():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    print("\n--- Sampling Ground Height ---")
    
    drivable = stage.GetPrimAtPath("/World/drivable_surfaces")
    if not drivable.IsValid():
        print("Drivable surfaces not found.")
        return

    # Sample a 5x5m grid around (0,0) including (0,0)
    for x in [-2, -1, 0, 1, 2]:
        for y in [-2, -1, 0, 1, 2]:
            highest_z = -1000
            found = False
            for prim in Usd.PrimRange(drivable):
                if prim.IsA(UsdGeom.Mesh):
                    bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
                    bbox_range = bbox.GetRange()
                    if bbox_range.GetMin()[0] <= x <= bbox_range.GetMax()[0] and bbox_range.GetMin()[1] <= y <= bbox_range.GetMax()[1]:
                        # Check collision
                        from pxr import UsdPhysics
                        has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)
                        highest_z = max(highest_z, bbox_range.GetMax()[2])
                        found = True
                        if x == 0 and y == 0:
                            print(f"Mesh at (0,0): {prim.GetPath()} | Collision: {has_collision}")
            if found:
                print(f"Point ({x}, {y}) | Max Z hint: {highest_z}")

find_spawn()
simulation_app.close()
