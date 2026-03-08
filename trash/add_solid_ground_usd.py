
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

def add_ground_usd():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    ground_path = "/World/SolidGroundUSD"
    if not stage.GetPrimAtPath(ground_path).IsValid():
        print(f"Adding Solid USD Plane at z=15.14...")
        plane = UsdGeom.Mesh.Define(stage, ground_path)
        size = 1000.0
        plane.CreatePointsAttr([(-size, -size, 15.14), (size, -size, 15.14), (size, size, 15.14), (-size, size, 15.14)])
        plane.CreateFaceVertexCountsAttr([4])
        plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        
        # Add Physics
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
        # Make it a static collider
        # In Isaac Sim, a mesh with CollisionAPI but NO RigidBodyAPI is a static collider
    else:
        print("Solid USD plane already exists.")

    omni.usd.get_context().save_stage()
    print("World updated with SolidGroundUSD.")

add_ground_usd()
simulation_app.close()
