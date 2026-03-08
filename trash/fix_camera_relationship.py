
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, Sdf

def fix_relationship():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    print(f"Opening {path} for relationship fix...")
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    prim_path = "/World/F1Tenth/ActionGraph/setCamera"
    camera_path = "/World/F1Tenth/Rigid_Bodies/Chassis/Camera_Left"
    
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        rel = prim.GetRelationship("inputs:cameraPrim")
        if rel.IsValid():
            print(f"Adding target {camera_path} to relationship...")
            # Clear existing and add new
            rel.SetTargets([])
            rel.AddTarget(Sdf.Path(camera_path))
            
            # Save
            omni.usd.get_context().save_stage()
            print("SUCCESS: cameraPrim relationship set via USD API.")
        else:
            print("Relationship invalid.")
    else:
        print("Prim invalid.")

fix_relationship()
simulation_app.close()
