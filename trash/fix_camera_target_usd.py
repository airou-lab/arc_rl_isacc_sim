
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, Sdf

def fix_target():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    print(f"Opening {path} for target fix...")
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    attr_path = "/World/F1Tenth/ActionGraph/setCamera.inputs:cameraPrim"
    prim_path = "/World/F1Tenth/ActionGraph/setCamera"
    camera_path = "/World/F1Tenth/Rigid_Bodies/Chassis/Camera_Left"
    
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        attr = prim.GetAttribute("inputs:cameraPrim")
        if attr.IsValid():
            print(f"Setting {attr_path} to {camera_path}")
            # For target attributes, we set a list of Sdf.Path
            attr.Set([Sdf.Path(camera_path)])
            
            # Save
            omni.usd.get_context().save_stage()
            print("SUCCESS: cameraPrim target set via USD API.")
        else:
            print("Attribute invalid.")
    else:
        print("Prim invalid.")

fix_target()
simulation_app.close()
