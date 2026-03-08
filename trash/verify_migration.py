
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

def verify():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    print(f"\n--- Final Verification in {path} ---")
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    robot = stage.GetPrimAtPath("/World/F1Tenth")
    if robot.IsValid():
        print("SUCCESS: F1Tenth found.")
    
    node_path = "/World/F1Tenth/ActionGraph/setCamera"
    node_prim = stage.GetPrimAtPath(node_path)
    if node_prim.IsValid():
        rel = node_prim.GetRelationship("inputs:cameraPrim")
        targets = rel.GetTargets()
        print(f"ActionGraph Node Found: {node_path}")
        print(f"cameraPrim Targets: {targets}")
        if targets and len(targets) > 0:
            print("SUCCESS: cameraPrim relationship is VALID.")
        else:
            print("FAILED: cameraPrim relationship is EMPTY.")

verify()
simulation_app.close()
