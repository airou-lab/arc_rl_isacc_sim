
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.graph.core as og

def check_node():
    path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/arcpro_RL_open_street_sim.usd"
    omni.usd.get_context().open_stage(path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    
    node_path = "/World/F1Tenth/ActionGraph/setCamera"
    node = og.get_node_by_path(node_path)
    if node:
        print(f"\n--- Attributes for {node_path} ---")
        for attr in node.get_attributes():
            print(f" Name: {attr.get_name()} | Type: {attr.get_type_name()}")
    else:
        print("Node not found.")

check_node()
simulation_app.close()
