import sys
import os

# Set up paths properly
PROJECT_ROOT = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim"
sys.path.append(PROJECT_ROOT)

import argparse
from isaaclab.app import AppLauncher

# Generate a minimal Isaac Lab app to access USD
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd

def find_missing_masters(usd_path):
    print(f"\nExhaustive Search for Masters in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # Check if they exist as prototypes
    prototypes = stage.GetPrototypes()
    print(f"\nPrototypes found: {len(prototypes)}")
    for proto in prototypes:
        print(f"  - Prototype: {proto.GetPath()}")
    
    # Traverse all including prototypes
    print("\nTraversing All Prims:")
    for prim in stage.TraverseAll():
        if "Flattened" in prim.GetName() or "Sign" in prim.GetName():
             print(f"  - {prim.GetPath()} ({prim.GetTypeName()})")

if __name__ == "__main__":
    track_path = os.path.join(PROJECT_ROOT, "openStreetUSD", "no_graph_sim.usd")
    find_missing_masters(track_path)
    simulation_app.close()
