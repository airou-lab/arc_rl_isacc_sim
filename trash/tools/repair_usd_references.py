import argparse
from isaaclab.app import AppLauncher

# Minimal app to access USD API
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
from pxr import Usd, Sdf

def fix_references(usd_path):
    print(f"\nRepairing: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"Failed to open {usd_path}")
        return
        
    # Use a LOCAL asset that we KNOW exists as a placeholder for signs.
    # This prevents the 'Could not open asset' error.
    # We use the robot asset itself just as a valid geometry target to test the link.
    # Later we can replace it with a real sign if we find one.
    LOCAL_PLACEHOLDER = os.path.abspath("arcproLab/assets/robot/F1Tenth_Metric.usd")
    
    to_repair = []
    to_remove = []
    
    # 1. Collect paths safely
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        name = prim.GetName()
        
        if "signpost_10ft_inst" in name or "sign_r1_1_" in name:
            to_repair.append(path)
            
        if name == "F1Tenth" and path == "/World/F1Tenth":
            to_remove.append(path)
            
    # 2. Apply repairs
    for path in to_repair:
        print(f"  Redirecting: {path}")
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            prim.GetReferences().ClearReferences()
            prim.GetReferences().AddReference(LOCAL_PLACEHOLDER)
            
    # 3. Apply removals
    for path in to_remove:
        print(f"  Removing: {path}")
        stage.RemovePrim(path)
            
    # 4. Final Cleanup of top-level dead prototypes if they exist in the root
    # (Often files have Flattened_Master at the root level as a prototype)
    root_prim = stage.GetPseudoRoot()
    for child in root_prim.GetChildren():
        if "Flattened_Master" in child.GetName():
            print(f"  Removing dead master: {child.GetPath()}")
            stage.RemovePrim(child.GetPath())

    # Save
    stage.GetRootLayer().Save()
    print("Repair complete.")

if __name__ == "__main__":
    fix_references('openStreetUSD/no_graph_sim.usd')
    fix_references('openStreetUSD/no_graph_sim_final.usd')
    simulation_app.close()
