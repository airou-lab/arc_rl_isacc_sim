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
from pxr import Usd, UsdGeom

def check_sign_geometry(path):
    print(f"\nChecking: {path}")
    stage = Usd.Stage.Open(path)
    if not stage:
        print(f"Failed to open {path}")
        return
        
    found = False
    # Search for any signpost prims
    for prim in stage.Traverse():
        if "signpost_10ft_inst" in prim.GetName():
            found = True
            print(f"Found prim: {prim.GetPath()}")
            print(f"  Is Instance: {prim.IsInstance()}")
            refs = prim.GetMetadata("references")
            print(f"  References: {refs}")
            
            # Check for mesh children or sibling geometry
            parent = prim.GetParent()
            print(f"  Parent: {parent.GetPath()}")
            for child in parent.GetChildren():
                print(f"    - Child: {child.GetPath()} ({child.GetTypeName()})")
            
            # Stop after first one for now
            break
            
    if not found:
        print("Could not find any signpost prim.")

if __name__ == "__main__":
    check_sign_geometry('openStreetUSD/archive/original_usd.usd')
    simulation_app.close()
