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

def find_masters(usd_dir):
    print(f"\nSearching for 'Sign' or 'Post' in {usd_dir}...")
    if not os.path.exists(usd_dir):
        print(f"Directory {usd_dir} does not exist.")
        return
        
    files = [f for f in os.listdir(usd_dir) if f.endswith(".usd")]
    
    for filename in files:
        path = os.path.join(usd_dir, filename)
        try:
            stage = Usd.Stage.Open(path)
            if not stage:
                print(f"Failed to open: {filename}")
                continue
                
            print(f"Checking: {filename}")
            for prim in stage.Traverse():
                low_name = prim.GetName().lower()
                if "sign" in low_name or "post" in low_name:
                    print(f"  [FOUND] {prim.GetName()} at path {prim.GetPath()} in {filename}")
                    if prim.IsA(UsdGeom.Mesh):
                        print(f"    (It is a Mesh)")
        except Exception as e:
            print(f"Error checking {filename}: {e}")

if __name__ == "__main__":
    # Check current dir
    find_masters("openStreetUSD")
    # Check archive
    find_masters("openStreetUSD/archive")
    simulation_app.close()
