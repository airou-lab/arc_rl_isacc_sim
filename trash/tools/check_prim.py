import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check all attributes of a prim.")
parser.add_argument("usd_path", type=str)
parser.add_argument("prim_path", type=str)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def check_prim(usd_path, prim_path):
    stage = Usd.Stage.Open(usd_path)
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        print(f"Prim not found: {prim_path}")
        return
    
    print(f"\n--- Attributes for {prim_path} ---")
    for attr in prim.GetAttributes():
        print(f"  {attr.GetName()}: {attr.Get()}")
    
    print(f"\n--- Applied Schemas ---")
    for schema in prim.GetAppliedSchemas():
        print(f"  {schema}")

def main():
    check_prim(args_cli.usd_path, args_cli.prim_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
