import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Read root scale.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def check_root_scale(usd_path):
    stage = Usd.Stage.Open(usd_path)
    prim = stage.GetPrimAtPath("/Full_Car")
    if prim:
        xform = UsdGeom.Xformable(prim)
        print(f"Ops for {prim.GetPath()}:")
        for op in xform.GetOrderedXformOps():
            print(f"  Op {op.GetOpName()}: {op.Get()}")

def main():
    check_root_scale(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
