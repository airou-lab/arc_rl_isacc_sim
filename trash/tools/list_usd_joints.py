import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics

def list_joints(usd_path):
    print(f"\nListing joints in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            print(f"  Joint: {prim.GetPath().name} ({prim.GetPath()})")

if __name__ == "__main__":
    list_joints('arcproLab/assets/robot/F1Tenth_Metric.usd')
    simulation_app.close()
