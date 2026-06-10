
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os
import sys
from pxr import UsdGeom, Usd

def main():
    stage = Usd.Stage.Open("arcproLab/assets/robot/F1Tenth_Metric.usd")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    
    # Look for the root or chassis
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xformable):
            bbox = bbox_cache.ComputeLocalBound(prim)
            r = bbox.GetRange()
            print(f"Prim: {prim.GetPath()}")
            print(f"  Local Min: {r.GetMin()}")
            print(f"  Local Max: {r.GetMax()}")
            print(f"  Height below origin: {-r.GetMin()[2]:.4f}m")

    simulation_app.close()

if __name__ == "__main__":
    main()
