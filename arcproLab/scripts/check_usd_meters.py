import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Measure robot bounds in USD.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def measure_usd(usd_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # Check MetersPerUnit
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    print(f"Meters Per Unit: {mpu}")
    print(f"Up Axis: {up_axis}")
    
    # Compute World BBox of the whole stage
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bbox = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot())
    range = bbox.ComputeAlignedRange()
    
    min_pt = range.GetMin()
    max_pt = range.GetMax()
    size = max_pt - min_pt
    
    print(f"World BBox Min: {min_pt}")
    print(f"World BBox Max: {max_pt}")
    print(f"World Size (units): {size}")
    print(f"World Size (meters): {size * mpu}")
    
    # Now check /Full_Car specifically
    prim = stage.GetPrimAtPath("/Full_Car")
    if prim:
        bbox = bbox_cache.ComputeWorldBound(prim)
        range = bbox.ComputeAlignedRange()
        print(f"\n/Full_Car BBox Min: {range.GetMin()}")
        print(f"/Full_Car BBox Max: {range.GetMax()}")
        print(f"/Full_Car Size (meters): {(range.GetMax() - range.GetMin()) * mpu}")

def main():
    measure_usd(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
