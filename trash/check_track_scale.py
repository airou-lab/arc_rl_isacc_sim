from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("usd_path", type=str)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def check_usd(usd_path):
    stage = Usd.Stage.Open(usd_path)
    print(f"Units: {UsdGeom.GetStageMetersPerUnit(stage)}")
    print(f"Up Axis: {UsdGeom.GetStageUpAxis(stage)}")
    
    # Get total bounding box
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.render])
    root_prim = stage.GetDefaultPrim()
    if root_prim.IsValid():
        bbox = bbox_cache.ComputeWorldBound(root_prim)
        range = bbox.ComputeAlignedRange()
        print(f"Default Prim Range: {range}")
        print(f"Dimensions: {range.GetSize()}")
    else:
        print("No default prim!")

    # Check some child prims
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
             bbox = bbox_cache.ComputeWorldBound(prim)
             range = bbox.ComputeAlignedRange()
             print(f"Mesh {prim.GetPath()} range: {range}")
             break # Just one

check_usd(args_cli.usd_path)
simulation_app.close()
