import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def check_bbox(usd_path):
    print(f"\nChecking overall BBox in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    root_prim = stage.GetPrimAtPath("/World/drivable_surfaces")
    if not root_prim:
        print("Root /World/drivable_surfaces not found!")
        return
        
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox = bbox_cache.ComputeWorldBound(root_prim)
    range_3d = bbox.ComputeAlignedRange()
    min_v = range_3d.GetMin()
    max_v = range_3d.GetMax()
    
    print(f"Overall BBox:")
    print(f"  Min: {min_v}")
    print(f"  Max: {max_v}")

if __name__ == "__main__":
    check_bbox('openStreetUSD/no_graph_sim.usd')
    simulation_app.close()
