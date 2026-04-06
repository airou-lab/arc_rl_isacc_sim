import argparse
from isaaclab.app import AppLauncher

# Minimal app to access USD API
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def check_road_height(usd_path):
    print(f"\nChecking mesh transforms in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            count += 1
            if "piece" in prim.GetName() or "terrain" in prim.GetName():
                # Get world transform
                bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim)
                range_3d = bbox.ComputeAlignedRange()
                min_v = range_3d.GetMin()
                max_v = range_3d.GetMax()
                center_z = (min_v[2] + max_v[2]) / 2.0
                print(f"  Mesh: {prim.GetPath()} | Z Range: [{min_v[2]:.2f}, {max_v[2]:.2f}] | Center Z: {center_z:.4f}")
                if count >= 10: break

if __name__ == "__main__":
    check_road_height('openStreetUSD/no_graph_sim.usd')
    simulation_app.close()
