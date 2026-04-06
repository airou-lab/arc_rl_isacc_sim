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

def inspect_road_mesh(usd_path):
    print(f"\nInspecting prims in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    print("Top-level prims:")
    for prim in stage.GetPseudoRoot().GetChildren():
        print(f"  - {prim.GetPath()} ({prim.GetTypeName()})")
    
    count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            count += 1
            if count < 50: # Limit output
                print(f"  Mesh: {prim.GetPath()}")
                
    print(f"Total meshes found: {count}")

if __name__ == "__main__":
    inspect_road_mesh('openStreetUSD/no_graph_sim.usd')
    simulation_app.close()
