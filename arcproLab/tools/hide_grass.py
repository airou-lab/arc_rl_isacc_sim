from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom
import os

usd_path = "openStreetUSD/no_graph_sim_cleaned.usd"
print(f"Opening {usd_path} to hide grass...")
stage = Usd.Stage.Open(usd_path)

if not stage:
    print(f"Failed to open {usd_path}")
else:
    grass_prim = stage.GetPrimAtPath("/World/grass")
    if grass_prim:
        imageable = UsdGeom.Imageable(grass_prim)
        if imageable:
            imageable.MakeInvisible()
            print("Set /World/grass to invisible.")
        else:
            print("/World/grass is not imageable.")
    else:
        print("/World/grass not found.")
    
    stage.GetRootLayer().Save()
    print("Saved USD.")

simulation_app.close()
