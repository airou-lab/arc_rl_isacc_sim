from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd
import os

usd_path = "openStreetUSD/no_graph_sim_cleaned.usd"
print(f"Opening {usd_path}")
stage = Usd.Stage.Open(usd_path)

if not stage:
    print(f"Failed to open {usd_path}")
else:
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "grass" in path.lower() or "terrain" in path.lower():
            print(f"Found: {path} (Type: {prim.GetTypeName()})")

simulation_app.close()
