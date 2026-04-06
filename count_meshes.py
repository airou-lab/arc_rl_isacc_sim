import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics
import os

files = [
    'openStreetUSD/no_graph_sim_final.usd',
    'openStreetUSD/no_graph_sim.usd',
    'openStreetUSD/no_graph_sim_cleaned.usd'
]

for f in files:
    if os.path.exists(f):
        stage = Usd.Stage.Open(f)
        meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
        colliders = [p for p in meshes if p.HasAPI(UsdPhysics.CollisionAPI)]
        print(f"{f}: {len(meshes)} meshes, {len(colliders)} with CollisionAPI")
    else:
        print(f"{f}: Not found")

simulation_app.close()
