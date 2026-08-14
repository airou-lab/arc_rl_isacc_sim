import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom
stage = Usd.Stage.Open("arcproLab/assets/robot/F1Tenth_Metric.usd")

for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xformable):
        print(f"Xformable/Mesh: {prim.GetPath()}")

simulation_app.close()
