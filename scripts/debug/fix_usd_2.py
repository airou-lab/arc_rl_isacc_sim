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

for prim_path in ["/Robot/Wheel_FL", "/Robot/Wheel_RL", "/Robot/Joint_Steer_L"]:
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        # get world transform
        time = Usd.TimeCode.Default()
        matrix = xform.ComputeLocalToWorldTransform(time)
        pos = matrix.ExtractTranslation()
        print(f"{prim_path}: {pos}")

simulation_app.close()
