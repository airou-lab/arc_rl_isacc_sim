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

geom = stage.GetPrimAtPath("/Robot/Chassis/Geom")
print(f"Geom type: {geom.GetTypeName()}")
print(f"Has references? {geom.HasAuthoredReferences()}")
print(f"Has payloads? {geom.HasAuthoredPayloads()}")

for child in geom.GetChildren():
    print(child.GetPath())
    
# check if chassis has children
chassis = stage.GetPrimAtPath("/Robot/Chassis")
for child in chassis.GetChildren():
    print(child.GetPath())

simulation_app.close()
