import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Apply Mass to Robot.")
parser.add_argument("usd_path", type=str, help="Path to the robot USD.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics

def apply_mass(usd_path):
    print(f"Opening robot: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    # 1. Apply 3.5kg to Chassis
    chassis_path = "/Robot/Chassis"
    prim = stage.GetPrimAtPath(chassis_path)
    if prim:
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.GetMassAttr().Set(3.5) # 3.5 kg
        print(f"Applied 3.5kg to {chassis_path}")
    
    # 2. Apply 0.1kg to each wheel
    for wheel in ["Wheel_FL", "Wheel_FR", "Wheel_RL", "Wheel_RR"]:
        path = f"/Robot/{wheel}"
        prim = stage.GetPrimAtPath(path)
        if prim:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.GetMassAttr().Set(0.1) # 100g per wheel
            print(f"Applied 0.1kg to {path}")
            
    stage.GetRootLayer().Save()
    print("Robot mass properties updated.")

apply_mass(args_cli.usd_path)
simulation_app.close()
