from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Generated.usd"
    print(f"Searching for Fixed Joints in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.FixedJoint):
            print(f"WARNING: Found FixedJoint at {prim.GetPath()}")
            body0 = prim.GetProperty("physics:body0").GetTargets()
            body1 = prim.GetProperty("physics:body1").GetTargets()
            print(f"  -> Connects: {body0} to {body1}")
            
        if prim.IsA(UsdPhysics.Joint):
            # General joint check
            enabled = prim.GetProperty("physics:jointEnabled").Get()
            print(f"Joint: {prim.GetPath()} | Enabled: {enabled}")

    simulation_app.close()

if __name__ == "__main__":
    main()
