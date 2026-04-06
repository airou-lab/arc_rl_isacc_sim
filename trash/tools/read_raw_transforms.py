from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

from pxr import Usd, UsdGeom

def get_robot_transform(usd_path):
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("Failed to open stage.")
        return

    # Check for the robot prim you placed
    robot_path = "/World/F1Tenth"
    prim = stage.GetPrimAtPath(robot_path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        local_transform = xform.GetLocalTransformation()
        translation = local_transform.ExtractTranslation()
        rotation = local_transform.ExtractRotationQuat()
        print(f"\nFound Robot at {robot_path}:")
        print(f"  Translation: {translation}")
        print(f"  Rotation (Quat): {rotation}")
    else:
        print(f"Could not find prim at {robot_path}")
        # Search for anything with F1Tenth
        for p in stage.Traverse():
            if "F1Tenth" in p.GetPath().pathString:
                print(f"Found alternative: {p.GetPath()}")

if __name__ == "__main__":
    get_robot_transform("openStreetUSD/no_graph_sim.usd")
    app_launcher.app.close()
