from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("usd_path")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def main():
    path = args_cli.usd_path
    print(f"Cleaning stage: {path}")
    stage = Usd.Stage.Open(path)
    if stage:
        # Prims to remove (covering different potential paths)
        to_remove = [
            "/Track/Visuals/F1Tenth", 
            "/Track/Visuals/ActionGraph", 
            "/Track/Visuals/Intersection_Systems",
            "/World/F1Tenth",
            "/World/ActionGraph",
            "/World/Intersection_Systems"
        ]
        
        for p in to_remove:
            prim = stage.GetPrimAtPath(p)
            if prim.IsValid():
                print(f"Removing prim: {p}")
                stage.RemovePrim(p)
            else:
                print(f"Prim not found: {p}")
        
        stage.GetRootLayer().Save()
        print("Stage cleaned successfully.")
    else:
        print("Failed to open stage.")
    simulation_app.close()

if __name__ == "__main__":
    main()
