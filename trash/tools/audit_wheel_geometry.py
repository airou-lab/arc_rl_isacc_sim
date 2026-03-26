from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Generated.usd"
    print(f"Auditing Wheel Geometry in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    wheel_paths = [
        "/Robot/Wheel_FL", "/Robot/Wheel_FR",
        "/Robot/Wheel_RL", "/Robot/Wheel_RR"
    ]
    
    for path in wheel_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"Error: Prim {path} not found.")
            continue
            
        # Get local transform
        xform = UsdGeom.Xformable(prim)
        local_pos = xform.GetLocalTransformation().ExtractTranslation()
        print(f"Wheel: {path:20s} | Local Pos: {local_pos}")
        
        # Check child meshes (visuals)
        for child in prim.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                child_xform = UsdGeom.Xformable(child)
                child_pos = child_xform.GetLocalTransformation().ExtractTranslation()
                print(f"  -> Mesh {child.GetName():15s} | Local Pos: {child_pos}")

    simulation_app.close()

if __name__ == "__main__":
    main()
