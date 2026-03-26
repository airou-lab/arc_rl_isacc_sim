from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
from pxr import Usd, UsdGeom, Gf
import isaaclab.sim as sim_utils

def main():
    output_path = os.path.abspath("world_calibration.usd")
    print(f"Creating calibration world at: {output_path}")
    
    # Create a new stage
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    # Define a root Xform
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())

    # 1. Reference the Track
    track_path = os.path.abspath("openStreetUSD/no_graph_sim.usd")
    track_prim = stage.OverridePrim("/World/Track")
    track_prim.GetReferences().AddReference(track_path)
    
    # 2. Reference the Robot
    robot_path = os.path.abspath("f1tenth_trainer/assets/F1Tenth_Generated.usd")
    robot_prim = stage.DefinePrim("/World/Calibration_Robot", "Xform")
    robot_prim.GetReferences().AddReference(robot_path)
    
    # Set an initial scale so it's visible
    UsdGeom.Xformable(robot_prim).AddScaleOp().Set(Gf.Vec3f(10.0, 10.0, 10.0))
    UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))

    stage.GetRootLayer().Save()
    print("Done! You can now open 'world_calibration.usd' in Isaac Sim.")
    simulation_app.close()

if __name__ == "__main__":
    main()
