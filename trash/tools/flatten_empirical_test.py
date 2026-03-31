import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Empirical Flattening Test.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom

def flatten_empirical(usd_path, output_path):
    print(f"\n--- EMPIRICAL FLATTEN TEST: {usd_path} ---")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"FAILED to open {usd_path}")
        return

    # Use stage.Flatten() to bake all references into a single layer
    flattened_layer = stage.Flatten()
    flattened_stage = Usd.Stage.CreateNew(output_path)
    flattened_stage.GetRootLayer().TransferContent(flattened_layer)
    flattened_stage.GetRootLayer().Save()
    print(f"Successfully Flattened to: {output_path}")

flatten_empirical(args_cli.usd_path, args_cli.output_path)
simulation_app.close()
