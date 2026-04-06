from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

from pxr import Usd

def convert_to_usda(usd_path, usda_path):
    stage = Usd.Stage.Open(usd_path)
    if stage:
        stage.GetRootLayer().Export(usda_path)
        print(f"Exported {usd_path} to {usda_path}")

if __name__ == "__main__":
    convert_to_usda("openStreetUSD/no_graph_sim.usd", "openStreetUSD/no_graph_sim.usda")
    app_launcher.app.close()
