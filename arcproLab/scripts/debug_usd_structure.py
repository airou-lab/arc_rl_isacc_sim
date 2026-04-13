import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="USD structure diagnostic.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
from pxr import Usd, UsdGeom

def print_hierarchy(prim, indent=0):
    print("  " * indent + f"{prim.GetName()} [{prim.GetTypeName()}]")
    for child in prim.GetChildren():
        print_hierarchy(child, indent + 1)

def main():
    stage = Usd.Stage.Open("openStreetUSD/no_graph_sim.usd")
    print("\nUSD HIERARCHY for no_graph_sim.usd:")
    print_hierarchy(stage.GetPseudoRoot())
    simulation_app.close()

if __name__ == "__main__":
    main()
