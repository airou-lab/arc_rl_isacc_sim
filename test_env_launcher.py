import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser()
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

try:
    from pxr import Usd, UsdGeom
    print("SUCCESS: pxr imported after AppLauncher")
except ImportError as e:
    print(f"FAILURE: {e}")
    
simulation_app.close()
