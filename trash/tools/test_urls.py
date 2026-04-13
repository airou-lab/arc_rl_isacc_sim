import sys
import os

# Set up paths properly
PROJECT_ROOT = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim"
sys.path.append(PROJECT_ROOT)

import argparse
from isaaclab.app import AppLauncher

# Generate a minimal Isaac Lab app to access USD
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd

urls = [
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/Signs/Stop_Sign.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.0/Isaac/Props/Signs/Stop_Sign.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Props/Signs/Stop_Sign.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/TrafficCone/TrafficCone.usd"
]

for url in urls:
    print(f"\nTesting URL: {url}")
    stage = Usd.Stage.Open(url)
    if stage:
        print(f"  [SUCCESS] Stage opened!")
    else:
        print(f"  [FAILED] Could not open stage.")

simulation_app.close()
