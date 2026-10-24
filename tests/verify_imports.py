import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify imports")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys

sys.path.insert(0, os.path.abspath("."))
sys.path.insert(0, os.path.abspath("arcproLab"))

import arcproLab.agents.skrl_wrappers as w

print("[VERIFY] skrl_wrappers imported successfully!")
print(f"[VERIFY] SKRLFlattenWrapper: {w.SKRLFlattenWrapper}")

simulation_app.close()
print("IMPORTS_OK")
