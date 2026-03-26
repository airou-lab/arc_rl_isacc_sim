from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from pxr import Usd, UsdPhysics

stage = Usd.Stage.Open("f1tenth_trainer/assets/F1Tenth_Aggressive.usd")
print("Stage opened successfully.")

def traverse(prim):
    name = prim.GetName()
    type_name = prim.GetTypeName()
    if "Joint" in type_name:
        print(f"Found Joint: {prim.GetPath()} ({type_name})")
        for schema in prim.GetAppliedSchemas():
            print(f"  Schema: {schema}")
    
    for child in prim.GetChildren():
        traverse(child)

traverse(stage.GetPseudoRoot())
print("Traverse complete.")
app.close()
