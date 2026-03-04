# inspect_stage.py
# Run this in Isaac Sim Script Editor to see the stage hierarchy

import omni.usd
from pxr import Usd

def list_prims(path="/"):
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("No stage loaded.")
        return

    print(f"--- Listing Prims under {path} ---")
    root = stage.GetPrimAtPath(path)
    
    if not root.IsValid():
        print(f"Path {path} is not valid.")
        return

    for prim in root.GetChildren():
        print(f"Name: {prim.GetName()} | Path: {prim.GetPath()} | Type: {prim.GetTypeName()}")
        # Traverse one level deeper if it's the robot
        if "mushr" in prim.GetName().lower():
             print(f"  --- Details for {prim.GetName()} ---")
             for child in prim.GetChildren():
                 print(f"  Child: {child.GetName()} | Type: {child.GetTypeName()}")

list_prims("/World")
list_prims("/")
