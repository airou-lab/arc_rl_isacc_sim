import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Hardened Flattening Fix.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics

def harden_and_flatten(usd_path, output_path):
    print(f"\n--- HARDENED FLATTEN: {usd_path} ---")
    stage = Usd.Stage.Open(usd_path)
    
    # Flatten first to resolve all references
    flattened_layer = stage.Flatten()
    new_stage = Usd.Stage.CreateNew(output_path)
    new_stage.GetRootLayer().TransferContent(flattened_layer)
    
    # Now, iterate through all meshes and FORCE stable physics
    for prim in new_stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName().lower()
            if "terrain" in name or "road" in name:
                print(f"  Hardening Physics for: {prim.GetPath()}")
                
                # Apply/Update Collision APIs
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                
                # FORCE convexDecomposition for stability on complex OSM meshes
                mesh_api.GetApproximationAttr().Set("convexDecomposition")
                
    new_stage.GetRootLayer().Save()
    print(f"Hardened file saved to: {output_path}")

harden_and_flatten(args_cli.usd_path, args_cli.output_path)
simulation_app.close()
