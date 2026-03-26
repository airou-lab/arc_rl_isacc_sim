import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check robot wheel collisions.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics

def check_collisions(usd_path):
    stage = Usd.Stage.Open(usd_path)
    print(f"Checking: {usd_path}")
    
    for prim in stage.Traverse():
        if "Wheel" in prim.GetName() and prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"\nCollision Prim: {prim.GetPath()}")
            # Check for sphere radius if it's a sphere
            if prim.IsA(UsdGeom.Sphere):
                sphere = UsdGeom.Sphere(prim)
                print(f"  Sphere Radius: {sphere.GetRadiusAttr().Get()}")
            
            # Check for approximation
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_coll = UsdPhysics.MeshCollisionAPI(prim)
                print(f"  Approximation: {mesh_coll.GetApproximationAttr().Get()}")
            
            # Check bounds
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
            bbox = bbox_cache.ComputeLocalBound(prim, Usd.TimeCode.Default(), "default")
            range = bbox.GetRange()
            print(f"  Local BBox: {range.GetMin()} to {range.GetMax()}")
            
            # World Pos
            xform = UsdGeom.Xformable(prim)
            world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            print(f"  World Translation: {world_transform.ExtractTranslation()}")

def main():
    check_collisions("f1tenth_trainer/assets/F1Tenth_TrulyRescued.usd")
    simulation_app.close()

if __name__ == "__main__":
    main()
