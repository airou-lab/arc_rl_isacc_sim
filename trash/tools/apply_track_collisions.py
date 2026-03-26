from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema

def main():
    usd_path = "openStreetUSD/no_graph_sim_cleaned.usd"
    print(f"Baking SDF collisions into: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        
        # Harden ALL meshes in the track, not just pavement
        if prim.IsA(UsdGeom.Mesh):
            print(f"Applying exact mesh collision to: {path_str}")
            
            # Enable basic collision
            UsdPhysics.CollisionAPI.Apply(prim)
            
            # Use exact triangle mesh collision
            mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_api.GetApproximationAttr().Set("none")
            
            # Physics offsets
            px_coll = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            px_coll.GetContactOffsetAttr().Set(0.1) # 10cm shell
            px_coll.GetRestOffsetAttr().Set(0.02)    # 2cm rest

    stage.GetRootLayer().Save()
    print("SDF baking complete.")
    simulation_app.close()

if __name__ == "__main__":
    main()
