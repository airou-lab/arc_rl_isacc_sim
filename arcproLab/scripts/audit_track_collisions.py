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
    print(f"Auditing collisions in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            path_str = str(prim.GetPath())
            # Check for generic MeshCollisionAPI
            mesh_api = UsdPhysics.MeshCollisionAPI.Get(stage, prim.GetPath())
            if mesh_api:
                approx = mesh_api.GetApproximationAttr().Get()
                print(f"Mesh: {path_str:40s} | Approx: {approx}")
            else:
                # Check if it has collision enabled at all
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    print(f"Mesh: {path_str:40s} | Collision: ENABLED (No MeshAPI)")

    simulation_app.close()

if __name__ == "__main__":
    main()
