import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema

def check_collisions(usd_path):
    print(f"\nChecking collisions in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    count = 0
    coll_count = 0
    mesh_coll_count = 0
    approx_types = {}

    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            count += 1
            has_coll = prim.HasAPI(UsdPhysics.CollisionAPI)
            if has_coll:
                coll_count += 1
            
            has_mesh_coll = prim.HasAPI(UsdPhysics.MeshCollisionAPI)
            if has_mesh_coll:
                mesh_coll_count += 1
                mesh_coll = UsdPhysics.MeshCollisionAPI(prim)
                approx = mesh_coll.GetApproximationAttr().Get()
                approx_types[approx] = approx_types.get(approx, 0) + 1
                
                if mesh_coll_count <= 5:
                    print(f"  [OK] Mesh: {prim.GetPath()} | Approx: {approx}")
            else:
                if (count - mesh_coll_count) <= 5:
                    print(f"  [MISSING MeshCollisionAPI] Mesh: {prim.GetPath()}")
                
    print(f"\nTotal meshes: {count}")
    print(f"Meshes with CollisionAPI: {coll_count}")
    print(f"Meshes with MeshCollisionAPI: {mesh_coll_count}")
    print(f"Approximation Types: {approx_types}")

if __name__ == "__main__":
    check_collisions('openStreetUSD/no_graph_sim.usd')
    simulation_app.close()
