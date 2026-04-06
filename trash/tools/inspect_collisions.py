from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

from pxr import Usd, UsdGeom, UsdPhysics, Gf

def inspect_collisions_near_spawn(usd_path, spawn_xy):
    print(f"Opening: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    spawn_vec = Gf.Vec2d(spawn_xy[0], spawn_xy[1])
    threshold = 50.0 # Meters radius to check
    
    print(f"Checking for colliders within {threshold}m of {spawn_xy}...")
    
    found_count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Plane) or prim.IsA(UsdGeom.Cube):
            # Check if it has physics
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = world_transform.ExtractTranslation()
                
                # Check distance
                dist = (Gf.Vec2d(translation[0], translation[1]) - spawn_vec).GetLength()
                
                # Also check bounding box for large meshes that might overlap spawn even if origin is far
                bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim)
                r = bbox.GetRange()
                
                inside = (r.GetMin()[0] <= spawn_xy[0] <= r.GetMax()[0] and 
                          r.GetMin()[1] <= spawn_xy[1] <= r.GetMax()[1])
                
                if dist < threshold or inside:
                    print(f"\nPotential Colliding Prim: {prim.GetPath()} ({prim.GetTypeName()})")
                    print(f"  World Translation: {translation}")
                    print(f"  BBox Min: {r.GetMin()}")
                    print(f"  BBox Max: {r.GetMax()}")
                    print(f"  Distance from Spawn origin: {dist:.2f}m")
                    if inside: print("  [!] OVERLAPS SPAWN POINT")
                    
                    # Check for approximation
                    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                        mca = UsdPhysics.MeshCollisionAPI(prim)
                        print(f"  Approximation: {mca.GetApproximationAttr().Get()}")
                    
                    found_count += 1

    if found_count == 0:
        print("No immediate colliders found near spawn point.")

if __name__ == "__main__":
    # Your spawn point: (-129.465, 46.927)
    inspect_collisions_near_spawn("openStreetUSD/no_graph_sim.usd", (-129.465, 46.927))
    app_launcher.app.close()
