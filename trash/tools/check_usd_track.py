import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def inspect_track_in_usd(usd_path):
    print(f"Opening: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("Failed to open stage.")
        return

    # Typical names for track root
    potential_names = ["Track", "no_graph_sim", "drivable_surfaces"]
    
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        name = prim.GetName()
        
        if any(n.lower() in name.lower() for n in potential_names) and prim.IsA(UsdGeom.Xformable):
            print(f"\nFound Track-related Prim: {path} ({prim.GetTypeName()})")
            
            xform = UsdGeom.Xformable(prim)
            world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = world_transform.ExtractTranslation()
            
            # Check Scale
            row0 = world_transform.GetRow(0)
            row1 = world_transform.GetRow(1)
            row2 = world_transform.GetRow(2)
            actual_scale = (Gf.Vec3d(row0[0], row0[1], row0[2]).GetLength(),
                            Gf.Vec3d(row1[0], row1[1], row1[2]).GetLength(),
                            Gf.Vec3d(row2[0], row2[1], row2[2]).GetLength())
            
            print(f"  World Translation: {translation}")
            print(f"  Actual World Scale: {actual_scale}")

if __name__ == "__main__":
    inspect_track_in_usd("openStreetUSD/no_graph_sim_hardened.usd")
    simulation_app.close()
