import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def inspect_robot_in_usd(usd_path):
    print(f"Opening: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("Failed to open stage.")
        return

    found = False
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        # Look for F1Tenth or Robot in the path
        if "f1tenth" in path.lower() or "robot" in path.lower():
            print(f"\nFound Robot Prim: {path}")
            found = True
            
            # Get Transform
            xform = UsdGeom.Xformable(prim)
            world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            translation = world_transform.ExtractTranslation()
            rotation = world_transform.ExtractRotationQuat()
            
            # Get Scale specifically from the Ops if possible, or extract from matrix
            # Extracting scale from matrix is safer for world scale
            # But let's try to see if there is a scale op
            scale = Gf.Vec3d(1,1,1)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale = op.Get()
            
            print(f"  Translation: {translation}")
            print(f"  Rotation (Quat): {rotation}")
            print(f"  Local Scale Op: {scale}")
            
            # Also check the actual scale in the transform matrix
            row0 = world_transform.GetRow(0)
            row1 = world_transform.GetRow(1)
            row2 = world_transform.GetRow(2)
            actual_scale = (Gf.Vec3d(row0[0], row0[1], row0[2]).GetLength(),
                            Gf.Vec3d(row1[0], row1[1], row1[2]).GetLength(),
                            Gf.Vec3d(row2[0], row2[1], row2[2]).GetLength())
            print(f"  Actual World Scale: {actual_scale}")

    if not found:
        print("No Robot/F1Tenth prim found in the USD.")

if __name__ == "__main__":
    inspect_robot_in_usd("openStreetUSD/no_graph_sim_hardened.usd")
    simulation_app.close()
