from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("usd_path")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def get_mesh_extents(prim):
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim)
    range = bbox.GetRange()
    min = range.GetMin()
    max = range.GetMax()
    return max[0] - min[0], max[1] - min[1], max[2] - min[2]

def main():
    path = args_cli.usd_path
    print(f"Opening stage: {path}")
    stage = Usd.Stage.Open(path)
    
    # 1. Check Stage Units
    m_per_u = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"Stage Meters Per Unit: {m_per_u}")

    # 2. Measure existing robot if present
    robot_prim = stage.GetPrimAtPath("/World/F1Tenth")
    if robot_prim.IsValid():
        w, l, h = get_mesh_extents(robot_prim)
        print(f"Existing Robot Dimensions: Width={w:.4f}m, Length={l:.4f}m, Height={h:.4f}m")
    
    # 3. Measure a road segment
    # Looking for a typical road mesh
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and "road" in prim.GetName().lower():
            w, l, h = get_mesh_extents(prim)
            print(f"Road Mesh '{prim.GetName()}' Extents: {w:.4f} x {l:.4f} x {h:.4f}")
            # Usually the smaller dimension is the width if it's a segment
            width = min(w, l)
            print(f"Estimated Road/Lane Width: {width:.4f}m")
            break

    simulation_app.close()

if __name__ == "__main__":
    main()
