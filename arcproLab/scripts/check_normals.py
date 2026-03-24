import os
import sys
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Check track normals.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

def check_normals(usd_path):
    stage = Usd.Stage.Open(usd_path)
    print(f"Checking normals: {usd_path}")
    
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            normals = mesh.GetNormalsAttr().Get()
            if normals:
                # Check average normal
                avg_norm = Gf.Vec3f(0)
                for n in normals:
                    avg_norm += n
                avg_norm /= len(normals)
                print(f"Mesh: {prim.GetPath()} | Avg Normal: {avg_norm}")
            else:
                print(f"Mesh: {prim.GetPath()} | NO NORMALS")

def main():
    check_normals(args_cli.usd_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
