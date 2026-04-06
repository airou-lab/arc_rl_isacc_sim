from isaaclab.app import AppLauncher
import argparse
import sys
import os

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

def audit_usd_physics(path, name):
    print(f"\n=== AUDIT: {name} ({path}) ===")
    stage = Usd.Stage.Open(path)
    if not stage:
        print(f"FAILED TO OPEN {path}")
        return

    for prim in stage.Traverse():
        # Check for Physics Materials
        if prim.IsA(UsdPhysics.MaterialAPI):
            mat = UsdPhysics.MaterialAPI(prim)
            print(f"\n[Material] {prim.GetPath()}")
            print(f"  Static Fric: {mat.GetStaticFrictionAttr().Get()}")
            print(f"  Dynamic Fric: {mat.GetDynamicFrictionAttr().Get()}")
            print(f"  Restitution: {mat.GetRestitutionAttr().Get()}")

        # Check for Rigid Body / Mass properties
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass = UsdPhysics.MassAPI(prim)
            print(f"\n[Mass] {prim.GetPath()}")
            print(f"  Mass: {mass.GetMassAttr().Get()} kg")
            print(f"  CoM: {mass.GetCenterOfMassAttr().Get()}")
            print(f"  Inertia: {mass.GetDiagonalInertiaAttr().Get()}")

        # Check for Collision Mesh Properties
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"\n[Mesh] {prim.GetPath()}")
            mesh = UsdGeom.Mesh(prim)
            print(f"  Double Sided: {mesh.GetDoubleSidedAttr().Get()}")
            print(f"  Orientation: {mesh.GetOrientationAttr().Get()}")
            
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mca = UsdPhysics.MeshCollisionAPI(prim)
                print(f"  Approx: {mca.GetApproximationAttr().Get()}")
            
            if prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                pca = PhysxSchema.PhysxCollisionAPI(prim)
                print(f"  Contact Offset: {pca.GetContactOffsetAttr().Get()}")
                print(f"  Rest Offset: {pca.GetRestOffsetAttr().Get()}")

def main():
    # Define absolute paths based on project root
    PROJECT_ROOT = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim"
    
    # 1. Audit the Robot
    robot_usd = os.path.join(PROJECT_ROOT, "arcproLab", "assets", "robot", "F1Tenth_Metric.usd")
    audit_usd_physics(robot_usd, "Robot")

    # 2. Audit the Track
    track_usd = os.path.join(PROJECT_ROOT, "openStreetUSD", "no_graph_sim.usd")
    audit_usd_physics(track_usd, "Track")

    app_launcher.app.close()

if __name__ == "__main__":
    main()
