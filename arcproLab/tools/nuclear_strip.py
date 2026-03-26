from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, PhysxSchema

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Generated.usd"
    print(f"Stripping corrupted root physics from: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    
    root_prim = stage.GetPrimAtPath("/Robot")
    if root_prim.IsValid():
        # Remove RigidBodyAPI and MassAPI from root grouping prim if present
        if root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            print("Removing RigidBodyAPI from /Robot...")
            root_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if root_prim.HasAPI(UsdPhysics.MassAPI):
            print("Removing MassAPI from /Robot...")
            root_prim.RemoveAPI(UsdPhysics.MassAPI)
        if root_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            print("Removing PhysxRigidBodyAPI from /Robot...")
            root_prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
            
        print("Root stripped. Checking Chassis...")
        
        chassis = stage.GetPrimAtPath("/Robot/Chassis")
        if chassis.IsValid():
            # Ensure Chassis IS a rigid body
            UsdPhysics.RigidBodyAPI.Apply(chassis)
            PhysxSchema.PhysxRigidBodyAPI.Apply(chassis)
            print("Chassis confirmed as dynamic rigid body.")

    stage.GetRootLayer().Save()
    print("Cleanup complete.")
    simulation_app.close()

if __name__ == "__main__":
    main()
