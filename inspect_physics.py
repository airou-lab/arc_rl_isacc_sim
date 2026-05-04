
import os
import sys
from pxr import Usd, UsdPhysics, UsdGeom, Gf

# This script must be run with isaaclab.sh, which sets up the python environment.
# We will not launch the full simulation, just inspect the USD.

def audit_mass(stage):
    print("=== AUDIT: MASS PROPERTIES ===")
    total_mass = 0
    found_chassis = False
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            mass = mass_api.GetMassAttr().Get()
            com = mass_api.GetCenterOfMassAttr().Get()
            if mass > 0:
                print(f"Prim: {prim.GetPath()} | Mass: {mass:.4f}")
                total_mass += mass
                if "Chassis" in str(prim.GetPath()):
                    found_chassis = True
                    print(f"  > Center of Mass: {com}")
                    if com != Gf.Vec3f(0, 0, 0):
                        print("  > !!! WARNING: Center of Mass is not at the origin.")
    if not found_chassis:
        print("[ERROR] Could not find a 'Chassis' prim with MassAPI.")
    print(f"Total Mass defined in USD: {total_mass:.4f} kg")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    usd_path = os.path.join(script_dir, "arcproLab", "assets", "robot", "F1Tenth_Metric.usd")
    if not os.path.exists(usd_path):
        print(f"[ERROR] Cannot find robot USD at: {usd_path}")
        return
        
    print(f"Opening stage for inspection: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("[ERROR] Failed to open USD stage.")
        return
        
    audit_mass(stage)

if __name__ == "__main__":
    # Add required paths for Isaac Lab's python environment
    sys.path.append(os.environ.get("ISAAC_PATH"))
    from isaaclab.app import AppLauncher
    
    # Minimal app launcher to get python environment
    app_launcher = AppLauncher(headless=True)
    simulation_app = app_launcher.app
    
    main()
    
    # Close the app
    simulation_app.close()
