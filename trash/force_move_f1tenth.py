# force_move_f1tenth.py
import omni.usd
from pxr import UsdPhysics, Sdf

def run_fix_and_move():
    stage = omni.usd.get_context().get_stage()
    robot_path = "/World/F1Tenth"
    
    print("--- F1Tenth Force Move Start ---")

    # 1. Clear all existing ArticulationRoots to stop the "Nested" error
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    
    # 2. Apply a fresh ArticulationRoot to the top-level robot prim
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        print("ERROR: Robot not found at " + robot_path)
        return
    
    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    print("Applied ArticulationRoot to " + robot_path)

    # 3. Target ONLY wheel joints to avoid errors on suspension arms
    actuated_count = 0
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if prim.IsA(UsdPhysics.Joint) and path_str.startswith(robot_path):
            name = prim.GetName().lower()
            
            # Only actuate things that look like wheels
            if "wheel" in name:
                vel_attr = prim.GetAttribute("drive:angular:physics:targetVelocity")
                effort_attr = prim.GetAttribute("drive:angular:physics:maxEffort")
                
                # Verify attribute is valid and has a type before setting
                if vel_attr.IsValid():
                    try:
                        vel_attr.Set(100.0) # Much faster
                        if effort_attr.IsValid():
                            effort_attr.Set(1e12) # Even more torque
                        print("  [ACTUATED] " + prim.GetName())
                        actuated_count += 1
                    except Exception as e:
                        print("  [SKIP] Could not set drive on " + prim.GetName() + ": " + str(e))

    print("SUMMARY: Motors Ready: " + str(actuated_count))
    print(">>> PRESS PLAY IN THE GUI NOW <<<")

if __name__ == "__main__":
    run_fix_and_move()
