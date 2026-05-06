
from omni.isaac.kit import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom

def inspect_usd(usd_path):
    if not os.path.exists(usd_path):
        print(f"File {usd_path} not found")
        return

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"Failed to open stage {usd_path}")
        return

    print(f"Inspecting: {usd_path}")
    
    # Traverse all prims
    mesh_count = 0
    xform_count = 0
    scale_ops_found = []

    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
        elif prim.IsA(UsdGeom.Xform):
            xform_count += 1
        
        # Check for scale ops
        xformable = UsdGeom.Xformable(prim)
        if xformable:
            ops = xformable.GetOrderedXformOps()
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    val = op.Get()
                    if val and val != (1.0, 1.0, 1.0):
                        scale_ops_found.append((str(prim.GetPath()), val))

    print(f"Total Meshes: {mesh_count}")
    print(f"Total Xforms: {xform_count}")
    print(f"Prims with non-identity scale:")
    for path, val in scale_ops_found[:20]: # Show first 20
        print(f"  {path}: {val}")
    if len(scale_ops_found) > 20:
        print(f"  ... and {len(scale_ops_found) - 20} more")

if __name__ == "__main__":
    inspect_usd("openStreetUSD/no_graph_sim_clean_1x.usda")
    simulation_app.close()
