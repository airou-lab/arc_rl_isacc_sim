import omni.kit
from omni.isaac.kit import SimulationApp

# Start simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf
import os
import numpy as np

def scale_mesh_vertices(usd_path, scale_factor=0.125):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("Failed to open stage.")
        return

    # 1. Iterate over all Mesh prims
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points_attr = mesh.GetPointsAttr()
            points = points_attr.Get()
            
            if points:
                # Multiply every vertex by the scale factor
                scaled_points = [p * scale_factor for p in points]
                points_attr.Set(scaled_points)
                print(f"  Scaled {len(points)} vertices for mesh: {prim.GetPath()}")
            
            # 2. Also scale any local translations (to keep positions correct)
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val:
                        op.Set(val * scale_factor)
                elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    # Reset scale to 1.0 (since we've baked it into vertices)
                    op.Set(Gf.Vec3f(1.0, 1.0, 1.0))

    # 3. Handle Xform prims (non-mesh parents)
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xform):
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    val = op.Get()
                    if val:
                        op.Set(val * scale_factor)
                elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    op.Set(Gf.Vec3f(1.0, 1.0, 1.0))

    # 4. Handle Scope/other prims with custom transforms if any
    # (Simplified for this use case as most geometry is in Xforms/Meshes)

    # Save as a new file
    new_path = usd_path.replace(".usda", "_flattened.usda")
    stage.GetRootLayer().Export(new_path)
    print(f"Flattened asset saved to: {new_path}")
    return new_path

if __name__ == "__main__":
    usd_dir = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD"
    usd_file = os.path.join(usd_dir, "no_graph_sim_clean_1x.usda")
    scale_mesh_vertices(usd_file)
    simulation_app.close()
