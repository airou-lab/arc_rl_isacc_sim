
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf
import numpy as np

def dry_run_scale(usd_path, scale_factor=0.125):
    stage = Usd.Stage.Open(usd_path)
    
    stats = {
        "meshes": 0,
        "points_scaled": 0,
        "translations_scaled": 0,
        "matrices_scaled": 0,
        "extents_scaled": 0,
        "other_scaled": 0
    }

    for prim in stage.Traverse():
        # Scale Transforms
        xformable = UsdGeom.Xformable(prim)
        if xformable:
            ops = xformable.GetOrderedXformOps()
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    stats["translations_scaled"] += 1
                elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                    stats["matrices_scaled"] += 1
        
        # Scale Geometry
        if prim.IsA(UsdGeom.Mesh):
            stats["meshes"] += 1
            mesh = UsdGeom.Mesh(prim)
            if mesh.GetPointsAttr().HasAuthoredValue():
                stats["points_scaled"] += 1
            if mesh.GetExtentAttr().HasAuthoredValue():
                stats["extents_scaled"] += 1
        
        elif prim.IsA(UsdGeom.BasisCurves):
            stats["other_scaled"] += 1
        
        # Physics shapes
        elif any(prim.IsA(t) for t in [UsdGeom.Sphere, UsdGeom.Cube, UsdGeom.Cylinder, UsdGeom.Capsule]):
            stats["other_scaled"] += 1

    print(f"Dry Run Stats for {usd_path}:")
    for k, v in stats.items():
        print(f"  {k}: {v}")

if __name__ == "__main__":
    dry_run_scale("openStreetUSD/no_graph_sim_clean_1x.usda")
    simulation_app.close()
