
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, Gf

def inspect_transforms(usd_path):
    stage = Usd.Stage.Open(usd_path)
    for prim in stage.Traverse():
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            continue
            
        ops = xformable.GetOrderedXformOps()
        if ops:
            print(f"Prim: {prim.GetPath()}")
            for op in ops:
                print(f"  Op: {op.GetOpType()} - {op.Get()}")

if __name__ == "__main__":
    # Just check a few interesting ones
    usd_path = "openStreetUSD/no_graph_sim_clean_1x.usda"
    stage = Usd.Stage.Open(usd_path)
    
    # Let's look at /World and some children
    world = stage.GetPrimAtPath("/World")
    print(f"World Ops: {[op.GetOpType() for op in UsdGeom.Xformable(world).GetOrderedXformOps()]}")
    
    count = 0
    for prim in stage.Traverse():
        if prim == world: continue
        xformable = UsdGeom.Xformable(prim)
        if xformable:
            ops = xformable.GetOrderedXformOps()
            if ops:
                print(f"Prim: {prim.GetPath()}")
                for op in ops:
                    print(f"  {op.GetOpName()}: {op.Get()}")
                count += 1
        if count > 10: break

    simulation_app.close()
