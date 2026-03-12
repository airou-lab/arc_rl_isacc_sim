from pxr import Usd, UsdGeom, Gf
import sys

usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/no_graph_sim.usd"
stage = Usd.Stage.Open(usd_path)

print(f"--- Map Proportionality Check (Roads): {usd_path} ---")

count = 0
for prim in stage.Traverse():
    if "road" in prim.GetName().lower() and prim.IsA(UsdGeom.Mesh):
        bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
        size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
        print(f"Road Prim: {prim.GetPath()} | Size: {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m")
        count += 1
        if count >= 10: break

if count == 0:
    print("No road meshes found. Checking all meshes...")
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
            size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
            print(f"Mesh: {prim.GetPath()} | Size: {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m")
            count += 1
            if count >= 10: break
