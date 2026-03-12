from pxr import Usd, UsdGeom, Gf
import sys

usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/no_graph_sim.usd"
stage = Usd.Stage.Open(usd_path)

print(f"--- Elevation Audit for {usd_path} ---")
min_z = 99999
max_z = -99999

for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
        p_min = bbox.GetRange().GetMin()[2]
        p_max = bbox.GetRange().GetMax()[2]
        min_z = min(min_z, p_min)
        max_z = max(max_z, p_max)

print(f"Track Elevation Range: {min_z:.2f}m to {max_z:.2f}m")
