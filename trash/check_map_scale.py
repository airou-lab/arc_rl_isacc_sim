from pxr import Usd, UsdGeom, Gf
import sys

usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/no_graph_sim.usd"
stage = Usd.Stage.Open(usd_path)

print(f"--- Map Proportionality Check: {usd_path} ---")

lane_count = 0
avg_width = 0

for prim in stage.Traverse():
    if "road_mark_lane" in prim.GetName().lower() and prim.IsA(UsdGeom.Mesh):
        bbox = UsdGeom.Imageable(prim).ComputeWorldBound(0, "default")
        size = bbox.GetRange().GetMax() - bbox.GetRange().GetMin()
        # Usually lane markings are thin in Z and X (length), width is Y (or vice-versa)
        # We'll take the second smallest dimension as the 'width'
        dims = sorted([size[0], size[1], size[2]])
        width = dims[1] # Smallest is thickness, second is width
        avg_width += width
        lane_count += 1
        if lane_count <= 5:
            print(f"Lane Mark Prim: {prim.GetPath()} | Apparent Width: {width:.4f}m")

if lane_count > 0:
    print(f"\nMeasured {lane_count} lane markings. Avg Width: {avg_width/lane_count:.4f}m")
else:
    print("No lane markings found with 'road_mark_lane' in name.")
