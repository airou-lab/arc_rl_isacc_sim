from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

from pxr import Usd, UsdGeom, Gf

def audit_usd_road_bbox(usd_path):
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("Failed to open stage.")
        return

    # Look for the drivable surfaces group
    road_root = stage.GetPrimAtPath("/World/drivable_surfaces")
    if not road_root:
        # Fallback to searching
        for prim in stage.Traverse():
            if "drivable" in prim.GetPath().pathString.lower():
                road_root = prim
                break
    
    if road_root:
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(road_root)
        r = bbox.GetRange()
        print(f"Road Root: {road_root.GetPath()}")
        print(f"Road BBox: {r.GetMin()} to {r.GetMax()}")
        print(f"Road Center: {r.GetMid()}")
    else:
        print("Could not find road root prim.")

if __name__ == "__main__":
    audit_usd_road_bbox("openStreetUSD/no_graph_sim.usd")
    app_launcher.app.close()
