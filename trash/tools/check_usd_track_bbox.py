from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

from pxr import Usd, UsdGeom

stage = Usd.Stage.Open('openStreetUSD/no_graph_sim_hardened.usd')
track_prim = stage.GetPrimAtPath("/World/drivable_surfaces")
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(track_prim)
r = bbox.GetRange()

print(f"Track World Min: {r.GetMin()}")
print(f"Track World Max: {r.GetMax()}")

app_launcher.app.close()
