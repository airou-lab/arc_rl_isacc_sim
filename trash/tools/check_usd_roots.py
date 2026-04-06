from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
app_launcher = AppLauncher(parser.parse_args())

from pxr import Usd, UsdGeom

stage = Usd.Stage.Open('openStreetUSD/no_graph_sim_hardened.usd')
for p in stage.GetPseudoRoot().GetChildren():
    xform = UsdGeom.Xformable(p)
    if xform:
        translation = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        print(f"Prim: {p.GetPath()} | Type: {p.GetTypeName()} | World Trans: {translation}")
    else:
        print(f"Prim: {p.GetPath()} | Type: {p.GetTypeName()} | (Not Xformable)")

app_launcher.app.close()
