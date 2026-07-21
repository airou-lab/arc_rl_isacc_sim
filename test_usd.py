import argparse
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext, SimulationCfg
from arcproLab.arcpro_env_cfg import ARCProSceneCfg
import omni.usd
from pxr import Usd, UsdGeom

sim_cfg = SimulationCfg(dt=1.0/60.0, device="cuda:0")
sim = SimulationContext(sim_cfg)
scene_cfg = ARCProSceneCfg(num_envs=1, env_spacing=5.0)
scene_cfg.tiled_camera = None
scene = InteractiveScene(scene_cfg)
sim.reset()

stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.BasisCurves) or prim.IsA(UsdGeom.NurbsCurve):
        print(f"Curve found: {prim.GetPath()}")
    if "center" in str(prim.GetPath()).lower() or "path" in str(prim.GetPath()).lower():
        print(f"Path/Center found: {prim.GetPath()} ({prim.GetTypeName()})")
        
simulation_app.close()
