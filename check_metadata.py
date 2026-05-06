
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom

usd_path = "openStreetUSD/no_graph_sim_clean_1x.usda"
stage = Usd.Stage.Open(usd_path)
print(f"Meters Per Unit: {UsdGeom.GetStageMetersPerUnit(stage)}")
print(f"Up Axis: {UsdGeom.GetStageUpAxis(stage)}")

simulation_app.close()
