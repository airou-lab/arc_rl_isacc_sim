
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom

usd_path = "openStreetUSD/no_graph_sim_clean_1x.usda"
stage = Usd.Stage.Open(usd_path)
print(f"External References: {stage.GetRootLayer().externalReferences}")

# Check for references in prims
for prim in stage.Traverse():
    if prim.HasAuthoredReferences():
        print(f"Prim {prim.GetPath()} has references")

simulation_app.close()
