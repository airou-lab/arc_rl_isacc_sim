from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import inspect
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationAction

print("\n--- ArticulationView.set_gains Signature ---")
print(inspect.signature(ArticulationView.set_gains))

print("\n--- ArticulationAction.__init__ Signature ---")
print(inspect.signature(ArticulationAction.__init__))

simulation_app.close()
