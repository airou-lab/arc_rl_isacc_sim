from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import inspect
from omni.isaac.core.controllers import ArticulationController

print("\n--- ArticulationController.__init__ Signature ---")
print(inspect.signature(ArticulationController.__init__))

print("\n--- ArticulationController.apply_action Signature ---")
print(inspect.signature(ArticulationController.apply_action))

simulation_app.close()
