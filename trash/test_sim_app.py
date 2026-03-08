
import os
print("Starting script...")
from isaacsim import SimulationApp
print("Imported SimulationApp. Initializing...")
simulation_app = SimulationApp({"headless": False})
print("SimulationApp initialized successfully!")
simulation_app.close()
print("SimulationApp closed.")
