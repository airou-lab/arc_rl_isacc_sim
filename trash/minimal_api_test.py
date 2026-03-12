from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("[Test] SimulationApp started.")
for i in range(10):
    simulation_app.update()
    print(f"[Test] Update {i}")

print("[Test] Success.")
# No close()
