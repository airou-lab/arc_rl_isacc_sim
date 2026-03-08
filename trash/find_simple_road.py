
import os
from isaacsim import SimulationApp

# Start simulation app
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.client

def find_simple_road():
    assets_root = get_assets_root_path()
    if not assets_root: return

    # The current root is /Isaac/5.0
    # Let's search the whole Environments folder
    path = f"{assets_root}/Isaac/Environments"
    
    print(f"\n--- Searching for 'simple_road.usd' in {path} ---")
    
    result, entries = omni.client.list(path)
    if result == omni.client.Result.OK:
        for entry in entries:
            name = entry.relative_path
            # Check if it's a folder
            if "." not in name:
                sub_path = f"{path}/{name}"
                sub_res, sub_entries = omni.client.list(sub_path)
                if sub_res == omni.client.Result.OK:
                    for sub in sub_entries:
                        if "simple_road.usd" in sub.relative_path.lower():
                            print(f" SUCCESS FOUND: {sub_path}/{sub.relative_path}")

    simulation_app.close()

if __name__ == "__main__":
    find_simple_road()
