
import os
from isaacsim import SimulationApp

# Start simulation app
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.client

def find_nvidia_one():
    assets_root = get_assets_root_path()
    if not assets_root: return

    # The current root is /Isaac/5.0
    # Let's try to find where high-quality streets were actually stored
    base_url = assets_root.split("/Isaac")[0]
    
    # Try different official version paths
    paths = [
        f"{base_url}/Isaac/4.2/Isaac/Environments/Urban",
        f"{base_url}/Isaac/2023.1.1/Isaac/Environments/Simple_Road",
        f"{base_url}/Isaac/2022.2.1/Isaac/Environments/Urban"
    ]
    
    print(f"\n--- Final Search for NVIDIA Official Streets ---")
    
    for path in paths:
        print(f"\nScanning: {path}")
        result, entries = omni.client.list(path)
        if result == omni.client.Result.OK:
            for entry in entries:
                print(f" FOUND: {path}/{entry.relative_path}")
        else:
            print(" Path not found.")

    simulation_app.close()

if __name__ == "__main__":
    find_nvidia_one()
