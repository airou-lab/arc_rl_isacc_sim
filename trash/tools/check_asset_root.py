from isaaclab.utils.assets import NVIDIA_ASSET_ROOT
import os

print(f"NVIDIA_ASSET_ROOT: {NVIDIA_ASSET_ROOT}")

# Common path for Stop Sign in modern Isaac Sim
stop_sign_path = f"{NVIDIA_ASSET_ROOT}/Isaac/Props/Signs/Stop_Sign.usd"
print(f"Potential stop sign path: {stop_sign_path}")

# Check if it exists (might not work for URLs but good for local)
# For URLs, we just have to trust them or try to open them with Usd.Stage.Open
