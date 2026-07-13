import os
try:
    from pxr import Usd, UsdGeom
    print("SUCCESS: pxr imported")
except ImportError as e:
    print(f"FAILURE: {e}")
