from pxr import Usd
import sys

usd_path = sys.argv[1]
stage = Usd.Stage.Open(usd_path)

print(f"--- Attribute Audit for {usd_path} ---")
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        if "semantic" in attr.GetName().lower():
            print(f"Prim: {prim.GetPath()} | Attr: {attr.GetName()} | Value: {attr.Get()}")
