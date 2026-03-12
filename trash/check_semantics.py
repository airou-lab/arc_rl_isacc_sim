from pxr import Usd, Semantics
import sys

usd_path = sys.argv[1]
stage = Usd.Stage.Open(usd_path)

print(f"--- Semantic Label Audit for {usd_path} ---")
for prim in stage.Traverse():
    if prim.HasAPI(Semantics.SemanticsAPI):
        sem = Semantics.SemanticsAPI(prim)
        label = sem.GetSemanticTypeAttr().Get()
        data = sem.GetSemanticDataAttr().Get()
        print(f"Prim: {prim.GetPath()} | Type: {label} | Data: {data}")
