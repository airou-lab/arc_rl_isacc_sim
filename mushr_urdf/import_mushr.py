import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
from pxr import Usd, Sdf

urdf_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/mushr_urdf/mushr_patched.urdf"
dest_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/mushr_urdf/mushr/mushr_fixed.usd"

# Set import options
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.make_default_prim = True

print(f"Starting URDF import from: {urdf_path}")
success = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path=dest_path,
)

if success:
    print(f"Successfully imported URDF to {dest_path}")
    stage = Usd.Stage.Open(dest_path)
    
    # Importer usually sets it, but let's be sure
    if not stage.GetDefaultPrim():
        root_prims = [p for p in stage.GetPseudoRoot().GetChildren()]
        if root_prims:
            print(f"Manually setting default prim to: {root_prims[0].GetName()}")
            stage.SetDefaultPrim(root_prims[0])
            stage.GetRootLayer().Save()
    else:
        print(f"Default prim already set to: {stage.GetDefaultPrim().GetName()}")
else:
    print("Import failed.")

simulation_app.close()
