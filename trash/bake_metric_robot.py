from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf, UsdPhysics
import os

# Paths
original_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth.usd"
baked_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"

# 1. Create a transient stage to perform the scaling
temp_stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(temp_stage, 1.0)
UsdGeom.SetStageUpAxis(temp_stage, UsdGeom.Tokens.z)

# Reference the original car
ref_prim = temp_stage.DefinePrim("/F1Tenth", "Xform")
ref_prim.GetReferences().AddReference(original_usd)

# 2. Apply the Scaling (Baking preparation)
# Based on earlier diagnostics: 
# X: 32.0 raw -> 0.25m target => 0.0078125
# Y: 24.0 raw -> 0.24m target => 0.01
# Z: 0.01 consistency
xform = UsdGeom.Xformable(ref_prim)
xform.ClearXformOpOrder()
xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0078125, 0.01, 0.01))

# 3. Flatten the stage
# This "bakes" the reference and the scale into a single set of geometry
print(f"[Phase 2.5] Flattening {original_usd} into baked metric version...")
flattened_layer = temp_stage.Flatten()
flattened_layer.Export(baked_usd)

# 4. Final Polish: Open the baked file to set Mass and Metadata
final_stage = Usd.Stage.Open(baked_usd)
UsdGeom.SetStageMetersPerUnit(final_stage, 1.0)

# Apply Mass (4.092kg)
# In the flattened version, the path might have changed slightly or preserved /F1Tenth
chassis_path = "/F1Tenth/Rigid_Bodies/Chassis/Chassis"
chassis_prim = final_stage.GetPrimAtPath(chassis_path)
if chassis_prim.IsValid():
    mass_api = UsdPhysics.MassAPI.Apply(chassis_prim)
    mass_api.GetMassAttr().Set(3.592) # 3.59kg chassis
    print(f"[Phase 2.5] Applied 3.592kg to {chassis_path}")

wheels = [
    "/F1Tenth/Rigid_Bodies/Wheel_Front_Left/Wheel_Front_Left",
    "/F1Tenth/Rigid_Bodies/Wheel_Front_Right/Wheel_Front_Right",
    "/F1Tenth/Rigid_Bodies/Wheel_Rear_Left/Wheel_Rear_Left",
    "/F1Tenth/Rigid_Bodies/Wheel_Rear_Right/Wheel_Rear_Right"
]
for w in wheels:
    w_prim = final_stage.GetPrimAtPath(w)
    if w_prim.IsValid():
        mass_api = UsdPhysics.MassAPI.Apply(w_prim)
        mass_api.GetMassAttr().Set(0.125) # ~0.5kg total wheels
        print(f"[Phase 2.5] Applied 0.125kg to {w}")

final_stage.GetRootLayer().Save()
print(f"\n[Phase 2.5] SUCCESS: Baked Metric Digital Twin created at: {baked_usd}")

simulation_app.close()
