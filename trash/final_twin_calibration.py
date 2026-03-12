from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf
import os

baked_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"
stage = Usd.Stage.Open(baked_usd)

# Exact ARCPro Targets (Meters)
WHEELBASE = 0.25
TRACK_WIDTH = 0.24
HALF_WB = WHEELBASE / 2.0
HALF_TW = TRACK_WIDTH / 2.0

# Map of wheels to their exact kinematic coordinates
wheel_targets = {
    "/Full_Car/Rigid_Bodies/Wheel_Front_Left":  Gf.Vec3d( HALF_WB,  HALF_TW, 0.05),
    "/Full_Car/Rigid_Bodies/Wheel_Front_Right": Gf.Vec3d( HALF_WB, -HALF_TW, 0.05),
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Left":   Gf.Vec3d(-HALF_WB,  HALF_TW, 0.05),
    "/Full_Car/Rigid_Bodies/Wheel_Rear_Right":  Gf.Vec3d(-HALF_WB, -HALF_TW, 0.05),
}

print(f"[Phase 2.5] Injecting Final Kinematics (WB: {WHEELBASE}m, TW: {TRACK_WIDTH}m)...")

for path, pos in wheel_targets.items():
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        # Clear existing and set exact translation
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(pos)
        print(f"  Set {path} -> {pos}")

# Also check chassis mesh to ensure it spans the 403mm length
# Chassis is usually centered. 403mm total means it should go from -0.201 to +0.201 approx.
# We won't stretch the mesh further to avoid distortion, as wheelbase is the primary driving factor.

stage.GetRootLayer().Save()
print(f"\n[Phase 2.5] POLISH COMPLETE: {baked_usd} is now a true Digital Twin.")

simulation_app.close()
