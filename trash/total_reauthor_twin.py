from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Vt
import os

original_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth.usd"
baked_usd = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/f1tenth_trainer/assets/F1Tenth_Metric_Baked.usd"

if os.path.exists(baked_usd): os.remove(baked_usd)

# 1. Open Original and Export to New File to work on a copy
stage = Usd.Stage.Open(original_usd)
stage.GetRootLayer().Export(baked_usd)
stage = Usd.Stage.Open(baked_usd)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Scaling factors to hit ARCPro specs
SCALE_X = 0.0078125 
SCALE_Y = 0.01      
SCALE_Z = 0.01      

def scale_value(val):
    if isinstance(val, Gf.Vec3d):
        return Gf.Vec3d(val[0] * SCALE_X, val[1] * SCALE_Y, val[2] * SCALE_Z)
    if isinstance(val, Gf.Vec3f):
        return Gf.Vec3f(val[0] * SCALE_X, val[1] * SCALE_Y, val[2] * SCALE_Z)
    return val

print(f"[Phase 2.5] Starting Atomic Vertex Scaling...")

for prim in stage.Traverse():
    # A. Scale Transforms (Translations)
    if prim.IsA(UsdGeom.Xformable):
        xformable = UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                old_t = op.Get()
                if old_t:
                    new_t = scale_value(old_t)
                    op.Set(new_t)
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3d(1,1,1))

    # B. Scale Mesh Vertices
    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        points_attr = mesh.GetPointsAttr()
        points = points_attr.Get()
        if points:
            new_points = Vt.Vec3fArray(len(points), [scale_value(p) for p in points])
            points_attr.Set(new_points)

# 3. Post-Process Sensors & Mass
sensors = {
    "/Full_Car/Rigid_Bodies/Chassis/Lidar": Gf.Vec3d(0.235, 0.0, 0.26523),
    "/Full_Car/Rigid_Bodies/Chassis/Camera_Left": Gf.Vec3d(0.145, 0.0, 0.195)
}

for path, pos in sensors.items():
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        found = False
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(pos)
                found = True
        if not found:
            xform.AddTranslateOp().Set(pos)

# Mass
chassis_prim = stage.GetPrimAtPath("/Full_Car/Rigid_Bodies/Chassis/Chassis")
if chassis_prim.IsValid():
    UsdPhysics.MassAPI.Apply(chassis_prim).GetMassAttr().Set(3.592)

stage.GetRootLayer().Save()
print(f"\n[Phase 2.5] ATOMIC SUCCESS: Digital Twin authored at {baked_usd}")

simulation_app.close()
