import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Generate a clean, fully-functioning F1Tenth robot USD.")
parser.add_argument("output_path", type=str, nargs='?', default="f1tenth_trainer/assets/F1Tenth_Generated.usd", help="Path to the output USD file.")
parser.add_argument("--scale", type=float, default=20.0, help="Scale factor (e.g., 1.0 for metric, 20.0 for giant).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdShade, Sdf

def generate_f1tenth_from_scratch(output_path, scale_factor):
    # F1Tenth Spec (e.g., 20x Giant Scale or 1.0x Metric Scale)
    SCALE = scale_factor
    wheelbase = 0.325 * SCALE
    track_width = 0.245 * SCALE
    wheel_radius = 0.052 * SCALE
    wheel_width = 0.04 * SCALE
    chassis_length = 0.45 * SCALE
    chassis_width = 0.20 * SCALE
    chassis_height = 0.05 * SCALE

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    # --- ROOT LINK ---
    root_prim = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(root_prim.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root_prim.GetPrim())
    # NO RigidBodyAPI here. The child links will hold them.

    # --- Materials ---
    def create_material(name, color):
        mat_path = f"/Robot/Looks/{name}"
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    UsdGeom.Scope.Define(stage, "/Robot/Looks")
    mat_chassis = create_material("Chassis", (0.1, 0.1, 0.2))
    mat_wheel = create_material("Wheel", (0.05, 0.05, 0.05))
    mat_knuckle = create_material("Knuckle", (0.5, 0.5, 0.5))

    # --- Links (Rigid Bodies) ---
    def create_link(name, pos, size, material, shape='Cube'):
        link_path = f"/Robot/{name}"
        link_prim = UsdGeom.Xform.Define(stage, link_path)
        link_prim.AddTranslateOp().Set(Gf.Vec3d(pos))
        
        UsdPhysics.RigidBodyAPI.Apply(link_prim.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(link_prim.GetPrim())
        mass.GetMassAttr().Set(4.0 if name == "Chassis" else 0.1)
        
        geom_path = f"{link_path}/Geom"
        if shape == 'Cube':
            geom = UsdGeom.Cube.Define(stage, geom_path)
            geom.CreateSizeAttr(1.0)
            geom.AddScaleOp().Set(Gf.Vec3f(size))
        elif shape == 'Cylinder':
            geom = UsdGeom.Cylinder.Define(stage, geom_path)
            geom.CreateRadiusAttr(size[0])
            geom.CreateHeightAttr(size[1])
            geom.CreateAxisAttr("Y")
        
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        return link_prim

    # Chassis
    chassis = create_link("Chassis", (0, 0, 0), (chassis_length, chassis_width, chassis_height), mat_chassis)
    
    z_offset = -chassis_height/2 
    
    # Knuckles
    create_link("Knuckle_L", (wheelbase/2, track_width/2, z_offset), (0.02, 0.02, 0.02), mat_knuckle)
    create_link("Knuckle_R", (wheelbase/2, -track_width/2, z_offset), (0.02, 0.02, 0.02), mat_knuckle)

    # Wheels
    wheel_size = (wheel_radius, wheel_width)
    create_link("Wheel_FL", (wheelbase/2, track_width/2, z_offset), wheel_size, mat_wheel, 'Cylinder')
    create_link("Wheel_FR", (wheelbase/2, -track_width/2, z_offset), wheel_size, mat_wheel, 'Cylinder')
    create_link("Wheel_RL", (-wheelbase/2, track_width/2, z_offset), wheel_size, mat_wheel, 'Cylinder')
    create_link("Wheel_RR", (-wheelbase/2, -track_width/2, z_offset), wheel_size, mat_wheel, 'Cylinder')

    # --- Joints ---
    def add_joint(name, p0_name, p1_name, axis, drive_type=None):
        j_path = f"/Robot/Joint_{name}"
        # CRITICAL FIX: Use the schema define method so attributes are created correctly
        joint = UsdPhysics.RevoluteJoint.Define(stage, j_path)
        
        joint.CreateBody0Rel().SetTargets([f"/Robot/{p0_name}"])
        joint.CreateBody1Rel().SetTargets([f"/Robot/{p1_name}"])
        
        p0_prim = stage.GetPrimAtPath(f"/Robot/{p0_name}")
        p1_prim = stage.GetPrimAtPath(f"/Robot/{p1_name}")
        
        p0_pos = Gf.Vec3d(UsdGeom.Xformable(p0_prim).GetLocalTransformation().GetRow(3)[:3])
        p1_pos = Gf.Vec3d(UsdGeom.Xformable(p1_prim).GetLocalTransformation().GetRow(3)[:3])
        
        # CRITICAL FIX: Use Create*Attr instead of Get*Attr
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(p1_pos - p0_pos))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        joint.CreateAxisAttr(axis)
        
        if drive_type:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr(drive_type)
            drive.CreateStiffnessAttr(1000.0 if drive_type == "position" else 0.0)
            drive.CreateDampingAttr(10.0)
            drive.CreateMaxForceAttr(100.0)

    add_joint("Steer_L", "Chassis", "Knuckle_L", "Z", "position")
    add_joint("Steer_R", "Chassis", "Knuckle_R", "Z", "position")
    add_joint("Drive_FL", "Knuckle_L", "Wheel_FL", "Y", "velocity")
    add_joint("Drive_FR", "Knuckle_R", "Wheel_FR", "Y", "velocity")
    add_joint("Drive_RL", "Chassis", "Wheel_RL", "Y", "velocity")
    add_joint("Drive_RR", "Chassis", "Wheel_RR", "Y", "velocity")

    stage.GetRootLayer().Save()
    print(f"Generated clean working robot at: {output_path}")

def main():
    import os
    out_path = args_cli.output_path
    if os.path.exists(out_path):
        os.remove(out_path)
    generate_f1tenth_from_scratch(out_path, args_cli.scale)
    simulation_app.close()

if __name__ == "__main__":
    main()
