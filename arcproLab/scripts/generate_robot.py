# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Generate a clean, primitive-based F1Tenth robot USD.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdShade, Sdf

def generate_f1tenth_from_scratch(output_path):
    # F1Tenth Spec (in meters)
    wheelbase = 0.325
    track_width = 0.245
    wheel_radius = 0.052
    wheel_width = 0.04
    chassis_length = 0.45
    chassis_width = 0.20
    chassis_height = 0.05

    # Create new stage
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    root_prim = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(root_prim.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root_prim.GetPrim())

    # --- Materials ---
    def create_material(name, color):
        mat_path = f"/Robot/Looks/{name}"
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

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
        
        # Visual and Collision Geometry
        geom_path = f"{link_path}/Geom"
        if shape == 'Cube':
            geom = UsdGeom.Cube.Define(stage, geom_path)
            geom.GetSizeAttr().Set(1.0)
            geom.AddScaleOp().Set(Gf.Vec3f(size))
        elif shape == 'Cylinder':
            geom = UsdGeom.Cylinder.Define(stage, geom_path)
            geom.GetRadiusAttr().Set(size[0])
            geom.GetHeightAttr().Set(size[1])
            geom.GetAxisAttr().Set("Y")
        
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        return link_prim

    # Create Chassis
    chassis_pos = (0, 0, wheel_radius + chassis_height/2)
    chassis = create_link("Chassis", chassis_pos, (chassis_length, chassis_width, chassis_height), mat_chassis)
    
    # Create Knuckles
    knuckle_z = wheel_radius
    knuckle_l = create_link("Knuckle_L", (wheelbase/2, track_width/2, knuckle_z), (0.02, 0.02, 0.02), mat_knuckle)
    knuckle_r = create_link("Knuckle_R", (wheelbase/2, -track_width/2, knuckle_z), (0.02, 0.02, 0.02), mat_knuckle)

    # Create Wheels
    wheel_l_pos = (wheelbase/2, track_width/2, wheel_radius)
    wheel_r_pos = (wheelbase/2, -track_width/2, wheel_radius)
    rear_wheel_l_pos = (-wheelbase/2, track_width/2, wheel_radius)
    rear_wheel_r_pos = (-wheelbase/2, -track_width/2, wheel_radius)
    
    wheel_size = (wheel_radius, wheel_width)
    create_link("Wheel_FL", wheel_l_pos, wheel_size, mat_wheel, 'Cylinder')
    create_link("Wheel_FR", wheel_r_pos, wheel_size, mat_wheel, 'Cylinder')
    create_link("Wheel_RL", rear_wheel_l_pos, wheel_size, mat_wheel, 'Cylinder')
    create_link("Wheel_RR", rear_wheel_r_pos, wheel_size, mat_wheel, 'Cylinder')

    # --- Joints ---
    def add_joint(name, p0_name, p1_name, axis, drive_type=None):
        j_path = f"/Robot/Joint_{name}"
        j_prim = stage.DefinePrim(j_path, "PhysicsRevoluteJoint")
        joint = UsdPhysics.RevoluteJoint(j_prim)
        joint.GetBody0Rel().SetTargets([f"/Robot/{p0_name}"])
        joint.GetBody1Rel().SetTargets([f"/Robot/{p1_name}"])
        
        p0_pos = Gf.Vec3d(UsdGeom.Xformable(stage.GetPrimAtPath(f"/Robot/{p0_name}")).GetLocalTransformation().GetRow(3)[:3])
        p1_pos = Gf.Vec3d(UsdGeom.Xformable(stage.GetPrimAtPath(f"/Robot/{p1_name}")).GetLocalTransformation().GetRow(3)[:3])
        
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(p1_pos - p0_pos))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        joint.GetAxisAttr().Set(axis)
        
        if drive_type:
            drive = UsdPhysics.DriveAPI.Apply(j_prim, "angular")
            drive.GetTypeAttr().Set(drive_type)
            drive.GetStiffnessAttr().Set(1000.0 if drive_type == "position" else 0.0)
            drive.GetDampingAttr().Set(10.0)
            drive.GetMaxForceAttr().Set(1000.0)

    add_joint("Steer_L", "Chassis", "Knuckle_L", "Z", "position")
    add_joint("Steer_R", "Chassis", "Knuckle_R", "Z", "position")
    add_joint("Drive_FL", "Knuckle_L", "Wheel_FL", "Y", "velocity")
    add_joint("Drive_FR", "Knuckle_R", "Wheel_FR", "Y", "velocity")
    add_joint("Drive_RL", "Chassis", "Wheel_RL", "Y", "velocity")
    add_joint("Drive_RR", "Chassis", "Wheel_RR", "Y", "velocity")

    stage.GetRootLayer().Save()
    print(f"Generated clean F1Tenth robot at: {output_path}")

def main():
    generate_f1tenth_from_scratch(args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
