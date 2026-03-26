# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Truly Bake Robot to Meter Scale.")
parser.add_argument("usd_path", type=str, help="Path to the source USD file.")
parser.add_argument("output_path", type=str, help="Path to the output USD file.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, UsdPhysics

def truly_bake(usd_path, output_path):
    print(f"Opening stage: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    
    # 1. Compute World Centers of all Rigid Bodies
    # We want to move these to origin-relative metric positions
    rb_world_pos = {}
    chassis_world_pos = Gf.Vec3d(0)
    
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            xformable = UsdGeom.Xformable(prim)
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            # Compute centroid of children meshes for better center
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
            bbox = bbox_cache.ComputeLocalBound(prim, Usd.TimeCode.Default(), "default")
            local_center = bbox.ComputeAlignedRange().GetMidpoint()
            
            world_center = world_transform.Transform(local_center) * mpu
            rb_world_pos[prim.GetPath().pathString] = world_center
            if "Chassis" in prim.GetName():
                chassis_world_pos = world_center
                print(f"Chassis world center (meters): {chassis_world_pos}")

    # 2. Create new stage
    new_stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageMetersPerUnit(new_stage, 1.0)
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)
    root_prim = UsdGeom.Xform.Define(new_stage, "/Robot")
    new_stage.SetDefaultPrim(root_prim.GetPrim())

    # 3. Create Rigid Bodies at their relative-to-chassis world positions
    # and bake their meshes
    for prim_path, wpos in rb_world_pos.items():
        # Relative position to chassis
        rel_pos = wpos - chassis_world_pos
        # Special case: center the chassis at (0,0,0)?
        # For now let's just keep everything relative.
        
        # New prim name
        new_name = prim_path.split("/")[-1]
        new_path = f"/Robot/{new_name}"
        new_rb = UsdGeom.Xform.Define(new_stage, new_path)
        new_rb.AddTranslateOp().Set(rel_pos)
        
        # Copy Physics
        UsdPhysics.RigidBodyAPI.Apply(new_rb.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(new_rb.GetPrim())
        mass.GetMassAttr().Set(4.0 if "Chassis" in new_name else 0.1)
        UsdPhysics.CollisionAPI.Apply(new_rb.GetPrim())
        
        # Bake Meshes
        old_rb_prim = stage.GetPrimAtPath(prim_path)
        for child in old_rb_prim.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                mesh_name = child.GetName()
                new_mesh_path = f"{new_path}/{mesh_name}"
                new_mesh = UsdGeom.Mesh.Define(new_stage, new_mesh_path)
                
                # Bake vertices relative to the NEW RB origin
                # OldWorldPos = wpos_units * mpu
                # NewWorldPos = rel_pos
                # We need to transform original vertices to world, then to the new local frame.
                
                old_mesh_xform = UsdGeom.Xformable(child)
                old_mesh_world_transform = old_mesh_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                
                points = child.GetAttribute("points").Get()
                if points:
                    baked_points = []
                    for p in points:
                        # World pos in meters
                        wp = old_mesh_world_transform.Transform(Gf.Vec3d(p)) * mpu
                        # Local pos in new RB frame (origin at wpos)
                        lp = wp - wpos
                        baked_points.append(Gf.Vec3f(lp))
                    new_mesh.GetPointsAttr().Set(baked_points)
                
                new_mesh.GetFaceVertexCountsAttr().Set(child.GetAttribute("faceVertexCounts").Get())
                new_mesh.GetFaceVertexIndicesAttr().Set(child.GetAttribute("faceVertexIndices").Get())

    # 4. Fix Joints
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            t0 = joint.GetBody0Rel().GetTargets()
            t1 = joint.GetBody1Rel().GetTargets()
            if t0 and t1:
                p0_path = t0[0].pathString
                p1_path = t1[0].pathString
                if p0_path in rb_world_pos and p1_path in rb_world_pos:
                    w0 = rb_world_pos[p0_path]
                    w1 = rb_world_pos[p1_path]
                    
                    new_path = f"/Robot/{prim.GetName()}"
                    # Re-create joint
                    type_name = prim.GetTypeName()
                    new_joint_prim = new_stage.DefinePrim(new_path, type_name)
                    new_joint = UsdPhysics.Joint(new_joint_prim)
                    
                    # Set targets to new paths
                    n0 = f"/Robot/{p0_path.split('/')[-1]}"
                    n1 = f"/Robot/{p1_path.split('/')[-1]}"
                    new_joint.GetBody0Rel().SetTargets([n0])
                    new_joint.GetBody1Rel().SetTargets([n1])
                    
                    # LocalPos0 = w1 - w0
                    new_joint.GetLocalPos0Attr().Set(Gf.Vec3f(w1 - w0))
                    new_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
                    
                    if "Revolute" in type_name:
                        rev = UsdPhysics.RevoluteJoint(new_joint_prim)
                        rev.GetAxisAttr().Set("Y" if "Wheel" in prim.GetName() else "Z")
                        drive = UsdPhysics.DriveAPI.Apply(new_joint_prim, "angular")
                        if "Wheel" in prim.GetName():
                            drive.GetTypeAttr().Set("velocity")
                        else:
                            drive.GetTypeAttr().Set("position")

    new_stage.GetRootLayer().Save()
    print(f"Truly Baked Robot saved to: {output_path}")

def main():
    truly_bake(args_cli.usd_path, args_cli.output_path)
    simulation_app.close()

if __name__ == "__main__":
    main()
