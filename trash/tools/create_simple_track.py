# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to create a simple two-lane track in a USD stage.
"""

import argparse
from pxr import Usd, UsdGeom, Gf, Sdf, UsdShade

def create_simple_track(output_path):
    # 1. Create a new USD stage
    stage = Usd.Stage.CreateNew(output_path)
    
    # 2. Set Units and Up-Axis
    UsdGeom.SetStageMetersPerUnit(stage, 1.0) # Set units to meters
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z) # Set Z as up-axis

    # Create a root scope for organization
    track_scope = UsdGeom.Scope.Define(stage, "/Track")
    stage.SetDefaultPrim(track_scope.GetPrim())

    # 3. Create Road Plane
    road_prim = UsdGeom.Plane.Define(stage, "/Track/Road")

    # Set plane dimensions (default plane is 1x1, let's scale it)
    road_prim.GetWidthAttr().Set(100.0) # 100 meters wide
    road_prim.GetLengthAttr().Set(200.0) # 200 meters long
    # Position the plane at z=0
    road_xform = UsdGeom.Xformable(road_prim.GetPrim())
    translate_op = road_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0)) # Flat on the ground
    
    # Optional: Add a simple material for the road
    material_path = Sdf.Path("/Track/Materials/RoadMaterial")
    road_material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path.AppendChild("PBRShader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.3, 0.3, 0.3)) # Dark grey
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    road_material.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(road_prim.GetPrim()).Bind(road_material)


    # 4. Create Lane Markers (using Cylinders for simplicity)
    lane_width = 3.0 # Distance from centerline to each lane marker
    marker_radius = 0.1 # Thin markers
    marker_height = 0.05 # Slightly above the road surface
    marker_length = road_prim.GetLengthAttr().Get() # Same length as road

    # Left Lane Marker
    left_marker_prim = UsdGeom.Cylinder.Define(stage, "/Track/LeftLaneMarker")
    left_marker_prim.SetPrimAsDefined()
    left_marker_prim.GetRadiusAttr().Set(marker_radius)
    left_marker_prim.GetHeightAttr().Set(marker_height)
    left_marker_prim.GetAxisAttr().Set(UsdGeom.Tokens.y) # Cylinder along Y-axis for length
    
    left_marker_xform = UsdGeom.Xformable(left_marker_prim.GetPrim())
    left_marker_translate_op = left_marker_xform.AddTranslateOp()
    left_marker_translate_op.Set(Gf.Vec3d(-lane_width, marker_length / 2.0, marker_height / 2.0)) # Halfway along the length, offset by lane_width
    left_marker_xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0)) # Rotate to lie flat on the ground

    # Right Lane Marker
    right_marker_prim = UsdGeom.Cylinder.Define(stage, "/Track/RightLaneMarker")
    right_marker_prim.SetPrimAsDefined()
    right_marker_prim.GetRadiusAttr().Set(marker_radius)
    right_marker_prim.GetHeightAttr().Set(marker_height)
    right_marker_prim.GetAxisAttr().Set(UsdGeom.Tokens.y) # Cylinder along Y-axis for length
    
    right_marker_xform = UsdGeom.Xformable(right_marker_prim.GetPrim())
    right_marker_translate_op = right_marker_xform.AddTranslateOp()
    right_marker_translate_op.Set(Gf.Vec3d(lane_width, marker_length / 2.0, marker_height / 2.0)) # Halfway along the length, offset by lane_width
    right_marker_xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0)) # Rotate to lie flat on the ground

    # Optional: Add a simple material for lane markers
    marker_material_path = Sdf.Path("/Track/Materials/MarkerMaterial")
    marker_material = UsdShade.Material.Define(stage, marker_material_path)
    marker_shader = UsdShade.Shader.Define(stage, marker_material_path.AppendChild("PBRShader"))
    marker_shader.CreateIdAttr("UsdPreviewSurface")
    marker_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0)) # White markers
    marker_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    marker_material.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(marker_shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    
    left_marker_prim.GetPrim().ApplyRelationship("material:binding").AddTarget(marker_material_path)
    right_marker_prim.GetPrim().ApplyRelationship("material:binding").AddTarget(marker_material_path)

    # 5. Save USD
    stage.GetRootLayer().Save()
    print(f"Simple two-lane track USD saved to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Create a simple two-lane track in a USD stage.")
    parser.add_argument("output_path", type=str, help="Path to the output USD file.")
    args = parser.parse_args()

    create_simple_track(args.output_path)

if __name__ == "__main__":
    # Ensure UsdShade is available
    from pxr import UsdShade
    main()
