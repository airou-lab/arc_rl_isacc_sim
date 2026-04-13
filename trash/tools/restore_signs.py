import sys
import os

# Set up paths properly
PROJECT_ROOT = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim"
sys.path.append(PROJECT_ROOT)

import argparse
from isaaclab.app import AppLauncher

# Generate a minimal Isaac Lab app to access USD
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf, Sdf, Vt

def create_procedural_stop_sign(stage, path, translation, rotation, scale):
    # Create an Xform for the sign
    sign_xform = UsdGeom.Xform.Define(stage, path)
    
    # 1. THE POLE (Grey Cylinder)
    pole_path = path.AppendChild("Pole")
    pole = UsdGeom.Cylinder.Define(stage, pole_path)
    pole.GetRadiusAttr().Set(0.05)
    pole.GetHeightAttr().Set(2.5)
    pole.GetDisplayColorAttr().Set([(0.5, 0.5, 0.5)]) # Grey
    # Position pole
    pole_xform = UsdGeom.Xformable(pole)
    pole_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.25))
    
    # 2. THE SIGN (Red Disk/Octagon approximation)
    face_path = path.AppendChild("StopFace")
    face = UsdGeom.Cylinder.Define(stage, face_path)
    face.GetRadiusAttr().Set(0.4)
    face.GetHeightAttr().Set(0.05)
    face.GetDisplayColorAttr().Set([(0.8, 0.1, 0.1)]) # Red
    # Position face at top of pole, rotated forward
    face_xform = UsdGeom.Xformable(face)
    face_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.2))
    # Rotate cylinder to face "forward" (around X axis)
    face_xform.AddRotateXOp().Set(90)
    
    # Apply World Transform to the root Xform
    main_xform = UsdGeom.Xformable(sign_xform)
    main_xform.ClearXformOpOrder()
    main_xform.AddTranslateOp().Set(translation)
    main_xform.AddOrientOp().Set(Gf.Quatf(rotation))
    
    # Adjust scale for 8x environment
    # If the original scale was small, we use a default visible size
    final_scale = Gf.Vec3f(8.0, 8.0, 8.0)
    main_xform.AddScaleOp().Set(final_scale)
    
    return sign_xform

def restore_stop_signs(usd_path):
    print(f"\nProcedurally Restoring Stop Signs in: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("FAILED to open USD")
        return

    # Prims to replace
    targets = [
        "/World/signs/sign_r1_1_0", "/World/signs/sign_r1_1_1", "/World/signs/sign_r1_1_2",
        "/World/signs/sign_r1_1_3", "/World/signs/sign_r1_1_4", "/World/signs/sign_r1_1_5",
        "/World/signs/sign_r1_1_6", "/World/signs/sign_r1_1_7"
    ]
    
    # Also look for props that might be signs
    for prim in stage.Traverse():
        if "SignPost" in prim.GetName() and "inst" in prim.GetName():
            targets.append(str(prim.GetPath()))

    restored_count = 0
    for path_str in targets:
        prim = stage.GetPrimAtPath(path_str)
        if not prim.IsValid():
            continue
            
        print(f"\nProcessing: {path_str}")
        
        # 1. Get Transform
        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = world_transform.ExtractTranslation()
        rotation = world_transform.ExtractRotation().GetQuat()
        
        # 2. Remove broken prim
        parent_path = prim.GetParent().GetPath()
        name = prim.GetName()
        stage.RemovePrim(path_str)
        
        # 3. Create Procedural Sign
        new_path = parent_path.AppendChild(f"{name}_restored")
        create_procedural_stop_sign(stage, new_path, translation, rotation, None)
        
        print(f"  - Created procedural restored sign at: {new_path}")
        restored_count += 1

    output_path = usd_path.replace(".usd", "_with_signs.usd")
    print(f"\nSaving restored stage to: {output_path}")
    print(f"Total signs restored: {restored_count}")
    stage.GetRootLayer().Export(output_path)
    
    # Also overwrite the 'clean' one so the user has a ready-to-use version
    clean_path = usd_path.replace(".usd", "_clean.usd")
    print(f"Saving copy to: {clean_path}")
    stage.GetRootLayer().Export(clean_path)
    
    return output_path

if __name__ == "__main__":
    track_path = os.path.join(PROJECT_ROOT, "openStreetUSD", "no_graph_sim.usd")
    restore_stop_signs(track_path)
    simulation_app.close()
