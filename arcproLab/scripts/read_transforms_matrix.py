# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Read raw joint translations from matrices.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf
import os

def get_translation(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim: return None
    xform = UsdGeom.Xformable(prim)
    # Compute local transform (relative to parent)
    local_to_parent = xform.GetLocalTransformation(Usd.TimeCode.Default())
    translation = local_to_parent.ExtractTranslation()
    return translation

def main():
    usd_path = "f1tenth_trainer/assets/F1Tenth_Modern.usd"
    stage = Usd.Stage.Open(usd_path)
    
    # We need to trace the hierarchy from /Full_Car down to find the axles
    # Let's check some key prims
    paths = [
        "/Full_Car/Joints/Wheel__Knuckle__Front_Left",
        "/Full_Car/Joints/Wheel__Upright__Rear_Left",
        "/Full_Car/Rigid_Bodies/Knuckle_Front_Left",
        "/Full_Car/Rigid_Bodies/Upright_Rear_Left"
    ]
    
    for p in paths:
        t = get_translation(stage, p)
        print(f"[Native] Translation {p}: {t}")

    # Let's try to find the total distance along X between a front and rear link
    front_link = "/Full_Car/Rigid_Bodies/Knuckle_Front_Left"
    rear_link = "/Full_Car/Rigid_Bodies/Upright_Rear_Left"
    
    # Trace front
    curr = stage.GetPrimAtPath(front_link)
    f_total_x = 0
    while curr and curr.GetPath() != "/Full_Car":
        t = get_translation(stage, curr.GetPath())
        if t: f_total_x += t[0]
        curr = curr.GetParent()
        
    # Trace rear
    curr = stage.GetPrimAtPath(rear_link)
    r_total_x = 0
    while curr and curr.GetPath() != "/Full_Car":
        t = get_translation(stage, curr.GetPath())
        if t: r_total_x += t[0]
        curr = curr.GetParent()
        
    native_wheelbase = abs(f_total_x - r_total_x)
    print(f"\n[Native] Calculated Native Wheelbase: {native_wheelbase:.4f} units")
    
    # Trace Y for track width
    curr = stage.GetPrimAtPath(front_link)
    f_total_y = 0
    while curr and curr.GetPath() != "/Full_Car":
        t = get_translation(stage, curr.GetPath())
        if t: f_total_y += t[1]
        curr = curr.GetParent()
    
    native_track_width = abs(f_total_y * 2.0) # Assuming symmetry
    print(f"[Native] Calculated Native Track Width: {native_track_width:.4f} units")

    simulation_app.close()

if __name__ == "__main__":
    main()
