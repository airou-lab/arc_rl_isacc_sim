import argparse
import os
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, Gf

input_usd = "arcproLab/assets/robot/F1Tenth_Metric.usd"
output_usd = "arcproLab/assets/robot/F1Tenth_Metric_Fixed.usd"

# If the output file already exists, remove it so we start fresh from the original
if os.path.exists(output_usd):
    os.remove(output_usd)

# Copy the original file to the new location first to preserve everything else
import shutil
shutil.copy2(input_usd, output_usd)

# Open the NEW file
stage = Usd.Stage.Open(output_usd)

geom = stage.GetPrimAtPath("/Robot/Chassis/Geom")
if not geom.IsValid():
    print("Could not find /Robot/Chassis/Geom!")
else:
    # Rotate the geom 180 degrees around Z axis
    xform = UsdGeom.Xformable(geom)
    # Check if there's already a rotate op
    rotate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() in [UsdGeom.XformOp.TypeRotateXYZ, UsdGeom.XformOp.TypeRotateZ]:
            rotate_op = op
            break
            
    if rotate_op is None:
        rotate_op = xform.AddRotateZOp()
        
    current_rot = rotate_op.Get()
    if current_rot is None:
        if rotate_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            rotate_op.Set(Gf.Vec3f(0, 0, 180))
        else:
            rotate_op.Set(180)
    else:
        if rotate_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            rotate_op.Set(current_rot + Gf.Vec3f(0, 0, 180))
        else:
            rotate_op.Set(current_rot + 180)
            
    stage.Save()
    print("Successfully rotated /Robot/Chassis/Geom 180 degrees in new file!")

simulation_app.close()
