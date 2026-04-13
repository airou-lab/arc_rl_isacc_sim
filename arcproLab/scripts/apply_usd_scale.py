import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Apply global scale to a USD file.")
parser.add_argument("input", type=str, help="Input USD file.")
parser.add_argument("output", type=str, help="Output USD file.")
parser.add_argument("--scale", type=float, default=0.125, help="Scale factor.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
from pxr import Usd, UsdGeom, Gf

def main():
    stage = Usd.Stage.Open(args_cli.input)
    if not stage:
        print(f"Failed to open {args_cli.input}")
        return

    # To avoid compounding, we only scale the "entry points" of the hierarchy
    # which are either root-level prims OR prims that author references.
    
def main():
    stage = Usd.Stage.Open(args_cli.input)
    if not stage:
        print(f"Failed to open {args_cli.input}")
        return

    # Remove unwanted prims (Grass, fences, etc as requested)
    unwanted = ["/World/grass", "/World/foliage", "/World/fences"]
    for path in unwanted:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            print(f"Removing {path}...")
            stage.RemovePrim(path)

    # Scale everything in World to 0.125
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xformable):
            # Use XformCommonAPI to set absolute scale
            common_api = UsdGeom.XformCommonAPI(prim)
            # This sets the scale of the prim. 
            # Note: This might overwrite existing translate/rotate if not careful, 
            # but usually it adds to the stack.
            # Actually, let's just use the scale op directly but ENSURE it's 0.125
            
            xform = UsdGeom.Xformable(prim)
            scale_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_op = op
                    break
            
            if not scale_op:
                scale_op = xform.AddScaleOp()
            
            # If it's a child of /World, we want it to be 0.125 total.
            # Inheritance: scale_world * scale_child = 0.125
            # If we set scale_world = 0.125 and scale_child = 1.0, we get 0.125.
            # If child already has 8.0, we MUST override it to 1.0 or 0.125.
            
            path = str(prim.GetPath())
            if path == "/World":
                scale_op.Set(Gf.Vec3f(args_cli.scale, args_cli.scale, args_cli.scale))
                print(f"Scaled /World to {args_cli.scale}")
            elif path.startswith("/World/"):
                # Reset children to 1.0 so they inherit World's 0.125
                scale_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
                # print(f"Reset child scale: {path}")
    
    output_path = args_cli.output
    if not output_path.endswith(".usda"):
        output_path = output_path.replace(".usd", ".usda")
    
    stage.GetRootLayer().Export(output_path)
    print(f"Saved to {output_path}")
    simulation_app.close()

if __name__ == "__main__":
    main()
