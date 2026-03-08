
import os
import numpy as np
from isaacsim import SimulationApp

# Start simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

def apply_asphalt():
    usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
    print(f"Loading World0: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    assets_root = get_assets_root_path()
    
    # 1. Define the NVIDIA Asphalt Material path
    # Standard high-quality asphalt from Isaac Sim 5.0
    asphalt_url = f"{assets_root}/Isaac/Materials/Base/Asphalt/Asphalt_New.mdl"
    print(f"Using NVIDIA Asphalt: {asphalt_url}")

    # 2. Ensure we have a GroundPlane to apply it to
    # If /World/GroundPlane doesn't exist, create a large mesh
    ground_path = "/World/GroundPlane"
    ground_prim = stage.GetPrimAtPath(ground_path)
    if not ground_prim.IsValid():
        print("Creating new Ground Plane mesh...")
        plane = UsdGeom.Mesh.Define(stage, ground_path)
        size = 100.0
        plane.CreatePointsAttr([(-size, -size, 0), (size, -size, 0), (size, size, 0), (-size, size, 0)])
        plane.CreateFaceVertexCountsAttr([4])
        plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        # Add physics
        from pxr import UsdPhysics
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(plane.GetPrim()) # Static floor
    
    # 3. Create Material Prim
    mat_path = "/World/Looks/AsphaltMaterial"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        print("Creating Material Prim...")
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("mdlMaterial")
        shader.SetSourceAsset(asphalt_url, "mdl")
        shader.GetPrim().CreateAttribute("info:mdl:sourceAsset:subIdentifier", Sdf.ValueTypeNames.Token).Set("Asphalt_New")
        
        # Connect shader to material
        material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    
    # 4. Bind Material to Ground
    print(f"Binding Asphalt to {ground_path}...")
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(ground_path)).Bind(UsdShade.Material(stage.GetPrimAtPath(mat_path)))

    # 5. Add a simple Yellow Center Line (for "Street" look)
    line_path = "/World/CenterLine"
    if not stage.GetPrimAtPath(line_path).IsValid():
        print("Adding Yellow Center Line...")
        line = UsdGeom.Mesh.Define(stage, line_path)
        w, l = 0.1, 100.0 # 10cm wide, 100m long
        line.CreatePointsAttr([(-w, -l, 0.001), (w, -l, 0.001), (w, l, 0.001), (-w, l, 0.001)])
        line.CreateFaceVertexCountsAttr([4])
        line.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        line.CreateDisplayColorAttr([(1.0, 0.8, 0.0)]) # Yellow

    # 6. Save Stage
    omni.usd.get_context().save_stage()
    print("World0.usd updated with NVIDIA Asphalt and Center Line.")

    simulation_app.close()

if __name__ == "__main__":
    apply_asphalt()
