import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Harden a USD by adding collisions to all meshes.")
parser.add_argument("input", type=str, help="Input USD path.")
parser.add_argument("--output", type=str, help="Output USD path (optional).", default=None)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade

def harden_usd(input_path, output_path=None):
    print(f"Opening: {input_path}")
    stage = Usd.Stage.Open(input_path)
    if not stage:
        print("Failed to open stage.")
        return

    # Create a Physics Material for the road
    material_path = "/World/RoadPhysicsMaterial"
    if not stage.GetPrimAtPath(material_path):
        material_prim = stage.DefinePrim(material_path, "Material")
        UsdPhysics.MaterialAPI.Apply(material_prim)
        road_material = UsdPhysics.MaterialAPI(material_prim)
        road_material.GetStaticFrictionAttr().Set(1.0)
        road_material.GetDynamicFrictionAttr().Set(0.8)
        road_material.GetRestitutionAttr().Set(0.1)
        print(f"Created Physics Material: {material_path}")

    count = 0
    # Use stage.TraverseAll() to catch everything including hidden/internal prims
    for prim in stage.TraverseAll():
        if prim.IsA(UsdGeom.Mesh):
            path = str(prim.GetPath()).lower()
            is_road = any(k in path for k in ["drivable", "pavement", "road"])

            # Apply CollisionAPI
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)

            # Apply MeshCollisionAPI
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_coll.GetApproximationAttr().Set("none") # Triangle Mesh for better accuracy on track
            # Ensure Double Sided
            mesh = UsdGeom.Mesh(prim)
            mesh.GetDoubleSidedAttr().Set(True)

            # Add Physx settings and BIND MATERIAL if it's road
            physx_coll = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_coll.GetContactOffsetAttr().Set(0.2) # Increased for 8x scale
            physx_coll.GetRestOffsetAttr().Set(0.0)

            if is_road:
                binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
                binding_api.Bind(UsdShade.Material(stage.GetPrimAtPath(material_path)), UsdShade.Tokens.strongerThanDescendants, "physics")

            count += 1


    if output_path:
        stage.GetRootLayer().Export(output_path)
        print(f"Hardened {count} meshes. Saved to: {output_path}")
    else:
        stage.GetRootLayer().Save()
        print(f"Hardened {count} meshes in-place: {input_path}")

def harden_robot(robot_path):
    print(f"Opening Robot: {robot_path}")
    stage = Usd.Stage.Open(robot_path)
    if not stage: return

    for prim in stage.TraverseAll():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Apply MassAPI if missing
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            # For 8x robot, we need real mass and inertia
            # 400kg is a balanced mass for an 8m long car in this sim
            mass_api.GetMassAttr().Set(400.0)
            mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            # Set a generic solid-box inertia for stability
            mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(10.0, 10.0, 10.0))
            print(f"Baked mass properties (400kg) into: {prim.GetPath()}")

    stage.GetRootLayer().Save()

if __name__ == "__main__":
    if "F1Tenth" in args_cli.input:
        harden_robot(args_cli.input)
    else:
        harden_usd(args_cli.input, args_cli.output)
    simulation_app.close()
