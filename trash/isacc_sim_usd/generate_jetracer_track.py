# generate_jetracer_track.py

from isaacsim import SimulationApp

# Initialize the SimulationApp before other imports
# headless=False allows you to see the viewport. Set to True for headless generation.
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.world import World
import numpy as np
import math
import os
import omni.kit.app

# --- Configuration ---
ROAD_WIDTH = 4.0
ROAD_THICKNESS = 0.1
LINE_WIDTH = 0.15
LINE_HEIGHT = 0.02  # Slightly above road to avoid z-fighting
LANE_WIDTH = 1.8

# --- Helper Functions ---

def create_material(stage, path, color):
    """Creates a simple colored material."""
    material = UsdShade.Material.Define(stage, path)
    pbrShader = UsdShade.Shader.Define(stage, f"{path}/PBRShader")
    pbrShader.CreateIdAttr("UsdPreviewSurface")
    pbrShader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    pbrShader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    pbrShader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(pbrShader.ConnectableAPI(), "surface")
    return material

def apply_material(prim, material):
    """Applies a material to a prim."""
    if isinstance(material, Usd.Prim):
        material = UsdShade.Material(material)
    binder = UsdShade.MaterialBindingAPI.Apply(prim)
    binder.Bind(material)

def add_physics_collider(prim):
    """Adds a static collider to a prim."""
    UsdPhysics.CollisionAPI.Apply(prim)

def create_road_segment(world, stage, name, position, rotation_deg, length, double_yellow=False):
    """
    Creates a straight road segment with lines.
    """
    road_path = f"/World/Roads/{name}"
    
    # 1. Create Asphalt Base
    prim_utils.create_prim(
        prim_path=road_path,
        prim_type="Cube",
        position=position,
        orientation=euler_angles_to_quat(np.array([0, 0, rotation_deg]), degrees=True),
        scale=(length / 2, ROAD_WIDTH / 2, ROAD_THICKNESS / 2), # Cube scale is half-extents
    )
    road_prim = stage.GetPrimAtPath(road_path)
    apply_material(road_prim, stage.GetPrimAtPath("/World/Materials/Asphalt"))
    add_physics_collider(road_prim) # Enable physics for driving

    # 2. Add White Edge Lines
    rad = math.radians(rotation_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    
    # Helper to place line relative to road center
    def place_line(line_name, offset_x, offset_y, color_path, scale_x=length/2, scale_y=LINE_WIDTH/2):
        # Local to World:
        wx = position[0] + (offset_x * cos_a - offset_y * sin_a)
        wy = position[1] + (offset_x * sin_a + offset_y * cos_a)
        wz = position[2] + ROAD_THICKNESS/2 + 0.01 # Slightly on top

        line_path = f"{road_path}/{line_name}"
        prim_utils.create_prim(
            prim_path=line_path,
            prim_type="Cube",
            position=(wx, wy, wz),
            orientation=euler_angles_to_quat(np.array([0, 0, rotation_deg]), degrees=True),
            scale=(scale_x, scale_y, LINE_HEIGHT/2)
        )
        apply_material(stage.GetPrimAtPath(line_path), stage.GetPrimAtPath(color_path))

    # Add White Lines
    place_line("WhiteLineLeft", 0, (ROAD_WIDTH/2 - LINE_WIDTH), "/World/Materials/White")
    place_line("WhiteLineRight", 0, -(ROAD_WIDTH/2 - LINE_WIDTH), "/World/Materials/White")

    # Add Yellow Center Lines
    if double_yellow:
        place_line("YellowLine1", 0, LINE_WIDTH, "/World/Materials/Yellow")
        place_line("YellowLine2", 0, -LINE_WIDTH, "/World/Materials/Yellow")
    else:
        place_line("YellowLineCenter", 0, 0, "/World/Materials/Yellow")

def create_intersection(world, stage, name, position):
    """
    Creates a simple 4-way intersection (asphalt only, no lines in the middle).
    """
    int_path = f"/World/Roads/{name}"
    
    # Intersection base (Square)
    prim_utils.create_prim(
        prim_path=int_path,
        prim_type="Cube",
        position=position,
        scale=(ROAD_WIDTH / 2, ROAD_WIDTH / 2, ROAD_THICKNESS / 2),
    )
    int_prim = stage.GetPrimAtPath(int_path)
    apply_material(int_prim, stage.GetPrimAtPath("/World/Materials/Asphalt"))
    add_physics_collider(int_prim)

def create_stop_sign(stage, name, position, rotation_deg):
    """
    Procedurally creates a stop sign.
    """
    base_path = f"/World/Props/{name}"
    
    # 1. Pole
    pole_path = f"{base_path}_Pole"
    prim_utils.create_prim(
        prim_path=pole_path,
        prim_type="Cylinder",
        position=(position[0], position[1], position[2] + 1.0), # 1m up
        scale=(0.05, 0.05, 1.0), # 2m tall
    )
    apply_material(stage.GetPrimAtPath(pole_path), stage.GetPrimAtPath("/World/Materials/Metal"))
    add_physics_collider(stage.GetPrimAtPath(pole_path))

    # 2. Sign
    sign_path = f"{base_path}_Sign"
    prim_utils.create_prim(
        prim_path=sign_path,
        prim_type="Cylinder",
        position=(position[0], position[1], position[2] + 2.0),
        orientation=euler_angles_to_quat(np.array([90, 0, rotation_deg + 90]), degrees=True), 
        scale=(0.4, 0.4, 0.05),
    )
    apply_material(stage.GetPrimAtPath(sign_path), stage.GetPrimAtPath("/World/Materials/Red"))


def create_road_mesh(stage, name, points_inner, points_outer, material_path, offset_z=0.0, skip_indices=None):
    """
    Creates a single continuous Mesh from a set of inner and outer boundary points.
    Optional skip_indices allows leaving gaps in the mesh (e.g. for intersections).
    """
    prim_path = f"/World/Roads/{name}"
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    
    vertices = []
    for i in range(len(points_inner)):
        vertices.append((points_inner[i][0], points_inner[i][1], points_inner[i][2] + offset_z))
        vertices.append((points_outer[i][0], points_outer[i][1], points_outer[i][2] + offset_z))
    
    mesh.GetPointsAttr().Set(vertices)
    
    face_indices = []
    face_counts = []
    for i in range(len(points_inner) - 1):
        if skip_indices and i in skip_indices:
            continue
        idx = i * 2
        # Create a quad for each segment
        face_indices.extend([idx, idx + 1, idx + 3, idx + 2])
        face_counts.append(4)
        
    mesh.GetFaceVertexIndicesAttr().Set(face_indices)
    mesh.GetFaceVertexCountsAttr().Set(face_counts)
    
    # Physics and Material
    apply_material(mesh.GetPrim(), stage.GetPrimAtPath(material_path))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    if "Asphalt" in material_path:
        # For the road surface, we need accurate triangle collision
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        mesh_collision.CreateApproximationAttr().Set("none")

def generate_track_path(track_width, track_height, corner_radius, resolution=20):
    """
    Generates a list of centerline points for a rounded rectangle.
    """
    half_w = (track_width - 2 * corner_radius) / 2
    half_h = (track_height - 2 * corner_radius) / 2
    
    path = []
    
    # 1. South Straight (East to West)
    for x in np.linspace(half_w, -half_w, resolution):
        path.append((x, -track_height/2, 0.05))
        
    # 2. SW Corner (270 to 180 deg)
    for a in np.linspace(270, 180, resolution):
        rad = math.radians(a)
        path.append((-half_w + corner_radius * math.cos(rad), -half_h + corner_radius * math.sin(rad), 0.05))
        
    # 3. West Straight (South to North)
    for y in np.linspace(-half_h, half_h, resolution):
        path.append((-track_width/2, y, 0.05))
        
    # 4. NW Corner (180 to 90 deg)
    for a in np.linspace(180, 90, resolution):
        rad = math.radians(a)
        path.append((-half_w + corner_radius * math.cos(rad), half_h + corner_radius * math.sin(rad), 0.05))
        
    # 5. North Straight (West to East)
    for x in np.linspace(-half_w, half_w, resolution):
        path.append((x, track_height/2, 0.05))
        
    # 6. NE Corner (90 to 0 deg)
    for a in np.linspace(90, 0, resolution):
        rad = math.radians(a)
        path.append((half_w + corner_radius * math.cos(rad), half_h + corner_radius * math.sin(rad), 0.05))
        
    # 7. East Straight (North to South)
    for y in np.linspace(half_h, -half_h, resolution):
        path.append((track_width/2, y, 0.05))
        
    # 8. SE Corner (0 to -90 deg)
    for a in np.linspace(0, -90, resolution):
        rad = math.radians(a)
        path.append((half_w + corner_radius * math.cos(rad), -half_h + corner_radius * math.sin(rad), 0.05))
        
    # Close the loop
    path.append(path[0])
    return path

def create_road_section(stage, name, centerline, material_paths, offset_z_base=0.0, intersection_points=None):
    """
    Generates asphalt and line meshes for a given centerline path.
    """
    points_asphalt_inner = []
    points_asphalt_outer = []
    points_line_left_in = []
    points_line_left_out = []
    points_line_right_in = []
    points_line_right_out = []
    points_yellow_in = []
    points_yellow_out = []

    skip_segments = []
    gap_radius = ROAD_WIDTH * 0.75

    for i in range(len(centerline)):
        curr = np.array(centerline[i])
        
        # Check for intersection gap
        if intersection_points:
            for ip in intersection_points:
                if np.linalg.norm(curr - np.array(ip)) < gap_radius:
                    skip_segments.append(i)
                    break

        # Central difference for smooth tangents
        prev_p = np.array(centerline[(i - 1) % len(centerline)])
        next_p = np.array(centerline[(i + 1) % len(centerline)])
        
        # For non-loops, fix ends
        if i == 0:
            tangent = np.array(centerline[1]) - curr
        elif i == len(centerline) - 1:
            tangent = curr - np.array(centerline[-2])
        else:
            tangent = next_p - prev_p
            
        if np.linalg.norm(tangent) < 1e-6:
            tangent = np.array([1, 0, 0])
            
        normal = np.array([-tangent[1], tangent[0], 0])
        normal = normal / np.linalg.norm(normal)
        
        # Asphalt
        points_asphalt_inner.append(curr - normal * (ROAD_WIDTH/2))
        points_asphalt_outer.append(curr + normal * (ROAD_WIDTH/2))
        
        # White Lines
        l_outer = ROAD_WIDTH/2 - 0.05
        l_inner = l_outer - LINE_WIDTH
        points_line_left_in.append(curr + normal * l_inner)
        points_line_left_out.append(curr + normal * l_outer)
        points_line_right_in.append(curr - normal * l_outer)
        points_line_right_out.append(curr - normal * l_inner)
        
        # Yellow Center Line
        points_yellow_in.append(curr - normal * (LINE_WIDTH/2))
        points_yellow_out.append(curr + normal * (LINE_WIDTH/2))

    scope_path = f"/World/Roads/{name}"
    prim_utils.create_prim(scope_path, "Scope")
    
    create_road_mesh(stage, f"{name}/Asphalt", points_asphalt_inner, points_asphalt_outer, material_paths["Asphalt"], offset_z=offset_z_base)
    create_road_mesh(stage, f"{name}/WhiteLineLeft", points_line_left_in, points_line_left_out, material_paths["White"], offset_z=offset_z_base + 0.01, skip_indices=skip_segments)
    create_road_mesh(stage, f"{name}/WhiteLineRight", points_line_right_in, points_line_right_out, material_paths["White"], offset_z=offset_z_base + 0.01, skip_indices=skip_segments)
    create_road_mesh(stage, f"{name}/YellowLine", points_yellow_in, points_yellow_out, material_paths["Yellow"], offset_z=offset_z_base + 0.01, skip_indices=skip_segments)

# --- Main Execution ---

def main():
    world = World()
    world.scene.add_default_ground_plane() 
    stage = world.stage

    # 1. Materials
    path_mat = "/World/Materials"
    prim_utils.create_prim(path_mat, "Scope")
    mats = {
        "Asphalt": f"{path_mat}/Asphalt",
        "White": f"{path_mat}/White",
        "Yellow": f"{path_mat}/Yellow",
        "Red": f"{path_mat}/Red",
        "Metal": f"{path_mat}/Metal"
    }
    create_material(stage, mats["Asphalt"], (0.1, 0.1, 0.1))
    create_material(stage, mats["White"], (0.9, 0.9, 0.9))
    create_material(stage, mats["Yellow"], (0.8, 0.7, 0.1))
    create_material(stage, mats["Red"], (0.8, 0.1, 0.1))
    create_material(stage, mats["Metal"], (0.4, 0.4, 0.4))

    # 2. Track Layout
    prim_utils.create_prim("/World/Roads", "Scope")
    
    intersections = [(15.0, 0, 0.05), (-15.0, 0, 0.05)]

    # Loop Track (30x20)
    loop_centerline = generate_track_path(30.0, 20.0, 5.0, resolution=60)
    create_road_section(stage, "MainLoop", loop_centerline, mats, intersection_points=intersections)
    
    # Crossing Road
    cross_centerline = []
    for x in np.linspace(-25, 25, 60):
        cross_centerline.append((x, 0, 0.05))
    create_road_section(stage, "CrossRoad", cross_centerline, mats, offset_z_base=0.001, intersection_points=intersections)

    # 3. Add Props
    prim_utils.create_prim("/World/Props", "Scope")
    create_stop_sign(stage, "StopSign1", (15.0, 3.0, 0), 0)
    create_stop_sign(stage, "StopSign2", (-15.0, -3.0, 0), 180)

    # 4. Finalize
    prim_utils.create_prim("/World/Light", "DistantLight", position=(0, 0, 10), orientation=euler_angles_to_quat(np.array([45, 45, 0]), degrees=True))
    
    for _ in range(10):
        simulation_app.update()

    if stage.GetPrimAtPath("/World"):
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    output_path = os.path.join(os.getcwd(), "jetracer_track.usda")
    stage.GetRootLayer().Export(output_path)
    print(f"{output_path}")
    
    simulation_app.close()

if __name__ == "__main__":
    main()
