import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Audit USD assets for Phase 5.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
from pxr import Usd, UsdGeom, Gf

def audit_track(path):
    print(f"\n--- Auditing Track: {path} ---")
    stage = Usd.Stage.Open(path)
    if not stage:
        print(f"FAILED to open {path}")
        return

    # Check for unresolved references
    print("Checking for unresolved references...")
    for prim in stage.Traverse():
        if prim.HasAuthoredReferences():
            for ref in prim.GetMetadata("references").prependedItems:
                resolved_path = stage.ResolveIdentifierToEditTarget(ref.assetPath)
                if not resolved_path:
                    print(f"  [!] Broken Reference on {prim.GetPath()}: {ref.assetPath}")

    # Check grounding of road surface
    print("Checking road grounding...")
    meshes = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(prim)
            area = (bbox.GetRange().GetMax()[0] - bbox.GetRange().GetMin()[0]) * (bbox.GetRange().GetMax()[1] - bbox.GetRange().GetMin()[1])
            meshes.append((prim, area, bbox.GetRange()))
    
    if meshes:
        # Sort by area descending
        meshes.sort(key=lambda x: x[1], reverse=True)
        road_prim, road_area, road_range = meshes[0]
        print(f"  Detected Road Prim (Largest): {road_prim.GetPath()}")
        print(f"  Z-Range (Raw): {road_range.GetMin()[2]:.3f} to {road_range.GetMax()[2]:.3f}")
    else:
        print("  [!] Could not find any mesh prims.")

def audit_robot(path):
    print(f"\n--- Auditing Robot: {path} ---")
    stage = Usd.Stage.Open(path)
    if not stage:
        print(f"FAILED to open {path}")
        return

    # Measure ChassisSpecifically
    chassis_prim = stage.GetPrimAtPath("/Robot/Chassis")
    if not chassis_prim.IsValid():
         chassis_prim = stage.GetDefaultPrim()

    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(chassis_prim)
    range = bbox.GetRange()
    size = range.GetMax() - range.GetMin()
    print(f"  Prim Measured: {chassis_prim.GetPath()}")
    print(f"  Dimensions: L={size[0]:.3f}m, W={size[1]:.3f}m, H={size[2]:.3f}m")
    
    # Standard F1Tenth is ~33cm long
    if 0.3 < size[0] < 0.4:
        print("  [OK] Scale is 1.0x Metric (~33cm).")
    else:
        print(f"  [!] Scale seems off for a 1.0x metric car: {size[0]:.3f}m")

    # Check joints
    print("Checking Articulation Joints...")
    for prim in stage.Traverse():
        if "Joint" in prim.GetName():
            print(f"  Joint: {prim.GetPath()} ({prim.GetTypeName()})")

print("Starting Asset Audit...")
audit_track("openStreetUSD/no_graph_sim_final.usd")
audit_track("openStreetUSD/arcpro_RL_open_street_sim.usd")
audit_track("openStreetUSD/original_usd.usd")
audit_robot("arcproLab/assets/robot/F1Tenth_Metric.usd")

simulation_app.close()
