# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from pxr import Usd
import isaaclab.sim as sim_utils
import isaaclab.sim.schemas as schemas
from isaaclab.sim.utils import clone, get_current_stage
import re
from pxr import Usd, UsdGeom

@clone
def spawn_guide_cone(
    prim_path: str,
    cfg: sim_utils.ConeCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawns a cone and sets its purpose to 'guide' so it's invisible to sensors."""
    prim = sim_utils.spawn_cone(prim_path, cfg, translation, orientation, **kwargs)
    # Set purpose to guide (Invisible to standard RenderProducts/Cameras)
    geom_prim = UsdGeom.Imageable(prim)
    geom_prim.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    return prim

@clone
def spawn_f1tenth(
    prim_path: str,
    cfg: sim_utils.UsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    mass_overrides: dict[str, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Custom spawner for F1Tenth that applies per-link mass overrides and RC car tire friction."""
    # 1. Spawn using default spawner
    prim = sim_utils.spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    stage = get_current_stage()
    
    if mass_overrides:
        # Apply overrides recursively to all descendants
        for child in Usd.PrimRange(prim):
            child_name = child.GetName()
            # print(f"[Spawner] Checking child: {child_name}")
            for pattern, mass in mass_overrides.items():
                if re.match(pattern, child_name):
                    # Apply mass override
                    mass_cfg = sim_utils.MassPropertiesCfg(mass=mass)
                    schemas.define_mass_properties(str(child.GetPath()), mass_cfg, stage)
                    
                    # Lower Center of Mass slightly to prevent flipping without clipping
                    from pxr import UsdPhysics, Gf
                    mass_api = UsdPhysics.MassAPI.Apply(child)
                    if child_name == "Chassis":
                        # Move CoM down by 1cm (0.01m) relative to the prim origin only for the Chassis
                        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, -0.01))
                    
                    # print(f"[Spawner] SUCCESS: Applied mass {mass} and lowered CoM for {child.GetPath()}")
                    break

    # 2. RC Car Tire Friction — applied to all Wheel_* prims
    # Rubber-on-asphalt: high grip, near-zero bounce.
    # static_friction  = 1.2  → resists initial slip (planted launches)
    # dynamic_friction = 0.8  → rolling grip during cornering
    # restitution      = 0.0  → no bounce on road contact
    _apply_tire_friction(prim, stage)
                    
    return prim


def _apply_tire_friction(robot_prim: Usd.Prim, stage) -> None:
    """Apply rubber-on-asphalt PhysX material to all Wheel_* child prims."""
    from pxr import UsdPhysics, UsdShade, Gf, Sdf

    STATIC_FRICTION  = 3.0
    DYNAMIC_FRICTION = 3.0
    RESTITUTION      = 0.0

    robot_path = str(robot_prim.GetPath())

    for child in Usd.PrimRange(robot_prim):
        child_name = child.GetName()
        if not re.match(r"Wheel_.*", child_name):
            continue

        child_path = str(child.GetPath())
        mat_path   = f"{child_path}/TireMaterial"

        # Create or reuse material prim
        mat_prim = stage.GetPrimAtPath(mat_path)
        if not mat_prim.IsValid():
            mat_prim = stage.DefinePrim(mat_path, "Material")

        # Bind PhysicsMaterialAPI
        phys_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
        phys_mat.CreateStaticFrictionAttr().Set(STATIC_FRICTION)
        phys_mat.CreateDynamicFrictionAttr().Set(DYNAMIC_FRICTION)
        phys_mat.CreateRestitutionAttr().Set(RESTITUTION)

        # Bind the material to the wheel prim
        mat_obj = UsdShade.Material(mat_prim)
        mat_binding = UsdShade.MaterialBindingAPI.Apply(child)
        mat_binding.Bind(mat_obj, UsdShade.Tokens.strongerThanDescendants, "physics")
        
        # Explicitly bind to Geom children just in case
        for geom_child in child.GetChildren():
            if "Geom" in geom_child.GetName() or geom_child.HasAPI(UsdPhysics.CollisionAPI):
                geom_binding = UsdShade.MaterialBindingAPI.Apply(geom_child)
                geom_binding.Bind(mat_obj, UsdShade.Tokens.strongerThanDescendants, "physics")
