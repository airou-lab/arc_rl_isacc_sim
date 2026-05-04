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
    """Custom spawner for F1Tenth that applies per-link mass overrides."""
    # 1. Spawn using default spawner
    prim = sim_utils.spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    stage = get_current_stage()
    
    if mass_overrides:
        # Apply overrides to children
        for child in prim.GetChildren():
            child_name = child.GetName()
            # print(f"[Spawner] Checking child: {child_name}")
            for pattern, mass in mass_overrides.items():
                if re.match(pattern, child_name):
                    # Apply mass override
                    mass_cfg = sim_utils.MassPropertiesCfg(mass=mass)
                    schemas.define_mass_properties(str(child.GetPath()), mass_cfg, stage)
                    
                    # Lower Center of Mass to make the robot bottom-heavy and prevent flipping
                    from pxr import UsdPhysics, Gf
                    mass_api = UsdPhysics.MassAPI.Apply(child)
                    # Move CoM down by 5cm relative to the prim origin
                    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, -0.05))
                    
                    # print(f"[Spawner] SUCCESS: Applied mass {mass} and lowered CoM for {child.GetPath()}")
                    break
                    
    return prim
