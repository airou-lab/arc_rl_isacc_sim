# Research: USD Asset Flattening (14-01)

## Goal
Flatten the USD track map geometry to natively be 1.0x metric scale instead of relying on a `scale=(0.125, 0.125, 0.125)` hack in `arcpro_env_cfg.py`.

## Codebase Analysis
- **Current State in `arcpro_env_cfg.py`**:
  ```python
  track = AssetBaseCfg(
      prim_path="{ENV_REGEX_NS}/Track",
      spawn=sim_utils.UsdFileCfg(
          usd_path=os.path.join(USD_DIR, "no_graph_sim_clean_1x.usda"),
          scale=(0.125, 0.125, 0.125), # Shrink world to match 1.0x robot
      ), ...
  ```
- **Current State in `no_graph_sim_clean_1x.usda`**:
  The root prim `/World` currently has an `xformOp:scale = (0.125, 0.125, 0.125)` applied.

## Technical Steps
1. Write a Python script to open `no_graph_sim_clean_1x.usda` using the `pxr.Usd` API.
2. Traverse all `UsdGeom.Mesh` prims in the stage. For each mesh, multiply its points by 0.125 to shrink the geometry physically (if they are at 8x scale).
3. Update the root `/World` prim (and any others as needed) using `pxr.UsdGeom.XformCommonAPI(prim).SetScale((1.0, 1.0, 1.0))` to remove the 0.125 scale factor from the node transforms, as the geometry itself is now scaled. 
   *(Alternatively, Isaac Sim/Omniverse might offer an "Apply Transforms" utility that does this automatically without manual vertex math)*.
4. Save the modified stage as `openStreetUSD/no_graph_sim_clean_true_1x.usda`.
5. Update `arcproLab/arcpro_env_cfg.py` to:
   - Point to `no_graph_sim_clean_true_1x.usda`.
   - Remove the `scale=(0.125, 0.125, 0.125)` override from `sim_utils.UsdFileCfg` and replace with `scale=(1.0, 1.0, 1.0)`.

## Edge Cases and Risks
- **Mesh Points vs Curves/Cameras**: If there are non-mesh geometries (like `UsdGeom.BasisCurves` or `UsdGeom.Camera` offsets), their points or translations also need to be scaled down by 0.125.
- **Physics Colliders**: If collision meshes are distinct from visual meshes, their geometries must also be traversed and scaled.
- **Sub-transforms**: There might be nested `Xform` prims that also carry translations. If we multiply vertices by 0.125, the local translations of nested Xforms must also be scaled by 0.125.
- **Simpler approach**: Often it's safer to use omniverse tools or load the USD into a DCC tool like Maya/Blender/Isaac Sim GUI, freeze transformations (flatten), and re-export.
