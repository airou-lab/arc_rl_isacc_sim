# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""USD Mesh Extraction and Centerline Generation Utilities for Track Boundaries."""

from typing import Dict, List, Optional, Tuple

import numpy as np


def get_mesh_info(prim, xform_cache, env0_origin) -> Dict:
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    points_attr = mesh.GetPointsAttr().Get()
    indices_attr = mesh.GetFaceVertexIndicesAttr().Get()
    counts_attr = mesh.GetFaceVertexCountsAttr().Get()
    if not points_attr or not indices_attr:
        raise ValueError(f"Mesh at {prim.GetPath()} is missing points or indices.")

    xform = xform_cache.GetLocalToWorldTransform(prim)
    transformed_pts = []
    for p in points_attr:
        pw = xform.Transform(p)
        transformed_pts.append(
            [pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]]
        )
    return {"points": transformed_pts, "indices": indices_attr, "counts": counts_attr}


def finalize_group(data_list: List[Tuple[str, Dict]], name: str) -> np.ndarray:
    if not data_list:
        raise ValueError(f"No valid meshes found for {name}.")
    all_dense_pts = []
    resolution = 0.01  # High resolution (1cm) for 1.0x scale
    for path, mesh_data in data_list:
        pts = np.array(mesh_data["points"])
        indices = mesh_data["indices"]
        counts = mesh_data["counts"]
        curr_idx = 0
        for count in counts:
            face_indices = indices[curr_idx : curr_idx + count]
            curr_idx += count
            for i in range(count):
                p1, p2 = pts[face_indices[i]], pts[face_indices[(i + 1) % count]]
                dist = np.linalg.norm(p2 - p1)
                if dist > 0:
                    num_steps = max(1, int(dist / resolution))
                    for step in range(num_steps):
                        all_dense_pts.append(p1 + (p2 - p1) * (step / num_steps))
    if not all_dense_pts:
        raise ValueError(f"Failed to sample any dense points for {name}.")
    final_pts = np.unique(np.round(np.array(all_dense_pts), 3), axis=0)
    print(f"[TrackManager] Finalized {name}: {len(final_pts)} points.")
    return final_pts


def punch_holes(
    raw_pts: Optional[np.ndarray], gate_xy: np.ndarray, name: str
) -> np.ndarray:
    if raw_pts is None or len(raw_pts) == 0:
        raise ValueError(
            f"Cannot punch holes in {name}: points array is empty or None."
        )
    from scipy.spatial import KDTree

    tree = KDTree(gate_xy)
    dists, _ = tree.query(raw_pts[:, :2], k=1)

    mask = dists > 1.0
    filtered = raw_pts[mask]
    print(
        f"[TrackManager] Punched holes in {name}: removed {len(raw_pts) - len(filtered)} overlapping points."
    )
    return filtered


def sample_waypoints(y_pts: np.ndarray, w_pts: np.ndarray) -> np.ndarray:
    from scipy.spatial import KDTree

    tree = KDTree(w_pts)
    dists, idxs = tree.query(y_pts)

    midpoints = (y_pts + w_pts[idxs]) / 2.0
    valid_mask = dists < 4.0
    midpoints = midpoints[valid_mask]

    dists_to_origin = np.linalg.norm(midpoints, axis=1)
    curr = np.argmin(dists_to_origin)

    tree_nn = KDTree(midpoints)
    visited = np.zeros(len(midpoints), dtype=bool)
    visited[curr] = True
    ordered = [midpoints[curr]]

    for _ in range(len(midpoints) - 1):
        curr_pt = ordered[-1]
        dists, idxs = tree_nn.query(curr_pt, k=50)

        found = False
        for idx in np.atleast_1d(idxs):
            if not visited[idx]:
                visited[idx] = True
                ordered.append(midpoints[idx])
                found = True
                break

        if not found:
            unvisited_idxs = np.where(~visited)[0]
            if len(unvisited_idxs) == 0:
                break
            unvisited_pts = midpoints[unvisited_idxs]
            dists_glob = np.linalg.norm(unvisited_pts - curr_pt, axis=1)
            best_idx = unvisited_idxs[np.argmin(dists_glob)]
            visited[best_idx] = True
            ordered.append(midpoints[best_idx])

    ordered = np.array(ordered)
    n = len(ordered)
    yaws = np.zeros(n)
    for i in range(n):
        i_next = (i + 1) % n
        dy = ordered[i_next, 1] - ordered[i, 1]
        dx = ordered[i_next, 0] - ordered[i, 0]
        yaws[i] = np.arctan2(dy, dx)

    final_wps = np.zeros((n, 3))
    final_wps[:, :2] = ordered
    final_wps[:, 2] = yaws
    return final_wps


def collect_raw_marker_points_from_stage() -> Tuple[
    np.ndarray, np.ndarray, Optional[np.ndarray]
]:
    """Traverses USD stage env_0 to extract yellow lines, white lines, and gates."""
    import omni.usd
    from pxr import Usd, UsdGeom, UsdShade

    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath("/World/envs/env_0")
    if not root_prim.IsValid():
        root_prim = stage.GetPseudoRoot()
    xform_cache = UsdGeom.XformCache()
    env0_origin = xform_cache.GetLocalToWorldTransform(root_prim).ExtractTranslation()

    y_data, w_data, g_data = [], [], []

    # 1. First, find all Gates explicitly
    gate_prim_paths = []
    gate_fallback_pts = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        path_str = str(prim.GetPath())
        is_gate_prim = False

        if prim.HasAttribute("primvars:ds_type"):
            attr_val = prim.GetAttribute("primvars:ds_type").Get()
            if attr_val == ["DSLaneGate"] or attr_val == "DSLaneGate":
                is_gate_prim = True

        if not is_gate_prim and "laneGate" in path_str:
            is_gate_prim = True

        if is_gate_prim:
            gate_prim_paths.append(path_str)
            xform = UsdGeom.Xformable(prim)
            world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            pw = world_transform.ExtractTranslation()
            pl = [
                pw[0] - env0_origin[0],
                pw[1] - env0_origin[1],
                pw[2] - env0_origin[2],
            ]
            gate_fallback_pts.append(pl)

    if len(gate_prim_paths) > 0:
        print(
            f"[TrackManager] Found {len(gate_prim_paths)} gate prims. Example: {gate_prim_paths[0]}"
        )

    # Add the right-turn intersection manually since it lacks a gate prim
    gate_fallback_pts.append([-15.95, 6.19, 0.0])
    print(
        f"[TrackManager] Added manual right-turn intersection fallback. Total fallbacks: {len(gate_fallback_pts)}"
    )

    # 2. Traverse meshes in env_0 and categorize
    for prim in Usd.PrimRange(root_prim):
        if "Robot" in str(prim.GetPath()):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue

        path_str = str(prim.GetPath())
        is_gate = (
            "laneGate" in path_str
            or "lane_gate" in path_str
            or "stop_line" in path_str.lower()
        )
        if not is_gate:
            for g_path in gate_prim_paths:
                if path_str.startswith(g_path):
                    is_gate = True
                    break

        mesh_info = get_mesh_info(prim, xform_cache, env0_origin)
        if not mesh_info or len(mesh_info["points"]) == 0:
            continue

        binding_api = UsdShade.MaterialBindingAPI(prim)
        mat, _ = binding_api.ComputeBoundMaterial()
        mat_path = str(mat.GetPath() if mat else "").lower()
        search_str = (path_str + mat_path).lower()

        is_near_gate_marker = False
        if len(gate_fallback_pts) > 0:
            mesh_center = np.mean(mesh_info["points"], axis=0)
            for g_pos in gate_fallback_pts:
                if np.linalg.norm(mesh_center - np.array(g_pos)) < 1.0:
                    is_near_gate_marker = True
                    break

        if (
            "lanegate" in path_str.lower()
            or "lane_gate" in path_str.lower()
            or "stop_line" in path_str.lower()
        ):
            is_gate = True
        elif "white" in search_str and is_near_gate_marker:
            is_gate = True
        else:
            is_gate = False

        if is_gate:
            g_data.append((path_str, mesh_info))
            continue

        is_marking = (
            "roadmarks" in path_str
            or "paint" in path_str
            or "marking" in path_str.lower()
        )

        if is_marking:
            if "yellow" in search_str:
                y_data.append((path_str, mesh_info))
            elif "white" in search_str:
                w_data.append((path_str, mesh_info))

    raw_yellow_pts = finalize_group(y_data, "Yellow")
    raw_white_pts = finalize_group(w_data, "White")

    raw_gate_pts = finalize_group(g_data, "Gate")
    if raw_gate_pts is None and len(gate_fallback_pts) > 0:
        print("[TrackManager] No gate meshes found. Using prim centers as fallback.")
        raw_gate_pts = np.array(gate_fallback_pts)

    if raw_gate_pts is not None:
        gate_xy = raw_gate_pts[:, :2]
        raw_yellow_pts = punch_holes(raw_yellow_pts, gate_xy, "Yellow")
        raw_white_pts = punch_holes(raw_white_pts, gate_xy, "White")

    return raw_yellow_pts, raw_white_pts, raw_gate_pts
