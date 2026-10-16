# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""USD Debug Sphere Markers Visualization for Track Boundaries and Centerline."""

import os
import numpy as np
import torch


def refresh_track_visuals(tm, target_lane_offset: float = 0.0) -> None:
    """Updates Isaac Sim debug visualization markers on the USD stage."""
    try:
        if tm.raw_yellow_pts is None or tm.waypoints is None:
            tm.load_cache()
            if os.path.exists(tm.wp_path) and tm.waypoints is None:
                wp_np = np.load(tm.wp_path)
                tm.waypoints = torch.tensor(
                    wp_np, device=tm.device, dtype=torch.float32
                )

        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
        import isaaclab.sim as sim_utils
        from pxr import UsdGeom, Gf, Usd
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
        origin = (
            UsdGeom.Xformable(env0_prim)
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            .ExtractTranslation()
            if env0_prim.IsValid()
            else Gf.Vec3d(0, 0, 0)
        )

        def create_v(path, color, radius=0.05):
            cfg = VisualizationMarkersCfg(
                prim_path=path,
                markers={
                    "dot": sim_utils.SphereCfg(
                        radius=radius,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=color
                        ),
                    )
                },
            )
            return VisualizationMarkers(cfg)

        def create_gate_v(path, color, radius=0.20):
            cfg = VisualizationMarkersCfg(
                prim_path=path,
                markers={
                    "dot": sim_utils.SphereCfg(
                        radius=radius,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=color
                        ),
                    )
                },
            )
            return VisualizationMarkers(cfg)

        def to_w(pts, z_off=0.05):
            if pts is None or len(pts) == 0:
                return None
            if isinstance(pts, np.ndarray):
                pts_t = torch.from_numpy(pts).to(
                    device=tm.device, dtype=torch.float32
                )
            else:
                pts_t = pts.to(device=tm.device, dtype=torch.float32)

            n_pts = pts_t.shape[0]
            w = torch.zeros((n_pts, 3), device=tm.device, dtype=torch.float32)
            w[:, 0] = pts_t[:, 0] + origin[0]
            w[:, 1] = pts_t[:, 1] + origin[1]
            if pts_t.shape[1] > 2:
                w[:, 2] = pts_t[:, 2] + origin[2] + z_off
            else:
                w[:, 2] = origin[2] + z_off
            return w

        if tm.visualizer_yellow is None:
            tm.visualizer_yellow = create_v(
                "/World/Visuals/YellowSpheres", (1.0, 1.0, 0.0), radius=0.05
            )
        if tm.visualizer_white is None:
            tm.visualizer_white = create_v(
                "/World/Visuals/WhiteSpheres", (1.0, 1.0, 1.0), radius=0.05
            )
        if tm.visualizer_gate is None:
            tm.visualizer_gate = create_gate_v(
                "/World/Visuals/GateSpheres", (0.0, 1.0, 0.0), radius=0.20
            )
        if tm.visualizer_waypoints is None:
            tm.visualizer_waypoints = create_v(
                "/World/Visuals/CyanPath", (0.0, 1.0, 1.0), radius=0.06
            )
        if tm.visualizer_target is None:
            tm.visualizer_target = create_v(
                "/World/Visuals/TargetLane", (1.0, 0.0, 1.0), radius=0.06
            )

        # 1. Yellow & White Boundaries
        if tm.raw_yellow_pts is not None:
            pts_y = to_w(tm.raw_yellow_pts[::16], z_off=0.05)
            if pts_y is not None:
                tm.visualizer_yellow.visualize(pts_y)

        if tm.raw_white_pts is not None:
            pts_w = to_w(tm.raw_white_pts[::16], z_off=0.05)
            if pts_w is not None:
                tm.visualizer_white.visualize(pts_w)

        # 2. Gate Markers (Green)
        if tm.raw_gate_pts is not None:
            pts_g = to_w(tm.raw_gate_pts[::4], z_off=0.25)
            if pts_g is not None:
                tm.visualizer_gate.visualize(pts_g)

        # 3. Centerline (Cyan) & Target Lane Offset (Magenta)
        if tm.waypoints is not None:
            pts_wp = to_w(tm.waypoints[::10, :2], z_off=0.08)
            if pts_wp is not None:
                tm.visualizer_waypoints.visualize(pts_wp)

            wp_np = tm.waypoints.cpu().numpy()
            yaws = wp_np[:, 2]
            normal_x = np.sin(yaws)
            normal_y = -np.cos(yaws)
            target_pts = wp_np[:, :2] + target_lane_offset * np.stack(
                [normal_x, normal_y], axis=1
            )
            pts_target = to_w(target_pts[::10], z_off=0.12)
            if pts_target is not None:
                tm.visualizer_target.visualize(pts_target)
    except Exception as e:
        print(f"[TrackManager] refresh_visuals error: {e}")
