# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Manages track waypoints and provides utilities for calculating lateral and heading errors.
    Supports procedural generation from USD road markers.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        self.visualizer = None
        self.left_visualizer = None
        self.right_visualizer = None
        
        # Raw marker points for direct visualization
        self.raw_yellow_pts = None
        self.raw_white_pts = None
        
        self.density = 0.5 # Default density
        self.usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/no_graph_sim_clean_1x.usda"
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline_1x.npy")

        # PRIORITIZE PROCEDURAL SAMPLING
        print(f"[TrackManager] Initializing procedural waypoint generation from road markers...")
        try:
            self.sample_waypoints_from_usd(density=self.density)
        except Exception as e:
            print(f"[TrackManager] Procedural sampling failed ({e}), falling back to file...")
            if os.path.exists(self.wp_path):
                self.load_waypoints(self.wp_path)
            else:
                print(f"[TrackManager] CRITICAL: No waypoint source found.")

    def load_waypoints(self, path: str):
        """Loads waypoints from a .npy file."""
        data = np.load(path)
        
        # 1. REVERSE ORDER: Ensure sequence moves South.
        data = data[::-1]
        
        # 2. FLIP HEADING: Ensure yaw faces South (-1.57).
        data[:, 2] += np.pi
        data[:, 2] = np.arctan2(np.sin(data[:, 2]), np.cos(data[:, 2]))
        
        # 3. CENTER ON YELLOW: The raw .npy is already the yellow line.
        
        # Use .copy() to fix negative strides
        self.waypoints = torch.tensor(data.copy(), device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Loaded {len(self.waypoints)} waypoints (REVERSED, FLIPPED SOUTH, YELLOW CENTERLINE)")
        self.refresh_visuals()

    def refresh_visuals(self):
        """Initializes or updates the waypoint and boundary markers in the GUI."""
        if self.waypoints is None:
            return

        try:
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            import isaaclab.sim as sim_utils
            
            # 1. YELLOW LINE (Center/Divider)
            if self.left_visualizer is None and self.raw_yellow_pts is not None:
                left_limit_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/RawYellowMarkers",
                    markers={
                        "dot": sim_utils.SphereCfg(
                            radius=0.15, # Enlarged
                            collision_enabled=False,
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0))
                        )
                    }
                )
                self.left_visualizer = VisualizationMarkers(left_limit_cfg)
            
            # 2. WHITE LINE (Outer Edge)
            if self.right_visualizer is None and self.raw_white_pts is not None:
                right_limit_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/RawWhiteMarkers",
                    markers={
                        "dot": sim_utils.SphereCfg(
                            radius=0.15, # Enlarged
                            collision_enabled=False,
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0))
                        )
                    }
                )
                self.right_visualizer = VisualizationMarkers(right_limit_cfg)

            # 3. WAYPOINT PATH (Computed Centerline - Cyan)
            if self.visualizer is None:
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/TrackWaypoints",
                    markers={
                        "dot": sim_utils.SphereCfg(
                            radius=0.15, # Enlarged
                            collision_enabled=False,
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0))
                        )
                    }
                )
                self.visualizer = VisualizationMarkers(marker_cfg)

            # Draw Yellow Markers
            if self.left_visualizer and self.raw_yellow_pts is not None:
                vis_y = torch.tensor(self.raw_yellow_pts, device=self.device, dtype=torch.float32)
                vis_y_3d = torch.zeros((len(vis_y), 3), device=self.device)
                vis_y_3d[:, :2] = vis_y
                vis_y_3d[:, 2] = 0.1 # Raised high for visibility
                self.left_visualizer.visualize(vis_y_3d)

            # Draw White Markers
            if self.right_visualizer and self.raw_white_pts is not None:
                vis_w = torch.tensor(self.raw_white_pts, device=self.device, dtype=torch.float32)
                vis_w_3d = torch.zeros((len(vis_w), 3), device=self.device)
                vis_w_3d[:, :2] = vis_w
                vis_w_3d[:, 2] = 0.1 # Raised high for visibility
                self.right_visualizer.visualize(vis_w_3d)

            # Draw Centerline
            vis_c = self.waypoints[::5, :3].clone()
            vis_c[:, 2] = 0.1 # Raised high for visibility
            self.visualizer.visualize(vis_c)

            print(f"[TrackManager] Visualized markers at size 0.15 and height 0.1.")
        except Exception as e:
            print(f"[TrackManager] Visualization error: {e}")

    def save_waypoints(self, path: str):
        """Saves waypoints to a .npy file."""
        if self.waypoints is not None:
            data = self.waypoints.cpu().numpy()
            np.save(path, data)
            print(f"[TrackManager] Saved {len(self.waypoints)} waypoints to {path}")

    def sample_waypoints_from_usd(self, density: float = 0.5):
        """Procedurally samples road markers and computes a centerline."""
        import omni.usd
        from pxr import Usd, UsdGeom, UsdShade, Gf
        
        stage = omni.usd.get_context().get_stage()
        if stage is None: return

        yellow_pts = []
        white_pts = []
        
        env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
        env0_origin = Gf.Vec3d(0, 0, 0)
        if env0_prim.IsValid():
            env0_xform = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            env0_origin = env0_xform.ExtractTranslation()

        # 1. Collect all points from roadmarks anywhere in the stage
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if not prim.IsA(UsdGeom.Mesh): continue
            path = str(prim.GetPath())
            
            # Use name-based fallback if material check is unreliable
            mat_path = ""
            binding_api = UsdShade.MaterialBindingAPI(prim)
            material, _ = binding_api.ComputeBoundMaterial()
            if material:
                mat_path = str(material.GetPath()).lower()
            
            # Combine name and material check
            search_str = (path + mat_path).lower()
            if "roadmarks" not in search_str and "piece" not in search_str: continue
                
            points_attr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not points_attr: continue
            xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            for p in points_attr:
                p_world = xform.Transform(p)
                # Ensure we are only looking at objects near the environment origins
                p_local = [p_world[0] - env0_origin[0], p_world[1] - env0_origin[1]]
                
                if "yellow" in search_str: yellow_pts.append(p_local)
                elif "white" in search_str: white_pts.append(p_local)

        if not yellow_pts: 
            print("[TrackManager] No yellow marker points found.")
            return
        
        self.raw_yellow_pts = np.unique(np.round(np.array(yellow_pts), 2), axis=0)
        self.raw_white_pts = np.unique(np.round(np.array(white_pts), 2), axis=0) if white_pts else None

        # 2. Generate a simple ordered path for the current segment
        pts = self.raw_yellow_pts
        spawn_target = np.array([-16.25, 5.56])
        
        ordered_pts = []
        start_dists = np.sum((pts[:, :2] - spawn_target)**2, axis=1)
        curr_idx = np.argmin(start_dists)
        
        visited = set([curr_idx])
        ordered_pts.append(pts[curr_idx])
        
        while len(visited) < len(pts):
            last_pt = ordered_pts[-1]
            dists = np.sum((pts - last_pt)**2, axis=1)
            for v in visited: dists[v] = np.inf
            next_idx = np.argmin(dists)
            if dists[next_idx] > 2.0**2: break # End of segment
            ordered_pts.append(pts[next_idx])
            visited.add(next_idx)
            
        ordered_pts = np.array(ordered_pts)
        
        # 3. Compute Yaw and Centerline (Directly on Yellow)
        final_wps = []
        for i in range(len(ordered_pts)):
            pt = ordered_pts[i]
            next_pt = ordered_pts[i+1] if i < len(ordered_pts)-1 else pt + (pt - ordered_pts[i-1])
            dx, dy = next_pt[0]-pt[0], next_pt[1]-pt[1]
            yaw = np.arctan2(dy, dx)
            final_wps.append([pt[0], pt[1], yaw])

        self.waypoints = torch.tensor(final_wps, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Generated {len(self.waypoints)} waypoints near spawn.")
        self.refresh_visuals()

    def get_closest_waypoint_data(self, pos: torch.Tensor) -> torch.Tensor:
        if self.waypoints is None:
             raise RuntimeError("TrackManager: Cannot find closest waypoint. Waypoints not loaded.")
        dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
        closest_indices = torch.argmin(dists, dim=1)
        return self.waypoints[closest_indices]

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor):
        wp_data = self.get_closest_waypoint_data(pos)
        wp_yaw = wp_data[:, 2]
        head_err = torch.atan2(torch.sin(yaw - wp_yaw), torch.cos(yaw - wp_yaw))
        v_wp_to_rob = pos[:, :2] - wp_data[:, :2]
        v_normal = torch.stack([-torch.sin(wp_yaw), torch.cos(wp_yaw)], dim=-1)
        lat_err = torch.sum(v_wp_to_rob * v_normal, dim=-1)
        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
