# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Manages track waypoints and provides vectorized distance/heading queries.
    Used by ObservationManager to compute lateral and heading errors.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        self.visualizer = None # Debug markers
        self.density = 0.5 # Default density
        
        # Use USD path from standard location
        self.usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/openStreetUSD/no_graph_sim_clean_1x.usda"
        
        # PRIORITIZE PROCEDURAL SAMPLING
        print(f"[TrackManager] Initializing procedural waypoint generation from road markers...")
        try:
            self.sample_waypoints_from_usd(density=self.density)
        except Exception as e:
            print(f"[TrackManager] Procedural sampling failed ({e}), falling back to file...")
            self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline_1x.npy")
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
        
        # 3. OFFSET TO LANE CENTER (Visual & Logic): 
        # When facing South (-Y), Right is -X. Double yellow is at current X.
        # We shift X by -0.5625 to put waypoints in the center of the right lane.
        # This makes compute_errors return 0 when robot is centered in the lane.
        data[:, 0] -= 0.5625
        
        # Use .copy() to fix negative strides
        self.waypoints = torch.tensor(data.copy(), device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Loaded {len(self.waypoints)} waypoints (REVERSED, FLIPPED SOUTH, OFFSET 0.56m)")
        
        # Initialize visualizer if in GUI mode
        try:
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            import isaaclab.sim as sim_utils
            
            # Use arrows to show direction
            marker_cfg = VisualizationMarkersCfg(
                prim_path="/Visuals/TrackWaypoints",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{sim_utils.ISAAC_ASSETS_DIR}/Visuals/checkpoint_arrow.usd",
                        scale=(0.2, 0.2, 0.2),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0))
                    )
                }
            )
            self.visualizer = VisualizationMarkers(marker_cfg)
            
            # Visualize waypoints
            vis_pos = self.waypoints[::10, :3].clone()
            vis_pos[:, 2] = 0.1 # Sit on road
            
            # Create quaternions for South facing (Yaw = -1.57)
            import math
            num_wps = vis_pos.shape[0]
            quats = torch.zeros((num_wps, 4), device=self.device)
            # South is -90 deg around Z
            half_yaw = -1.5708 / 2.0
            quats[:, 0] = math.cos(half_yaw)
            quats[:, 3] = math.sin(half_yaw)
            
            self.visualizer.visualize(vis_pos, quats)
            print(f"[TrackManager] Visualized {len(vis_pos)} waypoints as ARROWS")
        except Exception as e:
            print(f"[TrackManager] Visualization error: {e}")
        except Exception as e:
            print(f"[TrackManager] Visualization error: {e}")

    def save_waypoints(self, path: str):
        """Saves waypoints to a .npy file."""
        if self.waypoints is not None:
            data = self.waypoints.cpu().numpy()
            np.save(path, data)
            print(f"[TrackManager] Saved {len(self.waypoints)} waypoints to {path}")

    def sample_waypoints_from_usd(self, density: float = 0.5):
        """
        Scans the USD stage for road meshes and generates centerline waypoints.
        Args:
            density: Desired distance between waypoints in meters.
        """
        import omni.usd
        from pxr import Usd, UsdGeom, Gf
        
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage not found. USD sampling requires a running simulation.")

        print(f"[TrackManager] Sampling waypoints from road markers (Procedural)")
        yellow_pts = []
        white_pts = []
        
        # The track is scaled by 0.125 in the scene config.
        # ComputeLocalToWorldTransform already includes this scale.
        env0_track_path = "/World/envs/env_0/Track"
        track_prim = stage.GetPrimAtPath(env0_track_path)
        
        # Get env_0 origin to ensure waypoints are local
        env0_origin = Gf.Vec3d(0, 0, 0)
        env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
        if env0_prim.IsValid():
            env0_xform = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            env0_origin = env0_xform.ExtractTranslation()

        from pxr import UsdShade
        for prim in Usd.PrimRange(track_prim if track_prim.IsValid() else stage.GetPseudoRoot()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
                
            path = str(prim.GetPath())
            if "roadmarks" not in path.lower() and "piece" not in path.lower():
                continue
                
            binding_api = UsdShade.MaterialBindingAPI(prim)
            material, _ = binding_api.ComputeBoundMaterial()
            if not material:
                continue
                
            mat_path = str(material.GetPath()).lower()
            mesh = UsdGeom.Mesh(prim)
            points_attr = mesh.GetPointsAttr().Get()
            if not points_attr:
                continue
                
            xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            for p in points_attr:
                p_world = xform.Transform(p)
                # Subtract env origin to get local coordinates
                p_local = [p_world[0] - env0_origin[0], p_world[1] - env0_origin[1]]
                
                if "yellow" in mat_path:
                    yellow_pts.append(p_local)
                elif "white" in mat_path:
                    white_pts.append(p_local)

        if not yellow_pts or not white_pts:
            pts = np.array(yellow_pts + white_pts)
        else:
            y_pts = np.unique(np.round(np.array(yellow_pts), 2), axis=0)
            w_pts = np.unique(np.round(np.array(white_pts), 2), axis=0)
            lane_centers = []
            for yp in y_pts:
                dists = np.sum((w_pts - yp)**2, axis=1)
                center = (yp + w_pts[np.argmin(dists)]) / 2.0
                lane_centers.append(center)
            pts = np.unique(np.round(np.array(lane_centers), 2), axis=0)

        # 2. Simple ordering (Nearest Neighbor)
        ordered_pts = []
        spawn_target = np.array([-16.25, 5.56])
        start_dists = np.sum((pts[:, :2] - spawn_target)**2, axis=1)
        curr_idx = np.argmin(start_dists)
        
        visited = set([curr_idx])
        ordered_pts.append(pts[curr_idx])
        
        while len(visited) < len(pts):
            last_pt = ordered_pts[-1]
            dists = np.sum((pts - last_pt)**2, axis=1)
            for v in visited: dists[v] = np.inf
            next_idx = np.argmin(dists)
            if dists[next_idx] > 2.0**2: break
            ordered_pts.append(pts[next_idx])
            visited.add(next_idx)
            
        ordered_pts = np.array(ordered_pts)

        # 3. Resample to fixed density
        diffs = np.diff(ordered_pts, axis=0)
        segment_dists = np.sqrt(np.sum(diffs**2, axis=1))
        cum_dist = np.concatenate(([0], np.cumsum(segment_dists)))
        total_dist = cum_dist[-1]
        num_samples = int(total_dist / density)
        interp_dists = np.linspace(0, total_dist, num_samples)
        
        resampled_x = np.interp(interp_dists, cum_dist, ordered_pts[:, 0])
        resampled_y = np.interp(interp_dists, cum_dist, ordered_pts[:, 1])
        
        dx = np.gradient(resampled_x)
        dy = np.gradient(resampled_y)
        yaws = np.arctan2(dy, dx)
        
        wps = np.stack([resampled_x, resampled_y, yaws], axis=1)
        
        # 4. Offset to Right Lane Center
        # Road center is -15.835, Spawn is -16.255.
        # We need to shift waypoints along their normal by ~0.42m to the right.
        # For South-facing (Yaw -1.57), Right is -X.
        
        # Simple directional offset for now (assume mostly straight for spawn region)
        # In a full track we'd use the normal vector for each waypoint
        dx = np.gradient(resampled_x)
        dy = np.gradient(resampled_y)
        # Normal vector (-dy, dx)
        norms = np.stack([-dy, dx], axis=1)
        norms = norms / np.linalg.norm(norms, axis=1)[:, None]
        
        # Shift waypoints by 0.42m along the normal (towards the right lane)
        # Using -0.42 because +Normal is Left (West)
        wps[:, :2] += norms * -0.42
        
        self.waypoints = torch.tensor(wps, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Procedurally generated {len(self.waypoints)} waypoints (OFFSET TO RIGHT LANE).")

    def get_closest_waypoint_data(self, pos: torch.Tensor) -> torch.Tensor:
        """
        Finds the closest waypoint for each environment.
        Args:
            pos: (num_envs, 3) - World positions of robots
        Returns:
            (num_envs, 3) - [wp_x, wp_y, wp_yaw] of closest waypoints
        """
        # (num_envs, 1, 2) - robots
        # (1, num_waypoints, 2) - waypoints
        diff = pos[:, None, :2] - self.waypoints[None, :, :2]
        dist_sq = torch.sum(diff**2, dim=-1)
        closest_indices = torch.argmin(dist_sq, dim=-1)
        return self.waypoints[closest_indices]

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor):
        """
        Computes lateral and heading errors relative to the track centerline.
        Args:
            pos: (num_envs, 3) - World positions
            yaw: (num_envs,) - Current yaw angles
        Returns:
            lat_err: (num_envs,)
            head_err: (num_envs,)
        """
        wp_data = self.get_closest_waypoint_data(pos)
        wp_pos = wp_data[:, :2]
        wp_yaw = wp_data[:, 2]
        
        # Heading error: robot_yaw - track_yaw
        head_err = yaw - wp_yaw
        # Wrap to [-pi, pi]
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        
        # Lateral error: signed distance to the track centerline
        # vector from wp to robot
        v_wp_to_rob = pos[:, :2] - wp_pos
        # track tangent vector (cos(yaw), sin(yaw))
        v_tangent = torch.stack([torch.cos(wp_yaw), torch.sin(wp_yaw)], dim=-1)
        # track normal vector (-sin(yaw), cos(yaw))
        v_normal = torch.stack([-torch.sin(wp_yaw), torch.cos(wp_yaw)], dim=-1)
        
        # Signed distance = dot product with normal
        lat_err = torch.sum(v_wp_to_rob * v_normal, dim=-1)
        
        return lat_err, head_err

# Singleton instance for the environment
_TRACK_MANAGER = None

def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None:
        _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
