# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import numpy as np
import os

class TrackManager:
    """
    Robust Track Manager with VisualizationMarkers and Gap-Filling.
    Ensures 'Dense Walls' for reliable termination and visible debug spheres.
    """
    def __init__(self, device: str = "cuda:0"):
        self.device = device
        self.waypoints = None
        
        # Performance: Only enable visuals if explicitly requested via --debug flag
        import sys
        self.debug = "--debug" in sys.argv
        
        # Visualizers (Using reliable USD-based markers)
        self.visualizer_yellow = None
        self.visualizer_white = None
        self.visualizer_gate = None
        self.visualizer_waypoints = None
        
        # Point groups
        self.raw_yellow_pts = None 
        self.raw_white_pts = None  
        self.raw_gate_pts = None
        self.yellow_tensor = None  
        self.white_tensor = None   
        self.gate_tensor = None
        
        self.sync_attempts = 0
        self.max_sync_attempts = 10 
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline_1x.npy")

    def ensure_synced(self):
        if self.yellow_tensor is not None and self.sync_attempts >= self.max_sync_attempts:
            return

        if self.sync_attempts == 0:
            print(f"[TrackManager] Building high-fidelity boundary markers...")
            self.collect_raw_marker_points()
            
            if self.raw_yellow_pts is not None:
                self.yellow_tensor = torch.tensor(self.raw_yellow_pts, device=self.device, dtype=torch.float32)
            if self.raw_white_pts is not None:
                self.white_tensor = torch.tensor(self.raw_white_pts, device=self.device, dtype=torch.float32)
            if self.raw_gate_pts is not None:
                self.gate_tensor = torch.tensor(self.raw_gate_pts, device=self.device, dtype=torch.float32)
            
            if os.path.exists(self.wp_path):
                self.waypoints = torch.tensor(np.load(self.wp_path), device=self.device, dtype=torch.float32)
            else:
                self.waypoints = torch.tensor([[-16.25, 5.56, -1.57]], device=self.device)

        # Persistent visuals for GUI (Only if --debug is set)
        if self.debug and self.sync_attempts < 1000:
            self.refresh_visuals()
            self.sync_attempts += 1
        else:
            # Still increment sync_attempts to stop building the tensor repeatedly
            self.sync_attempts += 1

    def collect_raw_marker_points(self):
        import omni.usd
        from pxr import Usd, UsdGeom, UsdShade, Gf
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath("/World/envs/env_0")
        if not root_prim.IsValid(): root_prim = stage.GetPseudoRoot()
        xform_cache = UsdGeom.XformCache()
        env0_origin = xform_cache.GetLocalToWorldTransform(root_prim).ExtractTranslation()

        def get_mesh_info(prim, xform_cache, env0_origin):
            mesh = UsdGeom.Mesh(prim)
            points_attr = mesh.GetPointsAttr().Get()
            indices_attr = mesh.GetFaceVertexIndicesAttr().Get()
            counts_attr = mesh.GetFaceVertexCountsAttr().Get()
            if not points_attr or not indices_attr: return None
            
            xform = xform_cache.GetLocalToWorldTransform(prim)
            transformed_pts = []
            for p in points_attr:
                pw = xform.Transform(p)
                transformed_pts.append([pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]])
            return {'points': transformed_pts, 'indices': indices_attr, 'counts': counts_attr}

        def finalize_group(data_list, name):
            if not data_list: return None
            all_dense_pts = []
            resolution = 0.05
            for path, mesh_data in data_list:
                pts = np.array(mesh_data['points'])
                indices = mesh_data['indices']
                counts = mesh_data['counts']
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
            if not all_dense_pts: return None
            final_pts = np.unique(np.round(np.array(all_dense_pts), 3), axis=0)
            print(f"[TrackManager] Finalized {name}: {len(final_pts)} points.")
            return final_pts

        y_data, w_data, g_data = [], [], []
        
        # 1. First, find all Gates explicitly (Global Search like verify_gates.py)
        gate_prim_paths = []
        gate_fallback_pts = []
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.HasAttribute("primvars:ds_type"):
                attr_val = prim.GetAttribute("primvars:ds_type").Get()
                if attr_val == ["DSLaneGate"] or attr_val == "DSLaneGate":
                    g_path = str(prim.GetPath())
                    gate_prim_paths.append(g_path)
                    
                    # Fallback: Get center of gate prim in Local Env Frame
                    xform = UsdGeom.Xformable(prim)
                    world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    pw = world_transform.ExtractTranslation()
                    pl = [pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]]
                    gate_fallback_pts.append(pl)
        
        if len(gate_prim_paths) > 0:
            print(f"[TrackManager] Found {len(gate_prim_paths)} gate prims. Example: {gate_prim_paths[0]}")

        # 2. Now traverse meshes in env_0 and categorize
        meshes_found_in_gates = 0
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsA(UsdGeom.Mesh): continue
            
            path_str = str(prim.GetPath())
            
            # Check if this mesh belongs to a gate (Flexible check)
            is_gate = "laneGate" in path_str or "lane_gate" in path_str or "stop_line" in path_str.lower()
            if not is_gate:
                for g_path in gate_prim_paths:
                    if path_str.startswith(g_path):
                        is_gate = True
                        break

            mesh_info = get_mesh_info(prim, xform_cache, env0_origin)
            if not mesh_info or len(mesh_info['points']) == 0: continue

            binding_api = UsdShade.MaterialBindingAPI(prim)
            mat, _ = binding_api.ComputeBoundMaterial()
            mat_path = str(mat.GetPath() if mat else "").lower()
            search_str = (path_str + mat_path).lower()

            # PROXIMITY CHECK: If mesh center is near a gate marker, it MIGHT be a gate
            is_near_gate_marker = False
            if len(gate_fallback_pts) > 0:
                mesh_center = np.mean(mesh_info['points'], axis=0)
                for g_pos in gate_fallback_pts:
                    if np.linalg.norm(mesh_center - np.array(g_pos)) < 1.0: # Tightened to 1.0m
                        is_near_gate_marker = True
                        break

            # CATEGORIZATION LOGIC:
            # 1. Explicit names
            if "lanegate" in path_str.lower() or "lane_gate" in path_str.lower() or "stop_line" in path_str.lower():
                is_gate = True
            # 2. White marks very close to logical gate markers
            elif "white" in search_str and is_near_gate_marker:
                is_gate = True
            else:
                is_gate = False

            if is_gate:
                g_data.append((path_str, mesh_info))
                meshes_found_in_gates += 1
                continue

            if "yellow" in search_str: 
                y_data.append((path_str, mesh_info))
            elif "white" in search_str: 
                w_data.append((path_str, mesh_info))

        self.raw_yellow_pts = finalize_group(y_data, "Yellow")
        self.raw_white_pts = finalize_group(w_data, "White")
        
        # Finalize Gate points, using fallbacks if no meshes were found
        self.raw_gate_pts = finalize_group(g_data, "Gate")
        if self.raw_gate_pts is None and len(gate_fallback_pts) > 0:
            print("[TrackManager] No gate meshes found. Using prim centers as fallback.")
            self.raw_gate_pts = np.array(gate_fallback_pts)

    def compute_marker_distances(self, pos: torch.Tensor):
        self.ensure_synced()
        dist_y = torch.min(torch.cdist(pos[:, :3], self.yellow_tensor), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos[:, :3], self.white_tensor), dim=1)[0]
        
        # Gates distance (permeable)
        if self.gate_tensor is not None:
            dist_g = torch.min(torch.cdist(pos[:, :3], self.gate_tensor), dim=1)[0]
        else:
            dist_g = torch.ones_like(dist_y) * 10.0
            
        return dist_y, dist_w, dist_g


    def refresh_visuals(self):
        try:
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            import isaaclab.sim as sim_utils
            from pxr import UsdGeom, Gf, Usd
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            env0_prim = stage.GetPrimAtPath("/World/envs/env_0")
            origin = UsdGeom.Xformable(env0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation() if env0_prim.IsValid() else Gf.Vec3d(0,0,0)

            def create_v(path, color, radius=0.05):
                cfg = VisualizationMarkersCfg(prim_path=path, markers={"dot": sim_utils.SphereCfg(radius=radius, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))})
                return VisualizationMarkers(cfg)

            def to_w(pts, z_off=0.1):
                if pts is None: return None
                w = torch.zeros((len(pts), 3), device=self.device)
                w[:, 0], w[:, 1], w[:, 2] = torch.tensor(pts[:, 0], device=self.device) + origin[0], torch.tensor(pts[:, 1], device=self.device) + origin[1], torch.tensor(pts[:, 2], device=self.device) + origin[2] + z_off
                return w

            if self.visualizer_yellow is None: self.visualizer_yellow = create_v("/World/Visuals/YellowSpheres", (1.0, 1.0, 0.0))
            if self.visualizer_white is None: self.visualizer_white = create_v("/World/Visuals/WhiteSpheres", (1.0, 1.0, 1.0))
            if self.visualizer_gate is None: self.visualizer_gate = create_v("/World/Visuals/GateSpheres", (0.0, 1.0, 0.0), radius=0.1)
            if self.visualizer_waypoints is None: self.visualizer_waypoints = create_v("/World/Visuals/CyanPath", (0.0, 1.0, 1.0), radius=0.08)

            if self.raw_yellow_pts is not None: self.visualizer_yellow.visualize(to_w(self.raw_yellow_pts))
            if self.raw_white_pts is not None: self.visualizer_white.visualize(to_w(self.raw_white_pts))
            if self.raw_gate_pts is not None: self.visualizer_gate.visualize(to_w(self.raw_gate_pts, z_off=0.15))
            if self.waypoints is not None:
                # IMPORTANT: waypoints are [X, Y, Yaw]. Force Z=0 for visualization.
                wp_np = self.waypoints.cpu().numpy()
                wp_viz = np.zeros((len(wp_np), 3))
                wp_viz[:, :2] = wp_np[:, :2]
                wp_viz[:, 2] = 0.0 # Force ground level
                self.visualizer_waypoints.visualize(to_w(wp_viz, z_off=0.15))
        except: pass

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor, target_lane_offset: float = 0.238):
        self.ensure_synced()
        dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
        closest_idx = torch.argmin(dists, dim=1)
        closest_wp = self.waypoints[closest_idx]
        wp_to_robot = pos[:, :2] - closest_wp[:, :2]
        wp_yaw = closest_wp[:, 2]
        normal_x, normal_y = -torch.sin(wp_yaw), torch.cos(wp_yaw)
        raw_lat_err = wp_to_robot[:, 0] * normal_x + wp_to_robot[:, 1] * normal_y
        lat_err, head_err = raw_lat_err - target_lane_offset, yaw - wp_yaw
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        import math
        head_err = torch.where(head_err > math.pi/2, head_err - math.pi, head_err)
        head_err = torch.where(head_err < -math.pi/2, head_err + math.pi, head_err)
        return lat_err, head_err

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None: _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
