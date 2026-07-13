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
        self.last_indices = None # Tensor (num_envs,) to track closest waypoints
        
        # Performance: Only enable visuals if explicitly requested via --debug flag
        import sys
        self.debug = "--debug" in sys.argv
        
        # Visualizers (Using reliable USD-based markers)
        self.visualizer_yellow = None
        self.visualizer_white = None
        self.visualizer_gate = None
        self.visualizer_waypoints = None
        self.visualizer_target = None
        
        # Point groups
        self.raw_yellow_pts = None 
        self.raw_white_pts = None  
        self.raw_gate_pts = None
        self.yellow_tensor = None  
        self.white_tensor = None   
        self.gate_tensor = None
        self.curvature_tensor = None
        
        self.sync_attempts = 0
        self.max_sync_attempts = 10 
        self.wp_path = os.path.join(os.path.dirname(__file__), "track_centerline.npy")
        self.cache_path = os.path.join(os.path.dirname(__file__), "track_boundaries_1x.npz")

    def ensure_synced(self, target_lane_offset: float = 0.11):
        if self.yellow_tensor is not None and self.sync_attempts >= self.max_sync_attempts:
            return

        if self.sync_attempts == 0:
            # TRY LOADING CACHE FIRST
            if self.load_cache():
                print(f"[TrackManager] Loaded boundaries from cache: {self.cache_path}")
            else:
                print(f"[TrackManager] Building high-fidelity boundary markers from USD (slow)...")
                self.collect_raw_marker_points()
                self.save_cache()
            
            # Convert to Tensors
            if self.raw_yellow_pts is not None:
                self.yellow_tensor = torch.tensor(self.raw_yellow_pts, device=self.device, dtype=torch.float32)
            if self.raw_white_pts is not None:
                self.white_tensor = torch.tensor(self.raw_white_pts, device=self.device, dtype=torch.float32)
            if self.raw_gate_pts is not None:
                self.gate_tensor = torch.tensor(self.raw_gate_pts, device=self.device, dtype=torch.float32)
            
            if os.path.exists(self.wp_path):
                wp_np = np.load(self.wp_path)
                self.waypoints = torch.tensor(wp_np, device=self.device, dtype=torch.float32)
                
                # Pre-calculate Curvature (Kappa = dYaw / ds)
                # wp_np: [N, 3] -> [X, Y, Yaw]
                n_wps = len(wp_np)
                kappa = np.zeros(n_wps)
                for i in range(n_wps):
                    i1 = (i + 1) % n_wps
                    dyaw = wp_np[i1, 2] - wp_np[i, 2]
                    # Handle yaw wrap-around
                    while dyaw > np.pi: dyaw -= 2 * np.pi
                    while dyaw < -np.pi: dyaw += 2 * np.pi
                    
                    ds = np.linalg.norm(wp_np[i1, :2] - wp_np[i, :2])
                    if ds > 0:
                        kappa[i] = dyaw / ds
                
                self.curvature_tensor = torch.tensor(kappa, device=self.device, dtype=torch.float32)
                print(f"[TrackManager] Calculated curvature for {n_wps} waypoints.")
            else:
                self.waypoints = torch.tensor([[-16.25, 5.56, -1.57]], device=self.device)
                self.curvature_tensor = torch.zeros(1, device=self.device)

        # Persistent visuals for GUI (Only if --debug is set)
        if self.debug and self.sync_attempts < 1000:
            self.refresh_visuals(target_lane_offset=target_lane_offset)
            self.sync_attempts += 1
        else:
            # Still increment sync_attempts to stop building the tensor repeatedly
            self.sync_attempts += 1

    def load_cache(self):
        if not os.path.exists(self.cache_path): return False
        try:
            data = np.load(self.cache_path)
            self.raw_yellow_pts = data['yellow']
            self.raw_white_pts = data['white']
            self.raw_gate_pts = data['gate']
            return True
        except Exception as e:
            print(f"[TrackManager] Cache load failed: {e}")
            return False

    def save_cache(self):
        try:
            # Ensure we don't save None objects
            y = self.raw_yellow_pts if self.raw_yellow_pts is not None else np.array([])
            w = self.raw_white_pts if self.raw_white_pts is not None else np.array([])
            g = self.raw_gate_pts if self.raw_gate_pts is not None else np.array([])
            np.savez(self.cache_path, yellow=y, white=w, gate=g)
            print(f"[TrackManager] Saved boundaries to cache: {self.cache_path}")
        except Exception as e:
            print(f"[TrackManager] Cache save failed: {e}")

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
            resolution = 0.01 # High resolution (1cm) for 1.0x scale
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
            path_str = str(prim.GetPath())
            is_gate_prim = False
            
            # Match by primvar (Standard)
            if prim.HasAttribute("primvars:ds_type"):
                attr_val = prim.GetAttribute("primvars:ds_type").Get()
                if attr_val == ["DSLaneGate"] or attr_val == "DSLaneGate":
                    is_gate_prim = True
            
            # Fallback: Match by name (Consistent with RoadManager)
            if not is_gate_prim and "laneGate" in path_str:
                is_gate_prim = True

            if is_gate_prim:
                gate_prim_paths.append(path_str)
                
                # Fallback: Get center of gate prim in Local Env Frame
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pw = world_transform.ExtractTranslation()
                pl = [pw[0] - env0_origin[0], pw[1] - env0_origin[1], pw[2] - env0_origin[2]]
                gate_fallback_pts.append(pl)
        
        if len(gate_prim_paths) > 0:
            print(f"[TrackManager] Found {len(gate_prim_paths)} gate prims. Example: {gate_prim_paths[0]}")
            
        # Add the right-turn intersection manually since it lacks a gate prim
        gate_fallback_pts.append([-15.95, 6.19, 0.0])
        print(f"[TrackManager] Added manual right-turn intersection fallback. Total fallbacks: {len(gate_fallback_pts)}")

        # 2. Now traverse meshes in env_0 and categorize
        meshes_found_in_gates = 0
        for prim in Usd.PrimRange(root_prim):
            # CRITICAL FIX: Skip the robot itself!
            if "Robot" in str(prim.GetPath()): continue
            
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
            # 1. Explicit names for Gates
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

            # CRITICAL FIX: Only pick up road markings, not the road itself.
            # We look for 'roadmarks' or 'paint' in the path, and ensure it's not the main asphalt.
            is_marking = "roadmarks" in path_str or "paint" in path_str or "marking" in path_str.lower()
            
            if is_marking:
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
            
        # 3. CRITICAL: Punch Holes for Gate Permeability
        # If any white/yellow point is within 1.0m of a Gate point, REMOVE IT.
        # This guarantees the robot can pass through gates without triggering termination.
        if self.raw_gate_pts is not None:
            gate_xy = self.raw_gate_pts[:, :2]
            
            def punch_holes(raw_pts, name):
                if raw_pts is None or len(raw_pts) == 0: return None
                from scipy.spatial import KDTree
                tree = KDTree(gate_xy)
                dists, _ = tree.query(raw_pts[:, :2], k=1)
                
                # Keep only points further than 1.0m from any gate
                mask = dists > 1.0
                filtered = raw_pts[mask]
                print(f"[TrackManager] Punched holes in {name}: removed {len(raw_pts) - len(filtered)} overlapping points.")
                return filtered

            self.raw_yellow_pts = punch_holes(self.raw_yellow_pts, "Yellow")
            self.raw_white_pts = punch_holes(self.raw_white_pts, "White")

    def sample_waypoints_from_usd(self, density=0.1):
        """Generates track centerline waypoints from USD boundary markers using a KD-Tree."""
        if self.raw_yellow_pts is None or self.raw_white_pts is None:
            self.collect_raw_marker_points()
            
        y_pts = self.raw_yellow_pts[:, :2]
        w_pts = self.raw_white_pts[:, :2]
        
        # 1. Use KDTree to find the nearest white point for each yellow point
        from scipy.spatial import KDTree
        tree = KDTree(w_pts)
        dists, idxs = tree.query(y_pts)
        
        # 2. The midpoint is the centerline
        midpoints = (y_pts + w_pts[idxs]) / 2.0
        
        # 3. Filter valid midpoints (track width usually < 4.0m)
        valid_mask = dists < 4.0
        midpoints = midpoints[valid_mask]
        
        # 4. Sequence the points using Fast Nearest Neighbor
        # Start at origin or closest to origin
        dists_to_origin = np.linalg.norm(midpoints, axis=1)
        curr = np.argmin(dists_to_origin)
        
        tree_nn = KDTree(midpoints)
        visited = np.zeros(len(midpoints), dtype=bool)
        visited[curr] = True
        ordered = [midpoints[curr]]
        
        for _ in range(len(midpoints) - 1):
            curr_pt = ordered[-1]
            dists, idxs = tree_nn.query(curr_pt, k=50) # Look at 50 nearest to find an unvisited one
            
            found = False
            for idx in np.atleast_1d(idxs):
                if not visited[idx]:
                    visited[idx] = True
                    ordered.append(midpoints[idx])
                    found = True
                    break
            
            if not found:
                # Fallback: find closest globally if gap is large
                unvisited_idxs = np.where(~visited)[0]
                if len(unvisited_idxs) == 0: break
                unvisited_pts = midpoints[unvisited_idxs]
                dists_glob = np.linalg.norm(unvisited_pts - curr_pt, axis=1)
                best_idx = unvisited_idxs[np.argmin(dists_glob)]
                visited[best_idx] = True
                ordered.append(midpoints[best_idx])
                
        ordered = np.array(ordered)
        
        # 5. Compute Yaw
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
        
        self.waypoints = torch.tensor(final_wps, device=self.device, dtype=torch.float32)
        print(f"[TrackManager] Generated {n} sequenced waypoints.")

    def save_waypoints(self, path):
        if self.waypoints is not None:
            np.save(path, self.waypoints.cpu().numpy())
            print(f"[TrackManager] Saved waypoints to {path}")

    def compute_marker_distances(self, pos: torch.Tensor):
        self.ensure_synced()
        # FIX: Use 2D distance for XY plane, ignoring Z height of the robot chassis
        dist_y = torch.min(torch.cdist(pos[:, :2], self.yellow_tensor[:, :2]), dim=1)[0]
        dist_w = torch.min(torch.cdist(pos[:, :2], self.white_tensor[:, :2]), dim=1)[0]
        
        # Gates distance (permeable)
        if self.gate_tensor is not None:
            dist_g = torch.min(torch.cdist(pos[:, :2], self.gate_tensor[:, :2]), dim=1)[0]
        else:
            dist_g = torch.ones_like(dist_y) * 10.0
            
        return dist_y, dist_w, dist_g


    def refresh_visuals(self, target_lane_offset: float = 0.0):
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
                
            def create_gate_v(path, color, radius=0.20):
                cfg = VisualizationMarkersCfg(prim_path=path, markers={"dot": sim_utils.SphereCfg(radius=radius, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))})
                return VisualizationMarkers(cfg)

            def to_w(pts, z_off=0.05):
                if pts is None: return None
                w = torch.zeros((len(pts), 3), device=self.device)
                w[:, 0], w[:, 1], w[:, 2] = torch.tensor(pts[:, 0], device=self.device) + origin[0], torch.tensor(pts[:, 1], device=self.device) + origin[1], torch.tensor(pts[:, 2], device=self.device) + origin[2] + z_off
                return w

            if self.visualizer_yellow is None: self.visualizer_yellow = create_v("/World/Visuals/YellowSpheres", (1.0, 1.0, 0.0))
            if self.visualizer_white is None: self.visualizer_white = create_v("/World/Visuals/WhiteSpheres", (1.0, 1.0, 1.0))
            if self.visualizer_gate is None: self.visualizer_gate = create_gate_v("/World/Visuals/GateSpheres", (0.0, 1.0, 0.0), radius=0.20)
            if self.visualizer_waypoints is None: self.visualizer_waypoints = create_v("/World/Visuals/CyanPath", (0.0, 1.0, 1.0))
            if self.visualizer_target is None: self.visualizer_target = create_v("/World/Visuals/TargetLane", (1.0, 0.0, 1.0))

            if self.raw_yellow_pts is not None: self.visualizer_yellow.visualize(to_w(self.raw_yellow_pts[::16]))
            if self.raw_white_pts is not None: self.visualizer_white.visualize(to_w(self.raw_white_pts[::16]))
            
            # User request: "remove the green spheres"
            # if self.raw_gate_pts is not None: self.visualizer_gate.visualize(to_w(self.raw_gate_pts, z_off=0.40))
            
            # User request: "cyan and purple are wrong direction though. But i thought we arent using them"
            # We will disable the visualization to avoid confusion, since the policy is vision-only.
            # (Note: the reward function still uses them for lateral error!)
            # if self.waypoints is not None:
            #     wp_np = self.waypoints.cpu().numpy()
            #     
            #     # 1. Visualize Base Centerline (Cyan)
            #     self.visualizer_waypoints.visualize(to_w(wp_np[:, :2]))
            #     
            #     # 2. Visualize Target Offset Path (Magenta)
            #     yaws = wp_np[:, 2]
            #     normals = np.stack([-np.sin(yaws), np.cos(yaws)], axis=1)
            #     target_pts = wp_np[:, :2] + target_lane_offset * normals
            #     target_viz = np.zeros((len(wp_np), 3))
            #     target_viz[:, :2] = target_pts
            #     target_viz[:, 2] = 0.0
            #     self.visualizer_target.visualize(to_w(target_viz, z_off=0.18))
        except: pass

    def compute_errors(self, pos: torch.Tensor, yaw: torch.Tensor, target_lane_offset: torch.Tensor | float = 0.0):
        """
        Calculates lateral and heading error relative to the track centerline.
        Uses Windowed Search for O(WindowSize) performance.
        
        target_lane_offset: Tensor (B,) or float. Allows independent targets per agent.
        """
        self.ensure_synced()
        num_envs_agents = pos.shape[0] # Total agents across all envs
        num_wps = self.waypoints.shape[0]

        # Ensure target_lane_offset is a tensor for vectorized math
        if not isinstance(target_lane_offset, torch.Tensor):
            target_lane_offset = torch.full((num_envs_agents,), target_lane_offset, device=self.device)

        # 1. Initialize or Re-sync last_indices if batch size changed
        if self.last_indices is None or self.last_indices.shape[0] != num_envs_agents:
            dists = torch.cdist(pos[:, :2], self.waypoints[:, :2])
            self.last_indices = torch.argmin(dists, dim=1)
        
        # 2. Windowed Search: Look +/- 50 waypoints around last known index
        window_size = 50
        offsets = torch.arange(-window_size, window_size + 1, device=self.device)
        search_indices = (self.last_indices.view(-1, 1) + offsets) % num_wps
        
        candidate_wps = self.waypoints[search_indices][:, :, :2]
        candidate_dists = torch.norm(pos[:, None, :2] - candidate_wps, dim=2)
        min_dists, best_in_window_idx = torch.min(candidate_dists, dim=1)
        
        closest_idx = torch.gather(search_indices, 1, best_in_window_idx.view(-1, 1)).view(-1)
        
        # FIX: Fallback to Global Search for any envs that teleported (e.g. resets)
        # If the closest waypoint in the window is > 2.0 meters away, the robot teleported.
        global_search_mask = min_dists > 2.0
        if global_search_mask.any():
            global_dists = torch.cdist(pos[global_search_mask, :2], self.waypoints[:, :2])
            best_global_idx = torch.argmin(global_dists, dim=1)
            closest_idx[global_search_mask] = best_global_idx
            
        self.last_indices = closest_idx

        # 3. Calculate Errors
        closest_wp = self.waypoints[closest_idx] # Always (B, 3) because closest_idx is (B,)
        wp_to_robot = pos[:, :2] - closest_wp[:, :2]
        wp_yaw = closest_wp[:, 2]
        
        kappa = self.curvature_tensor[closest_idx]
        normal_x, normal_y = torch.sin(wp_yaw), -torch.cos(wp_yaw)
        raw_lat_err = wp_to_robot[:, 0] * normal_x + wp_to_robot[:, 1] * normal_y
        
        # Vectorized offset application
        lat_err = raw_lat_err - target_lane_offset
        head_err = yaw - wp_yaw
        
        # Normalize to [-pi, pi]
        # This prevents the 180-degree 'flip' that allowed backwards driving to look correct
        head_err = torch.atan2(torch.sin(head_err), torch.cos(head_err))
        
        return lat_err, head_err, kappa

    def get_forward_waypoints(self, pos: torch.Tensor, yaw: torch.Tensor, offsets=[20, 40, 60, 80, 100]):
        """
        Extracts future waypoints relative to the robot's local frame.
        This provides the RL policy with forward-looking track curvature,
        bypassing the need for a fully converged CNN vision backbone to anticipate turns.
        """
        if self.last_indices is None:
            return torch.zeros(pos.shape[0], len(offsets) * 2, device=self.device)
            
        num_envs = pos.shape[0]
        num_wps = self.waypoints.shape[0]
        
        target_indices = (self.last_indices.view(-1, 1) + torch.tensor(offsets, device=self.device)) % num_wps
        target_wps = self.waypoints[target_indices][:, :, :2]
        
        # Vector from robot to future waypoints
        rel_pos = target_wps - pos[:, None, :2]
        
        # Rotate into robot's local frame
        cos_y = torch.cos(-yaw)
        sin_y = torch.sin(-yaw)
        
        local_x = rel_pos[:, :, 0] * cos_y.unsqueeze(1) - rel_pos[:, :, 1] * sin_y.unsqueeze(1)
        local_y = rel_pos[:, :, 0] * sin_y.unsqueeze(1) + rel_pos[:, :, 1] * cos_y.unsqueeze(1)
        
        # Flatten: [wp1_x, wp1_y, wp2_x, wp2_y, ...]
        return torch.stack((local_x, local_y), dim=-1).view(num_envs, -1)

_TRACK_MANAGER = None
def get_track_manager(device: str = "cuda:0"):
    global _TRACK_MANAGER
    if _TRACK_MANAGER is None: _TRACK_MANAGER = TrackManager(device=device)
    return _TRACK_MANAGER
