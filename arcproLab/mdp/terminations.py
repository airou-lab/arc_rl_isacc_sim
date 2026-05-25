# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

# Intersection traversal FSM thresholds:
#  - CROSS_NEAR  : within this distance counts as "touching" a marker.
#                  Triggers a NORMAL -> JUST_CROSSED transition (treated as
#                  the moment of crossing a stop line / intersection paint).
#  - EXIT_DIST   : meters of travel past the crossing point before we
#                  re-enable marker termination (must exceed the ~0.5 m
#                  intersection width with comfortable buffer).
#
# Earlier we tried a (last_dist > 0.15 AND now < 0.10) gate to require a
# crisp far-to-near transition, but the robot approaches markers gradually
# (0.12 -> 0.09 in a single step is normal) and the gate never fired.
# Using first-contact-while-NORMAL as the trigger is simpler and matches
# the intent: any time the robot freshly touches a marker, assume it's
# attempting a crossing and grant a free intersection-traversal window.
_CROSS_NEAR = 0.10
_EXIT_DIST = 2.0
_STATE_NORMAL = 0
_STATE_JUST_CROSSED = 1


def white_line_contact(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Marker-contact termination with an intersection-traversal FSM.

    Per env, we track a state machine:
      NORMAL       : yellow or white marker proximity < CROSS_NEAR -> terminate
      JUST_CROSSED : marker terminations suspended; robot is presumed inside an
                     intersection and may need to drive past stop lines / lane
                     paint that's spatially adjacent to it. Re-enters NORMAL
                     after the robot has travelled EXIT_DIST meters from where
                     it first touched the marker.

    The transition NORMAL -> JUST_CROSSED fires on a crossing event: prior
    step's distance > CROSS_FAR AND current distance < CROSS_NEAR (either
    yellow or white). The same step's termination check is then bypassed,
    so the crossing itself doesn't end the episode.

    State self-corrects on episode reset: cross_position is from the previous
    episode and the new spawn is much farther than EXIT_DIST, so we transition
    JUST_CROSSED -> NORMAL on the first step of a new episode.
    """
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    asset = env.scene[asset_cfg.name]

    pos_local = asset.data.root_pos_w - env.scene.env_origins
    pos_xy = pos_local[:, :2]
    N = env.num_envs

    dist_y, dist_w, closest_y_xy, closest_w_xy = (
        tm.compute_marker_distances_with_positions(pos_local)
    )

    # Initialise FSM state on first call.
    if "stop_line_state" not in env.extras:
        env.extras["stop_line_state"] = torch.zeros(N, dtype=torch.long, device=env.device)
        env.extras["cross_position"] = torch.zeros(N, 2, device=env.device)

    state = env.extras["stop_line_state"]
    cross_pos = env.extras["cross_position"]

    # First-contact-while-NORMAL: simpler and reliable. Any time the robot
    # is in NORMAL state AND inside CROSS_NEAR of any marker, treat as a
    # crossing event.
    in_normal = state == _STATE_NORMAL
    contact = (dist_y < _CROSS_NEAR) | (dist_w < _CROSS_NEAR)
    just_crossed_now = in_normal & contact

    state = torch.where(just_crossed_now, torch.full_like(state, _STATE_JUST_CROSSED), state)
    cross_pos = torch.where(just_crossed_now.unsqueeze(-1), pos_xy, cross_pos)

    # JUST_CROSSED -> NORMAL once we've travelled EXIT_DIST past the crossing.
    dist_from_cross = torch.norm(pos_xy - cross_pos, dim=1)
    exited = (state == _STATE_JUST_CROSSED) & (dist_from_cross > _EXIT_DIST)
    state = torch.where(exited, torch.full_like(state, _STATE_NORMAL), state)

    # Persist updated state for next call.
    env.extras["stop_line_state"] = state
    env.extras["cross_position"] = cross_pos

    # Termination: only when we're in NORMAL state AFTER the crossing
    # check (i.e., not the env that just transitioned to JUST_CROSSED).
    inner_hit = dist_y < _CROSS_NEAR
    outer_hit = dist_w < _CROSS_NEAR
    in_normal_after = state == _STATE_NORMAL
    marker_hit = (inner_hit | outer_hit) & in_normal_after

    if marker_hit[0].item():
        rob_xy = pos_xy[0].detach().cpu().numpy()
        if inner_hit[0].item():
            reason = "Yellow Boundary Hit (NORMAL)"
            marker_xy = closest_y_xy[0].detach().cpu().numpy()
        else:
            reason = "White Boundary Hit (NORMAL)"
            marker_xy = closest_w_xy[0].detach().cpu().numpy()
        print(
            f"[TERMINATION] {reason}! "
            f"Robot=({rob_xy[0]:+.2f}, {rob_xy[1]:+.2f}) "
            f"Marker=({marker_xy[0]:+.2f}, {marker_xy[1]:+.2f}) "
            f"DistY={dist_y[0].item():.3f}m DistW={dist_w[0].item():.3f}m"
        )
    elif just_crossed_now[0].item():
        rob_xy = pos_xy[0].detach().cpu().numpy()
        print(
            f"[CROSSING] Entered JUST_CROSSED at "
            f"({rob_xy[0]:+.2f}, {rob_xy[1]:+.2f}) "
            f"DistY={dist_y[0].item():.3f}m DistW={dist_w[0].item():.3f}m"
        )
    elif exited[0].item():
        rob_xy = pos_xy[0].detach().cpu().numpy()
        print(
            f"[EXITED] Back to NORMAL at "
            f"({rob_xy[0]:+.2f}, {rob_xy[1]:+.2f}) "
            f"(travelled {dist_from_cross[0].item():.2f}m past crossing)"
        )

    return marker_hit

def fov_visibility_termination(env: ManagerBasedRLEnv, horizontal_aperture: float, focal_length: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot drives in a direction outside its camera's FOV."""
    import math
    half_fov = math.atan(horizontal_aperture / (2.0 * focal_length))
    asset = env.scene[asset_cfg.name]
    vel_b = asset.data.root_lin_vel_b
    driving_angle = torch.atan2(vel_b[:, 1], vel_b[:, 0])
    speed = torch.norm(vel_b[:, :2], dim=-1)
    settled = env.episode_length_buf > 20
    out_of_view = settled & (speed > 0.5) & (torch.abs(driving_angle) > half_fov)
    return out_of_view

def height_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot flips or falls."""
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return (height < 0.02) | (height > 0.5)

def stagnation_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Terminates if the robot hasn't made forward progress."""
    if "last_dist" not in env.extras:
        env.extras["last_dist"] = torch.zeros(env.num_envs, device=env.device)
        env.extras["stagnant_steps"] = torch.zeros(env.num_envs, device=env.device)
    current_dist = env.extras.get("distance", torch.zeros(env.num_envs, device=env.device))
    progress = (current_dist - env.extras["last_dist"]) > 0.01
    env.extras["stagnant_steps"] = torch.where(progress, torch.zeros_like(env.extras["stagnant_steps"]), env.extras["stagnant_steps"] + 1)
    env.extras["last_dist"] = current_dist.clone()
    return env.extras["stagnant_steps"] > 500
