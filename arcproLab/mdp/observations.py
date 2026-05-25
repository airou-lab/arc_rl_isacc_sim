# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv
import isaaclab.envs.mdp as mdp


def get_telemetry_vector(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """
    12-D telemetry vector. Layout conforms to the policy repo's TELEMETRY_INDICES
    (see ~/airou/shared/REUNIFICATION_V1_PROMPT.md edit #4):

        0: turn_token     (sim has no Worker  -> 0.0)
        1: go_signal      (sim has no Scheduler -> 0.0)
        2: goal_dist      (PVP-masked -> 0.0)
        3: speed          (body-frame X velocity, m/s)
        4: yaw_rate       (body-frame Z angular velocity, rad/s)
        5: last_steer     (action[:, 0])
        6: last_throttle  (action[:, 1])
        7: last_brake     (sim brake actuator deferred per OM 2-B -> 0.0)
        8: lat_err        (PVP-masked -> 0.0)
        9: head_err       (PVP-masked -> 0.0)
       10: reserved       (0.0)
       11: distance       (cumulative across resets - OM 1.2-C deferred)

    Reward / termination terms still read raw lat_err / head_err via env.extras,
    so the TrackManager call is preserved.
    """
    asset = env.scene[asset_cfg.name]

    obs = torch.zeros((env.num_envs, 12), device=env.device)

    q = asset.data.root_quat_w
    yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2]**2 + q[:, 3]**2))

    env_origins = env.scene.env_origins
    local_pos = asset.data.root_pos_w - env_origins

    # Slots 0, 1, 2: zeroed (no Worker / Scheduler on sim side; PVP mask).
    # Slot 3: forward speed (body-frame X).
    obs[:, 3] = asset.data.root_lin_vel_b[:, 0]

    # Slot 4: yaw rate (body-frame Z).
    obs[:, 4] = asset.data.root_ang_vel_b[:, 2]

    # Slots 5, 6: last steer / last throttle from prior action.
    # Slot 7 (last_brake): held at 0.0 until the brake actuator lands (OM 2-B).
    try:
        action = env.action_manager.action
        if action is not None and action.shape[1] >= 1:
            obs[:, 5] = action[:, 0]
        if action is not None and action.shape[1] >= 2:
            obs[:, 6] = action[:, 1]
    except Exception:
        pass

    # Compute raw lat_err / head_err for reward/termination consumers via env.extras.
    from mdp.track_manager import get_track_manager
    tm = get_track_manager(device=env.device)
    lat_err, head_err = tm.compute_errors(local_pos, yaw)
    env.extras["lat_err"] = lat_err
    env.extras["head_err"] = head_err
    env.extras["raw_lat_err"] = lat_err
    env.extras["raw_speed"] = asset.data.root_lin_vel_b[:, 0]

    # Slots 8, 9, 10: PVP-masked to 0.0 (already zero-initialized).

    # Slot 11: cumulative distance across resets. Reset semantics deferred (OM 1.2-C).
    if "distance" not in env.extras:
        env.extras["distance"] = torch.zeros(env.num_envs, device=env.device)
    env.extras["distance"] += asset.data.root_lin_vel_b[:, 0] * 0.05
    obs[:, 11] = env.extras["distance"]

    if env.num_envs == 1 and env.episode_length_buf[0] % 10 == 0:
        try:
            throttle_action = env.action_manager.action[0, 1].item()
            actual_speed = env.extras["raw_speed"][0].item()
            print(f"[DEBUG] Throttle Cmd: {throttle_action:.3f} | Local X Speed: {actual_speed:.3f}")
        except Exception:
            pass

    nan_mask = torch.isnan(obs)
    if nan_mask.any():
        obs[nan_mask] = 0.0

    return obs


def get_image_uint8(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Tiled-camera RGB image as uint8 in [0, 255] (NHWC).

    The policy repo declares `image` as `Box(0, 255, uint8)` and relies on SB3's
    preprocessing to divide by 255. Shipping float32 in [0, 1] from the sim and
    declaring uint8 on the policy side causes a silent double-scaling collapse
    toward zero (OM 1.3-B). This helper unifies on uint8 at the boundary.
    """
    img = mdp.image(env, sensor_cfg=sensor_cfg, normalize=False)
    if img.dtype != torch.uint8:
        img = img.clamp(0.0, 255.0).to(torch.uint8)
    return img
