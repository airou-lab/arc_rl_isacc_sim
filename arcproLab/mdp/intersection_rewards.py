"""Intersection-compliance reward terms for the ARCPro smart intersection.

Design contract: these functions read ONLY env.extras (plus robot velocity),
never the RoadManager singleton directly. The splice that wires the
IntersectionManager into RoadManager must also publish:

    env.extras["go_signal"]  (num_envs,) 1.0 = proceed, 0.0 = stop
    env.extras["dist_g"]     (num_envs,) unsigned distance to nearest gate
                             (already published by get_telemetry_vector)

Until that splice lands, extras lack "go_signal" and every function below
degrades to the legacy behavior (all-green): gated_speed == speed_reward,
gated_stationary == the old stationary lambda, hold/violation terms are
identically zero. Wiring these into RewardCfg is therefore behavior-neutral
before the splice.

Note: dist_g is min-Euclidean to the nearest gate and the nearest gate
switches after a crossing, so violations are detected zone-based (moving
above v_stop inside violation_radius on red), not by sign flip.
"""

import torch


def _go(env) -> torch.Tensor:
    go = env.extras.get("go_signal")
    if go is None:
        return torch.ones(env.num_envs, device=env.device)
    return go.view(-1)


def _dist_g(env) -> torch.Tensor:
    d = env.extras.get("dist_g")
    if d is None:
        return torch.full((env.num_envs,), 10.0, device=env.device)
    return d.view(-1)


def _planar_speed(env) -> torch.Tensor:
    return torch.norm(env.scene["robot"].data.root_lin_vel_b[:, :2], dim=1)


def _in_red_zone(env, radius: float) -> torch.Tensor:
    return (_go(env) < 0.5) & (_dist_g(env) < radius)


def gated_speed_reward(env, approach_radius: float = 3.0) -> torch.Tensor:
    """speed_reward, but zeroed while approaching a red light.

    Outside the red zone this is identical to mdp_rew.speed_reward
    (forward = -local X, clamped at -1). Inside a red zone, forward
    progress earns nothing -- removing the incentive to run the light.
    """
    local_vel = env.scene["robot"].data.root_lin_vel_b
    fwd = torch.clamp(-local_vel[:, 0], min=-1.0)
    return torch.where(_in_red_zone(env, approach_radius), torch.zeros_like(fwd), fwd)


def gated_stationary_penalty(
    env, approach_radius: float = 3.0, v_stop: float = 0.1
) -> torch.Tensor:
    """The legacy anti-idle penalty, suspended where stopping is correct.

    Legacy: -1 per step whenever planar speed < 0.1. Gated: no penalty
    while stopped in a red zone (that is the desired behavior there).
    """
    stationary = _planar_speed(env) < v_stop
    penalize = stationary & ~_in_red_zone(env, approach_radius)
    zero = torch.zeros(env.num_envs, device=env.device)
    return torch.where(penalize, zero - 1.0, zero)


def hold_at_red_reward(
    env, approach_radius: float = 3.0, v_stop: float = 0.1
) -> torch.Tensor:
    """+1 per step while correctly held stopped at a red light.

    Makes stopping positively attractive rather than merely un-penalized;
    without it the agent is indifferent between stopping and loitering
    just outside the zone.
    """
    held = _in_red_zone(env, approach_radius) & (_planar_speed(env) < v_stop)
    return held.float()


def red_light_violation(
    env, violation_radius: float = 0.5, v_stop: float = 0.1
) -> torch.Tensor:
    """-1 per step while moving through the stop line on red.

    Zone-based: fires only within violation_radius of the gate, on red,
    above stopping speed. An agent decelerating to a stop before the line
    never enters this state. Scale with the RewTerm weight (suggest ~100).
    """
    running = _in_red_zone(env, violation_radius) & (_planar_speed(env) > v_stop)
    return -running.float()


def red_light_violation_termination(
    env, violation_radius: float = 0.5, v_stop: float = 0.1
) -> torch.Tensor:
    """DoneTerm variant of red_light_violation (optional, phase in later).

    Prefer starting with the reward penalty alone; add this termination
    only if the policy plateaus while still running lights, since episode
    termination also triggers the -1000 terminating penalty on top.
    """
    return _in_red_zone(env, violation_radius) & (_planar_speed(env) > v_stop)
