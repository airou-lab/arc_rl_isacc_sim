#!/usr/bin/env python3
"""
Pure-CPU Invariant Suite for Reflection-Equivariant Vision Architecture, Smooth Log-Std,
Critic Compression, Decoupled TelemetryPPO, and Pass-Through WarmupActionWrapper
(`tests/test_rl_vision_invariants.py`).
"""

import copy
import os
import sys

import gymnasium as gym
import numpy as np
import pytest
import torch
from skrl.memories.torch import RandomMemory

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mdp.rewards as mdp_rew  # noqa: E402
from agents.skrl_models import (  # noqa: E402
    ARCProActor,
    ARCProCritic,
    synthesize_road_image,
)
from agents.skrl_wrappers import WarmupActionWrapper  # noqa: E402
from agents.telemetry_ppo import PPO_DEFAULT_CONFIG, TelemetryPPO  # noqa: E402
from mdp.track_topology import (  # noqa: E402
    ACTION_MIRROR_SIGNS,
    BOUNDARY_WARN_MARGIN,
    TELEMETRY_MIRROR_SIGNS,
)

pytestmark = pytest.mark.cpu


def test_actor_reflection_equivariance_and_zero_symmetric_steer():
    """Verify G(flip(I)) == spatial_parity * G(I), mu(flip(I), M_tel*t) == M_act * mu(I, t), and centered_sym -> mu_steer == 0.0."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    torch.manual_seed(123)
    actor = ARCProActor(
        obs_space, act_space, device="cpu", equivariant=True, vision_mode="rgb"
    )
    actor.eval()

    scenarios = (
        "centered_sym",
        "yellow_L_white_R",
        "white_L_yellow_R",
        "left_turn",
        "right_turn",
    )
    m_tel = torch.tensor(TELEMETRY_MIRROR_SIGNS, dtype=torch.float32)
    m_act = torch.tensor(ACTION_MIRROR_SIGNS, dtype=torch.float32)

    tel_sample = torch.tensor(
        [[-1.0, 1.0, 4.5, 0.52, 0.18, 0.25, 0.40, -0.05, 0.12, -0.15, 0.65, 12.0]],
        dtype=torch.float32,
    )

    with torch.no_grad():
        for sc in scenarios:
            img = synthesize_road_image(sc, device="cpu")
            img_flip = torch.flip(img, dims=[3])

            g_orig = actor.G(img)
            g_flip = actor.G(img_flip)
            fe_err = (g_flip - actor.spatial_parity * g_orig).abs().max().item()
            assert fe_err < 1e-5, (
                f"Frontend parity error for {sc}: {fe_err:.3e} >= 1e-5"
            )

            mu_orig = actor.forward_from_image(img, tel_sample)
            mu_flip = actor.forward_from_image(img_flip, tel_sample * m_tel)
            equiv_err = (mu_flip - mu_orig * m_act).abs().max().item()
            assert equiv_err < 1e-5, (
                f"Policy equivariance error for {sc}: {equiv_err:.3e} >= 1e-5"
            )

        # Verify centered_sym with neutral symmetric telemetry outputs mu_steer == 0.0 (< 1e-6)
        tel_neutral = torch.zeros((1, 12), dtype=torch.float32)
        tel_neutral[0, 1] = 1.0  # go_signal
        tel_neutral[0, 2] = 5.0  # goal_dist
        tel_neutral[0, 3] = 0.50  # speed
        img_sym = synthesize_road_image("centered_sym", device="cpu")
        mu_sym = actor.forward_from_image(img_sym, tel_neutral)
        assert abs(float(mu_sym[0, 0].item())) < 1e-6


def test_smooth_tanh_log_std_gradient_non_zero():
    """Verify get_log_std() smooth tanh bijection has strictly positive gradient (> 0.01) at -3.0 and +3.0."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    actor = ARCProActor(obs_space, act_space, device="cpu")

    for val in (-3.0, 0.0, 3.0):
        with torch.no_grad():
            actor.log_std_parameter.fill_(val)
        actor.zero_grad()
        eff_log_std = actor.get_log_std()
        assert (eff_log_std >= -2.0 - 1e-5).all()
        assert (eff_log_std <= 0.5 + 1e-5).all()
        eff_log_std.sum().backward()
        grad_min = actor.log_std_parameter.grad.abs().min().item()
        assert grad_min > 0.01, (
            f"Expected smooth log_std gradient > 0.01 at {val}, got {grad_min:.6f}"
        )


def test_critic_distance_compression_and_running_stats():
    """Verify ARCProCritic compresses distance=500.0m cleanly without exploding output or gradients."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    critic = ARCProCritic(obs_space, act_space, device="cpu")

    states = torch.zeros((4, 25100), dtype=torch.float32, requires_grad=True)
    with torch.no_grad():
        states[:, 25088 + 11] = 500.0  # 500m cumulative distance
    val, _ = critic.compute({"states": states})
    val.sum().backward()

    assert torch.isfinite(val).all()
    assert val.abs().max().item() < 50.0
    assert states.grad is not None and torch.isfinite(states.grad).all()


def test_decoupled_telemetry_ppo_zero_critic_starvation_and_gae_invariants():
    """Verify decoupled optimizers (0.0 Actor step change under 1e6 Critic spike), value_preprocessor, shuffling, GAE timeout bootstrap, and persistent episode window."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    num_envs = 4
    rollouts = 8

    def _run_ppo_step(
        spike_critic_returns: float,
        final_truncated: bool = False,
        final_terminated: bool = False,
        use_value_prep: bool = True,
    ):
        torch.manual_seed(999)
        np.random.seed(999)
        actor = ARCProActor(obs_space, act_space, device="cpu")
        critic = ARCProCritic(obs_space, act_space, device="cpu")
        with torch.no_grad():
            critic.critic_head[-1].bias.fill_(10.0)
        memory = RandomMemory(memory_size=rollouts, num_envs=num_envs, device="cpu")

        cfg = copy.deepcopy(PPO_DEFAULT_CONFIG)
        cfg["rollouts"] = rollouts
        cfg["mini_batches"] = 2
        cfg["learning_epochs"] = 2
        cfg["learning_rate"] = 3e-4
        cfg["value_learning_rate"] = 1e-3
        cfg["mirror_symmetry_weight"] = 0.5
        cfg["learning_rate_scheduler"] = None
        cfg["kl_threshold"] = 0.0
        if not use_value_prep:
            cfg["value_preprocessor"] = False

        agent = TelemetryPPO(
            models={"policy": actor, "value": critic},
            memory=memory,
            cfg=cfg,
            observation_space=obs_space,
            action_space=act_space,
            device="cpu",
            num_envs=num_envs,
        )
        agent.init()
        agent.set_mode("train")

        for i in range(rollouts):
            torch.manual_seed(100 + i)
            s = torch.randn(num_envs, 25100)
            a = torch.clamp(torch.randn(num_envs, 2), -1.0, 1.0)
            r = torch.tensor([[1.1], [0.9], [1.2], [0.8]])
            ns = torch.randn(num_envs, 25100)
            term = torch.zeros(num_envs, 1, dtype=torch.bool)
            trunc = torch.zeros(num_envs, 1, dtype=torch.bool)
            if i == rollouts - 1:
                if final_terminated:
                    term[:] = True
                if final_truncated:
                    trunc[:] = True
            lp = torch.zeros(num_envs, 1)
            v = torch.ones(num_envs, 1) * 2.0
            memory.add_samples(
                states=s,
                actions=a,
                rewards=r,
                next_states=ns,
                terminated=term,
                truncated=trunc,
                log_prob=lp,
                values=v,
            )

        epoch_perms = []
        orig_sample_by_index = memory.sample_by_index

        def _hooked_sample_by_index(names, indexes, mini_batches=1):
            epoch_perms.append([int(x) for x in indexes])
            batches = orig_sample_by_index(
                names=names, indexes=indexes, mini_batches=mini_batches
            )
            if spike_critic_returns > 0.0:
                spiked = []
                for b in batches:
                    # b = (states, actions, log_prob, values, returns, advantages)
                    spiked.append(
                        (b[0], b[1], b[2], b[3], b[4] + spike_critic_returns, b[5])
                    )
                return spiked
            return batches

        memory.sample_by_index = _hooked_sample_by_index

        agent._current_next_states = torch.zeros(num_envs, 25100)
        init_actor = {k: v.clone() for k, v in actor.state_dict().items()}
        agent._update(timestep=rollouts, timesteps=100)
        deltas = {k: (actor.state_dict()[k] - init_actor[k]) for k in init_actor}
        return agent, deltas, epoch_perms, memory.get_tensor_by_name("returns").clone()

    agent_normal, deltas_normal, perms_normal, _ = _run_ppo_step(spike_critic_returns=0.0)
    agent_spike, deltas_spike, _, _ = _run_ppo_step(spike_critic_returns=2000.0)

    # 1. Under a >10^6 Critic MSE spike, Actor weight step must be 100% identical (diff < 1e-7)
    v_loss_spike = agent_spike.tracking_data["Loss / Value loss"][-1]
    assert v_loss_spike > 1e6, f"Expected spiked V_loss > 1e6, got {v_loss_spike}"
    max_actor_step_diff = max(
        (deltas_normal[k] - deltas_spike[k]).abs().max().item() for k in deltas_normal
    )
    assert max_actor_step_diff < 1e-7, (
        f"Critic gradient starvation detected! Actor step differed by {max_actor_step_diff:.3e}"
    )

    # 2. Verify value_preprocessor updated running stats and mini-batches shuffled across epochs
    assert agent_normal._value_preprocessor is not None
    assert len(perms_normal) == 2
    assert perms_normal[0] != perms_normal[1]

    # 3. Verify GAE truncated bootstrapping: when truncated=True (and terminated=False), next_values = last_values != 0
    _, _, _, ret_boot = _run_ppo_step(
        spike_critic_returns=0.0,
        final_truncated=True,
        final_terminated=False,
        use_value_prep=False,
    )
    _, _, _, ret_kill = _run_ppo_step(
        spike_critic_returns=0.0,
        final_truncated=False,
        final_terminated=True,
        use_value_prep=False,
    )
    assert (ret_boot[-1] - ret_kill[-1]).abs().min().item() > 5.0

    # 4. Verify persistent _ep_rew_window never drops to 0.0 across write_interval=100 steps without new terminations
    s0 = torch.zeros((num_envs, 25100))
    a0 = torch.zeros((num_envs, 2))
    # Step 1: episode completes with reward 250.0
    agent_normal.record_transition(
        states=s0,
        actions=a0,
        rewards=torch.full((num_envs, 1), 250.0),
        next_states=s0,
        terminated=torch.ones((num_envs, 1), dtype=torch.bool),
        truncated=torch.zeros((num_envs, 1), dtype=torch.bool),
        infos={},
        timestep=1,
        timesteps=500,
    )
    # Next 198 steps: no episodes terminate, tracking_data cleared after first write_interval=100
    for step in range(2, 200):
        if step == 100:
            agent_normal.tracking_data.clear()
        agent_normal.record_transition(
            states=s0,
            actions=a0,
            rewards=torch.full((num_envs, 1), 1.0),
            next_states=s0,
            terminated=torch.zeros((num_envs, 1), dtype=torch.bool),
            truncated=torch.zeros((num_envs, 1), dtype=torch.bool),
            infos={},
            timestep=step,
            timesteps=500,
        )
    rew_mean_logged = agent_normal.tracking_data.get(
        "Reward / Total reward (mean)", [0.0]
    )[-1]
    assert rew_mean_logged >= 200.0


def test_warmup_action_wrapper_never_overwrites_policy_actions():
    """Verify WarmupActionWrapper passes policy actions straight through without overwriting."""

    class _MockEnv(gym.Env):
        def __init__(self):
            self.num_envs = 2
            self.device = "cpu"
            self.observation_space = gym.spaces.Box(-1.0, 1.0, (2, 12))
            self.action_space = gym.spaces.Box(-1.0, 1.0, (2, 2))
            self.last_action_received = None

        def reset(self, **kwargs):
            return torch.zeros((2, 12)), {}

        def step(self, action):
            self.last_action_received = action.clone()
            return (
                torch.zeros((2, 12)),
                torch.zeros(2),
                torch.zeros(2, dtype=torch.bool),
                torch.zeros(2, dtype=torch.bool),
                {},
            )

    base = _MockEnv()
    wrapped = WarmupActionWrapper(base, warmup_steps=10)
    wrapped.reset()

    policy_action = torch.tensor([[0.42, 0.73], [-0.31, 0.15]], dtype=torch.float32)
    wrapped.step(policy_action)
    assert torch.equal(base.last_action_received, policy_action)


def test_critic_running_stats_persistence_and_legacy_load(tmp_path):
    """Verify ARCProCritic persists running scaler buffers in state_dict and loads legacy checkpoints under strict=True / agent.load()."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))

    torch.manual_seed(77)
    critic_src = ARCProCritic(obs_space, act_space, device="cpu")
    sd_init = critic_src.state_dict()
    for buf_key in ("obs_running_mean", "obs_running_var", "obs_count"):
        assert buf_key in sd_init, f"Missing persistent buffer '{buf_key}' in state_dict()"

    batch = torch.randn((16, 25100), dtype=torch.float32)
    batch[:, 25088:25100] = torch.linspace(-2.0, 4.0, steps=16 * 12).view(16, 12)
    batch[:, 25088 + 11] = torch.linspace(0.0, 120.0, steps=16)
    critic_src.update_telemetry_stats(batch)
    critic_src.update_telemetry_stats(batch * 1.5 + 0.25)

    assert not torch.equal(critic_src.obs_running_mean, torch.zeros(12))
    assert not torch.equal(critic_src.obs_running_var, torch.ones(12))
    assert float(critic_src.obs_count.item()) > 1.0

    ckpt_path = tmp_path / "critic_persistent.pt"
    torch.save(critic_src.state_dict(), ckpt_path)

    critic_dst = ARCProCritic(obs_space, act_space, device="cpu")
    loaded_sd = torch.load(ckpt_path, map_location="cpu", weights_only=True)
    incompat = critic_dst.load_state_dict(loaded_sd, strict=True)
    assert incompat.missing_keys == []
    assert incompat.unexpected_keys == []
    assert torch.equal(critic_dst.obs_running_mean, critic_src.obs_running_mean)
    assert torch.equal(critic_dst.obs_running_var, critic_src.obs_running_var)
    assert torch.equal(critic_dst.obs_count, critic_src.obs_count)

    probe = torch.randn((4, 25100), dtype=torch.float32)
    with torch.no_grad():
        val_src, _ = critic_src.compute({"states": probe})
        val_dst, _ = critic_dst.compute({"states": probe})
    assert torch.allclose(val_src, val_dst, atol=1e-6)

    # Legacy checkpoint omitting the 3 running scaler buffers must load cleanly under strict=True
    legacy_sd = {
        k: v.clone()
        for k, v in critic_src.state_dict().items()
        if k not in ("obs_running_mean", "obs_running_var", "obs_count")
    }
    critic_legacy = ARCProCritic(obs_space, act_space, device="cpu")
    legacy_incompat = critic_legacy.load_state_dict(legacy_sd, strict=True)
    assert legacy_incompat.missing_keys == []
    assert legacy_incompat.unexpected_keys == []
    assert torch.equal(critic_legacy.obs_running_mean, torch.zeros(12))
    assert torch.equal(critic_legacy.obs_running_var, torch.ones(12))
    assert torch.equal(critic_legacy.obs_count, torch.tensor(1.0))
    assert torch.equal(
        critic_legacy.critic_head[0].weight, critic_src.critic_head[0].weight
    )

    # Loading a legacy checkpoint onto an already-updated critic restores clean defaults
    critic_dirty = ARCProCritic(obs_space, act_space, device="cpu")
    critic_dirty.update_telemetry_stats(batch)
    critic_dirty.load_state_dict(legacy_sd, strict=True)
    assert torch.equal(critic_dirty.obs_running_mean, torch.zeros(12))
    assert torch.equal(critic_dirty.obs_running_var, torch.ones(12))
    assert torch.equal(critic_dirty.obs_count, torch.tensor(1.0))

    # Genuinely broken checkpoints (missing weights or unexpected keys) still fail under strict=True
    broken_missing = {k: v for k, v in legacy_sd.items() if k != "critic_head.0.weight"}
    with pytest.raises(RuntimeError):
        critic_legacy.load_state_dict(broken_missing, strict=True)

    broken_unexpected = dict(legacy_sd)
    broken_unexpected["unexpected_legacy_key"] = torch.tensor(0.0)
    with pytest.raises(RuntimeError):
        critic_legacy.load_state_dict(broken_unexpected, strict=True)

    # End-to-end TelemetryPPO.load() compatibility with a legacy checkpoint
    actor = ARCProActor(obs_space, act_space, device="cpu")
    critic_agent = ARCProCritic(obs_space, act_space, device="cpu")
    critic_agent.update_telemetry_stats(batch)
    memory = RandomMemory(memory_size=4, num_envs=2, device="cpu")
    cfg = copy.deepcopy(PPO_DEFAULT_CONFIG)
    cfg["learning_rate_scheduler"] = None
    agent = TelemetryPPO(
        models={"policy": actor, "value": critic_agent},
        memory=memory,
        cfg=cfg,
        observation_space=obs_space,
        action_space=act_space,
        device="cpu",
        num_envs=2,
    )
    agent.init()

    legacy_agent_ckpt = tmp_path / "legacy_agent.pt"
    torch.save({"policy": actor.state_dict(), "value": legacy_sd}, legacy_agent_ckpt)
    agent.load(str(legacy_agent_ckpt))
    assert torch.equal(
        critic_agent.critic_head[0].weight, critic_src.critic_head[0].weight
    )
    assert torch.equal(critic_agent.obs_running_mean, torch.zeros(12))
    assert torch.equal(critic_agent.obs_running_var, torch.ones(12))
    assert torch.equal(critic_agent.obs_count, torch.tensor(1.0))


def test_boundary_penalty_symmetry_and_smooth_zero_steer():
    """Verify symmetric white/yellow boundary_penalty and smooth Gaussian straightaway_zero_steer_penalty."""

    class _DummyActionManager:
        def __init__(self, action: torch.Tensor):
            self.action = action

    class _DummyRewardEnv:
        def __init__(self, num_envs: int):
            self.num_envs = num_envs
            self.device = "cpu"
            self.extras = {"_road_state_cache_valid": True}
            self.action_manager = _DummyActionManager(
                torch.zeros((num_envs, 2), dtype=torch.float32)
            )

    # 1. Symmetric boundary_penalty outside stop bar
    env_b = _DummyRewardEnv(num_envs=4)
    env_b.extras.update(
        {
            "local_pos": torch.tensor([[-16.2, 0.0]] * 4, dtype=torch.float32),
            "dist_g": torch.full((4,), 5.0, dtype=torch.float32),
            "dist_y": torch.tensor([0.10, 0.25, 0.12, 0.14], dtype=torch.float32),
            "dist_w": torch.tensor([0.25, 0.10, 0.14, 0.12], dtype=torch.float32),
        }
    )
    pen_b = mdp_rew.boundary_penalty(env_b)
    expected_single = -(BOUNDARY_WARN_MARGIN - 0.10) * 160.0
    assert torch.isclose(pen_b[0], torch.tensor(expected_single))
    assert torch.isclose(pen_b[0], pen_b[1])
    assert torch.isclose(pen_b[2], pen_b[3])

    # Inside stop bar: white penalty is masked while yellow penalty remains active
    env_sb = _DummyRewardEnv(num_envs=2)
    env_sb.extras.update(
        {
            "local_pos": torch.tensor([[-5.5, -7.1], [-5.5, -7.1]], dtype=torch.float32),
            "dist_g": torch.tensor([0.20, 0.20], dtype=torch.float32),
            "dist_y": torch.tensor([0.10, 0.25], dtype=torch.float32),
            "dist_w": torch.tensor([0.10, 0.10], dtype=torch.float32),
        }
    )
    pen_sb = mdp_rew.boundary_penalty(env_sb)
    assert torch.isclose(pen_sb[0], torch.tensor(expected_single))
    assert torch.isclose(pen_sb[1], torch.tensor(0.0))

    # 2. Smooth Gaussian straightaway_zero_steer_penalty across 2.5cm + preserved corner gating
    lat_errs = torch.tensor([0.0, 0.024, 0.026, 0.04, 0.08], dtype=torch.float32)
    env_s = _DummyRewardEnv(num_envs=5)
    env_s.action_manager.action[:, 0] = 0.5
    env_s.extras.update(
        {
            "local_pos": torch.tensor([[-16.2, 0.0]] * 5, dtype=torch.float32),
            "lat_err": lat_errs,
            "kappa": torch.zeros(5, dtype=torch.float32),
            "turn_token": torch.zeros(5, dtype=torch.float32),
        }
    )
    pen_s = mdp_rew.straightaway_zero_steer_penalty(env_s)
    expected_s = -0.25 * torch.exp(-torch.square(lat_errs / 0.04))
    assert torch.allclose(pen_s, expected_s, atol=1e-6)
    # Smooth across old 2.5cm cutoff (non-zero at 2.6cm and no discontinuous cliff)
    assert pen_s[2].item() < -0.15
    assert abs(pen_s[1].item() - pen_s[2].item()) < 0.02
    assert torch.all(torch.abs(pen_s[:-1]) > torch.abs(pen_s[1:]))

    # Preserved corner zone gating: in_any_corner | |kappa| > 0.15 | |turn_token| >= 0.1 -> 0.0
    env_c = _DummyRewardEnv(num_envs=3)
    env_c.action_manager.action[:, 0] = 0.5
    env_c.extras.update(
        {
            "local_pos": torch.tensor(
                [[-16.2, -6.8], [-16.2, 0.0], [-16.2, 0.0]], dtype=torch.float32
            ),
            "lat_err": torch.zeros(3, dtype=torch.float32),
            "kappa": torch.tensor([0.0, 0.20, 0.0], dtype=torch.float32),
            "turn_token": torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32),
        }
    )
    pen_c = mdp_rew.straightaway_zero_steer_penalty(env_c)
    assert torch.allclose(pen_c, torch.zeros(3, dtype=torch.float32))
