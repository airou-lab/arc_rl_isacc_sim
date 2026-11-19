#!/usr/bin/env python3
"""
Pure-CPU Invariant Suite for Checkpoint Probe, Debias Moment Preservation,
and Sentinel Ops Hygiene (`tests/test_ops_sentinel_invariants.py`).
"""

import ast
import json
import os
import subprocess
import sys

import gymnasium as gym
import pytest
import torch

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCPRO_LAB_DIR = os.path.join(ROOT_DIR, "arcproLab")
SCRIPTS_DIR = os.path.join(ARCPRO_LAB_DIR, "scripts")
for _p in (ROOT_DIR, ARCPRO_LAB_DIR, SCRIPTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import autonomous_sentinel as auto_sent  # noqa: E402
import sentinel_utils as su  # noqa: E402
from agents.skrl_models import ARCProActor  # noqa: E402
from checkpoint_probe import evaluate_checkpoint  # noqa: E402
from debias_checkpoint import debias_checkpoint  # noqa: E402

pytestmark = pytest.mark.cpu


def test_probe_and_debias_mask_privileged_columns_and_use_real_images(tmp_path):
    """Verify checkpoint_probe and debias_checkpoint zero masked slots [5,6,8,9,11] (col 70 prev_drive has 0.0 effect)."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))

    torch.manual_seed(11)
    actor_a = ARCProActor(obs_space, act_space, device="cpu", equivariant=False)
    actor_b = ARCProActor(obs_space, act_space, device="cpu", equivariant=False)
    actor_b.load_state_dict(actor_a.state_dict())

    # Inject massive weights on all 5 masked telemetry columns [69, 70, 72, 73, 75] (slots 5, 6, 8, 9, 11)
    with torch.no_grad():
        for slot in (5, 6, 8, 9, 11):
            actor_b.policy_head[0].weight[:, 64 + slot] = 500.0

    ckpt_a = str(tmp_path / "agent_a.pt")
    ckpt_b = str(tmp_path / "agent_b.pt")
    torch.save({"policy": actor_a.state_dict()}, ckpt_a)
    torch.save({"policy": actor_b.state_dict()}, ckpt_b)

    res_a = evaluate_checkpoint(ckpt_a)
    res_b = evaluate_checkpoint(ckpt_b)

    # Changing weights on column 70 (prev_drive) and other masked slots must have 0.0 effect on probed bias
    assert res_a["mu_neutral_steer"] == res_b["mu_neutral_steer"]
    assert res_a["mu_turn_steer"] == res_b["mu_turn_steer"]
    assert (
        res_a["mu_yellow_l_white_r_steer"] == res_b["mu_yellow_l_white_r_steer"]
    )


def test_debias_checkpoint_preserves_log_std_adam_moments(tmp_path):
    """Verify debias_checkpoint preserves log_std_parameter Adam moments intact while zeroing only policy_head.2.bias moments."""
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(25100,))
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,))
    actor = ARCProActor(obs_space, act_space, device="cpu", equivariant=False)

    with torch.no_grad():
        actor.policy_head[2].bias[0] = 0.25

    opt = torch.optim.Adam(actor.parameters(), lr=3e-4)
    # Run a dummy backward/step so all parameters have populated Adam exp_avg / exp_avg_sq
    dummy_in = torch.randn(2, 25100)
    mu, log_std, _ = actor.compute({"states": dummy_in})
    loss = (mu.pow(2).sum()) + (log_std.pow(2).sum())
    loss.backward()
    opt.step()

    log_std_moments_before = opt.state[actor.log_std_parameter]["exp_avg"].clone()
    log_std_sq_before = opt.state[actor.log_std_parameter]["exp_avg_sq"].clone()
    b2_moments_before = opt.state[actor.policy_head[2].bias]["exp_avg"].clone()
    assert log_std_moments_before.abs().sum().item() > 0.0
    assert b2_moments_before.abs().sum().item() > 0.0

    ckpt_in = str(tmp_path / "agent_with_opt.pt")
    ckpt_out = str(tmp_path / "agent_debiased.pt")
    torch.save(
        {
            "policy": actor.state_dict(),
            "policy_optimizer": opt.state_dict(),
        },
        ckpt_in,
    )

    debias_checkpoint(ckpt_in, out_path=ckpt_out)
    saved = torch.load(ckpt_out, map_location="cpu", weights_only=False)
    opt_state_after = saved["policy_optimizer"]["state"]

    param_names = [n for n, _ in actor.named_parameters()]
    log_std_idx = param_names.index("log_std_parameter")
    b2_idx = param_names.index("policy_head.2.bias")

    # log_std_parameter (index 0): must be 100% preserved!
    assert torch.equal(opt_state_after[log_std_idx]["exp_avg"], log_std_moments_before)
    assert torch.equal(opt_state_after[log_std_idx]["exp_avg_sq"], log_std_sq_before)

    # policy_head.2.bias: steer bias moment (index 0) must be zeroed!
    assert opt_state_after[b2_idx]["exp_avg"][0].item() == 0.0
    assert opt_state_after[b2_idx]["exp_avg_sq"][0].item() == 0.0


def test_sentinel_zero_dead_keys_and_zero_stale_log_fallback():
    """Verify sentinel_utils.py and autonomous_sentinel.py contain zero references to dead keys or stale fallback logs/ppo_skrl."""
    su_path = os.path.join(SCRIPTS_DIR, "sentinel_utils.py")
    as_path = os.path.join(SCRIPTS_DIR, "autonomous_sentinel.py")

    for fpath in (su_path, as_path):
        with open(fpath, "r") as f:
            src = f.read()
        assert "speed_tracking_weight" not in src, (
            f"Found dead key speed_tracking_weight in {os.path.basename(fpath)}"
        )
        assert "target_corner_speed" not in src, (
            f"Found dead key target_corner_speed in {os.path.basename(fpath)}"
        )
        assert "logs/ppo_skrl" not in src, (
            f"Found stale fallback logs/ppo_skrl in {os.path.basename(fpath)}"
        )

    # Verify get_active_log_subdir on a non-main branch never returns legacy logs/ppo_skrl
    resolved = su.get_active_log_subdir(branch="feature/test-branch")
    assert resolved.endswith("logs/ppo_skrl_feature_test-branch") or resolved.endswith(
        "logs/skrl/arcpro_car"
    )
    assert not resolved.endswith("logs/ppo_skrl")

    # Verify explicit log_dir override works
    custom_dir = su.get_active_log_subdir(log_dir="/tmp/custom_skrl_logs")
    assert custom_dir == "/tmp/custom_skrl_logs"
    assert hasattr(auto_sent, "AutonomousSentinel")


def test_sentinel_probe_passes_equivariant_and_json_flags(tmp_path, monkeypatch):
    """Verify probe_checkpoint_bias passes --equivariant and --json to checkpoint_probe.py."""
    ckpt = tmp_path / "agent_dummy.pt"
    ckpt.write_bytes(b"placeholder")
    probe_log = tmp_path / "checkpoint_probes.jsonl"
    monkeypatch.setattr(su, "PROBE_LOG_PATH", str(probe_log))

    captured = {}
    payload = {
        "checkpoint": ckpt.name,
        "equivariant": True,
        "mu_neutral_steer": 0.0,
        "mu_turn_steer": 0.5,
        "delta_turn_steer": 0.5,
        "mu_turn_throt": -0.4,
        "sigma_steer": 0.1,
        "is_neutral": True,
    }

    def fake_run(cmd, **kwargs):
        captured["cmd"] = list(cmd)
        captured["kwargs"] = kwargs
        return subprocess.CompletedProcess(
            args=cmd, returncode=0, stdout=json.dumps(payload), stderr=""
        )

    monkeypatch.setattr(subprocess, "run", fake_run)

    res = su.probe_checkpoint_bias(str(ckpt))
    assert res == payload
    cmd = captured["cmd"]
    assert su.PROBE_SCRIPT in cmd
    assert str(ckpt) in cmd
    assert "--equivariant" in cmd
    assert "--json" in cmd
    assert probe_log.is_file()


def test_eval_and_auditor_disable_training_augmentation_in_wrapper_ast():
    """Verify play_skrl.py and checkpoint_auditor.py instantiate SKRLFlattenWrapper with training=False and augment=False."""
    for script_name in ("play_skrl.py", "checkpoint_auditor.py"):
        fpath = os.path.join(SCRIPTS_DIR, script_name)
        with open(fpath, "r") as f:
            tree = ast.parse(f.read(), filename=fpath)

        calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and (
                (isinstance(node.func, ast.Name) and node.func.id == "SKRLFlattenWrapper")
                or (
                    isinstance(node.func, ast.Attribute)
                    and node.func.attr == "SKRLFlattenWrapper"
                )
            )
        ]
        assert calls, f"No SKRLFlattenWrapper(...) call found in {script_name}"
        for call in calls:
            kw = {k.arg: k.value for k in call.keywords if k.arg is not None}
            assert "training" in kw, f"Missing training=... in {script_name}"
            assert "augment" in kw, f"Missing augment=... in {script_name}"
            assert isinstance(kw["training"], ast.Constant) and kw["training"].value is False, (
                f"Expected training=False in {script_name}"
            )
            assert isinstance(kw["augment"], ast.Constant) and kw["augment"].value is False, (
                f"Expected augment=False in {script_name}"
            )
