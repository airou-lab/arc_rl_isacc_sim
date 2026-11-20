"""Pure-CPU Contract Suite for Agent Docs, Telemetry Parity, MDP `env.extras` Ordering,
Sentinel Log Isolation, and Search Exclusions (`tests/test_agent_doc_contracts.py`).
"""

from __future__ import annotations

import ast
import json
import re
import sys
from pathlib import Path

import pytest

ROOT_DIR = Path(__file__).resolve().parent.parent
ARCPRO_LAB_DIR = ROOT_DIR / "arcproLab"
SCRIPTS_DIR = ARCPRO_LAB_DIR / "scripts"
for _p in (str(ROOT_DIR), str(ARCPRO_LAB_DIR), str(SCRIPTS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mdp.track_topology import (
    ACTOR_MASKED_TELEMETRY_INDICES,
    TELEMETRY_DIM,
    TELEMETRY_NAMES,
)

pytestmark = pytest.mark.cpu


def _extract_extras_keys(py_path: Path) -> tuple[set[str], set[str]]:
    """AST-extract literal `env.extras` keys written vs. read in a Python file."""
    tree = ast.parse(py_path.read_text(encoding="utf-8"), filename=str(py_path))
    writes: set[str] = set()
    reads: set[str] = set()

    def _is_extras_attr(node: ast.AST) -> bool:
        return isinstance(node, ast.Attribute) and node.attr == "extras"

    def _const_str(node: ast.AST) -> str | None:
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        return None

    for node in ast.walk(tree):
        if isinstance(node, ast.Subscript) and _is_extras_attr(node.value):
            key = _const_str(node.slice)
            if key is not None:
                if isinstance(node.ctx, ast.Store):
                    writes.add(key)
                elif isinstance(node.ctx, ast.Load):
                    reads.add(key)
        elif (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and _is_extras_attr(node.func.value)
            and node.args
        ):
            key = _const_str(node.args[0])
            if key is None:
                continue
            if node.func.attr == "get":
                reads.add(key)
            elif node.func.attr == "setdefault":
                writes.add(key)
                reads.add(key)
            elif node.func.attr == "pop":
                reads.add(key)
    return writes, reads


def test_rl_deep_audit_skill_matches_track_topology_telemetry_contract():
    """Verify `.agents/skills/rl-deep-audit/SKILL.md` matches live telemetry & masking constants."""
    skill_path = ROOT_DIR / ".agents" / "skills" / "rl-deep-audit" / "SKILL.md"
    assert skill_path.is_file(), f"Missing skill file: {skill_path}"
    text = skill_path.read_text(encoding="utf-8")

    # Parse Section 1 markdown table rows (allowing leading indentation):
    # | `0` | `64` | `turn_token` | Actor + Critic | ... |
    row_re = re.compile(
        r"^\s*\|\s*`(\d+)`\s*\|\s*`(\d+)`\s*\|\s*`([^`]+)`\s*\|\s*([^|]+?)\s*\|",
        re.MULTILINE,
    )
    rows = row_re.findall(text)
    assert len(rows) == TELEMETRY_DIM, (
        f"Expected {TELEMETRY_DIM} telemetry rows in SKILL.md, found {len(rows)}"
    )

    parsed_names = []
    parsed_critic_only = []
    for slot_s, head_s, feat_name, consumer in rows:
        slot = int(slot_s)
        head_idx = int(head_s)
        assert head_idx == 64 + slot, (
            f"SKILL.md slot {slot} has actor head idx {head_idx}, expected {64 + slot}"
        )
        parsed_names.append((slot, feat_name.strip()))
        if "critic only" in consumer.lower():
            parsed_critic_only.append(slot)

    parsed_names.sort(key=lambda item: item[0])
    assert [idx for idx, _ in parsed_names] == list(range(TELEMETRY_DIM))
    assert tuple(name for _, name in parsed_names) == TELEMETRY_NAMES, (
        f"SKILL.md telemetry names drifted from track_topology.TELEMETRY_NAMES: "
        f"{tuple(name for _, name in parsed_names)} != {TELEMETRY_NAMES}"
    )
    assert tuple(sorted(parsed_critic_only)) == tuple(
        sorted(ACTOR_MASKED_TELEMETRY_INDICES)
    ), (
        f"SKILL.md Critic-ONLY slots {tuple(sorted(parsed_critic_only))} != "
        f"ACTOR_MASKED_TELEMETRY_INDICES {ACTOR_MASKED_TELEMETRY_INDICES}"
    )

    # Also verify VISUAL_FEATURE_DIM constant mentioned in SKILL.md matches skrl_models.py
    skrl_models_src = (ARCPRO_LAB_DIR / "agents" / "skrl_models.py").read_text(
        encoding="utf-8"
    )
    m_dim = re.search(r"^VISUAL_FEATURE_DIM\s*=\s*(\d+)", skrl_models_src, re.MULTILINE)
    assert m_dim is not None, "Could not locate VISUAL_FEATURE_DIM in skrl_models.py"
    visual_dim = int(m_dim.group(1))
    assert f"VISUAL_FEATURE_DIM = {visual_dim}" in text, (
        f"SKILL.md must reference VISUAL_FEATURE_DIM = {visual_dim}"
    )


def test_rewards_never_depend_on_observations_only_extras():
    """Enforce IsaacLab execution order: rewards.py cannot consume keys written ONLY by observations.py."""
    mdp_dir = ARCPRO_LAB_DIR / "mdp"
    topo_w, _ = _extract_extras_keys(mdp_dir / "track_topology.py")
    term_w, _ = _extract_extras_keys(mdp_dir / "terminations.py")
    events_w, _ = _extract_extras_keys(mdp_dir / "events.py")
    rew_w, rew_r = _extract_extras_keys(mdp_dir / "rewards.py")
    obs_w, _ = _extract_extras_keys(mdp_dir / "observations.py")

    pre_reward_producers = topo_w | term_w | events_w | rew_w
    obs_only_keys = obs_w - pre_reward_producers
    illegal_reads = rew_r & obs_only_keys
    assert not illegal_reads, (
        f"rewards.py reads env.extras keys produced ONLY in observations.py (1-step stale!): "
        f"{sorted(illegal_reads)}"
    )

    # Core spatial road-state keys read by rewards.py must be produced before RewardManager
    required_pre_reward = {
        "lat_err",
        "head_err",
        "kappa",
        "dist_y",
        "dist_w",
        "dist_g",
        "lookahead_kappa",
        "turn_token",
        "go_signal",
    }
    missing = required_pre_reward - (topo_w | term_w)
    assert not missing, (
        f"Required pre-reward spatial keys missing from track_topology/terminations: {sorted(missing)}"
    )


def test_sentinel_logging_writes_jsonl_not_clearset_db(tmp_path, monkeypatch):
    """Verify sentinel event logging writes JSONL and never opens ClearSet `.cs/cs_turns.db`."""
    import sentinel_utils as su

    events_file = tmp_path / "sentinel_events.jsonl"
    monkeypatch.setattr(su, "SENTINEL_EVENTS_LOG", str(events_file))

    assert not hasattr(su, "CTA_DB_PATH"), (
        "sentinel_utils should not keep legacy CTA_DB_PATH SQLite coupling"
    )
    su.log_to_cta_db(
        "FIX",
        "Self-Healing: TEST_EVENT",
        "SUCCESS",
        "curriculum_sectors.json",
        "dry verification",
    )

    assert events_file.is_file()
    lines = events_file.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    rec = json.loads(lines[0])
    assert rec["action_type"] == "FIX"
    assert rec["description"] == "Self-Healing: TEST_EVENT"
    assert rec["status"] == "SUCCESS"


def test_ignore_and_reward_history_hygiene():
    """Verify root `.ignore` excludes bulk dirs/binaries and reward history has unique issue numbers."""
    ignore_path = ROOT_DIR / ".ignore"
    assert ignore_path.is_file(), "Root .ignore file is required for clean ripgrep searches"
    ignore_lines = {
        line.strip()
        for line in ignore_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    }
    required_ignores = {
        "venv/",
        "archive/",
        "logs/",
        "runs/",
        "openStreetUSD/",
        "arcpro-rl-course/",
        ".cs/",
        "*.npz",
        "*.bak",
        "*.pt",
        "*.pth",
        "*.db",
    }
    missing_ignores = required_ignores - ignore_lines
    assert not missing_ignores, f"Root .ignore missing required exclusions: {sorted(missing_ignores)}"

    history_path = ROOT_DIR / ".planning" / "reward_tuning_history.md"
    assert history_path.is_file()
    history_text = history_path.read_text(encoding="utf-8")
    issue_nums = re.findall(r"^###\s+Issue\s+(\d+)\b", history_text, re.MULTILINE)
    assert len(issue_nums) == len(set(issue_nums)), (
        f"Duplicate Issue headers found in reward_tuning_history.md: {issue_nums}"
    )
