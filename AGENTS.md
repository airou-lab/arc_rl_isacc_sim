# AGENTS.md — ARCPro RL (Isaac Sim / IsaacLab / SKRL)

> **Source-of-Truth Rule:** Executable code and `pytest -m cpu` contract tests override Markdown on any conflict. Do not duplicate live symbol tables or coordinate literals into prose.

## Core Commands

- **Fast pure-CPU invariant & contract suite (~6s, no Isaac Sim):**
  ```bash
  ./venv/bin/pytest -m cpu -q
  ```
- **CPU equivariant steering-bias probe (~0.1s):**
  ```bash
  ./venv/bin/python arcproLab/scripts/checkpoint_probe.py <ckpt.pt> --equivariant --json
  ```
- **5-sector headless Isaac Sim regression gate (stop training first if VRAM > 9 GB):**
  ```bash
  /home/arika/IsaacLab/isaaclab.sh -p arcproLab/scripts/checkpoint_auditor.py --checkpoint <ckpt.pt> --steps 200
  ```
- **Training & headed GUI playback:**
  - `./start_tmux_training.sh` (auto-resumes latest checkpoint on active branch in tmux `training`; `--fresh` for Step 0).
  - `./debug.sh --sector <1..5>` (headed GUI evaluation; needs ~2.3 GB free VRAM).
- **Fast code search:** Use `rg` (respects root `.ignore`, excluding `venv/`, `archive/`, `logs/`, `runs/`, `openStreetUSD/`, `arcpro-rl-course/`, `.cs/`, and binary assets).

## Canonical Context & State

- **Active state & next steps:** `RESUME HERE.md` + ClearSet SQLite DBs in `.cs/cs_turns.db` and `.cs/cs_codebase.db` (`cs`, `cs-sync`, `cs-gate`).
- **Roadmap & active phase spec:** `.planning/ROADMAP.md` and `.planning/phases/M006-01-SPEC.md`.
- **RL failure-mode archaeology:** `.planning/reward_tuning_history.md` (consult before editing rewards, terminations, or curriculum spawns; verbatim Issues 1–120 archived in `archive/reward_tuning_history_full_issues_1_120.md`).
- **Deep RL debugging runbook:** `.agents/skills/rl-deep-audit/SKILL.md` (6-gate forensic audit; telemetry table contract-checked by `tests/test_agent_doc_contracts.py`).
- **Legacy directories (do not treat as active truth):** `archive/` (incl. `archive/legacy_gsd_202606_202609/` and `archive/legacy_planning_docs/`), `arcpro-rl-course/`, and `arcproLab/policy_stack/` (separate git submodule imported outside itself only by `arcproLab/mdp/go_signal_manager.py`).

## Non-Negotiable RL & Architecture Invariants

1. **RGB-only vision policy:** Never remove `--enable_cameras` from training or leak privileged map coordinates to `ARCProActor`. Populate all 12 slots in `arcproLab/mdp/observations.py` for the privileged Critic; mask `ACTOR_MASKED_TELEMETRY_INDICES` only on a `.clone()` inside `ARCProActor._mask_actor_telemetry()`.
2. **Single source of truth for track geometry & telemetry:** All corner/intersection predicates, `SECTOR_DEFS`, `TELEMETRY_NAMES`, and `ACTOR_MASKED_TELEMETRY_INDICES` live in `arcproLab/mdp/track_topology.py` with **zero** `isaaclab` or `omni` imports at top level. Never duplicate coordinate boxes in downstream modules.
3. **IsaacLab step execution order (`terminations -> rewards -> observations`):** `rewards.py` must never depend on `env.extras` keys written solely by `observations.py`. All shared road-state keys are computed once per step via `track_topology.compute_step_road_state()` during `terminations.white_line_contact()`.
4. **Replay-buffer non-aliasing:** `SKRLFlattenWrapper` must preserve its 2-slot ping-pong observation buffer (`s_t.data_ptr() != s_{t+1}.data_ptr()`).
5. **Sentinel vs. ClearSet separation:** `arcproLab/scripts/sentinel_utils.py` logs automated daemon events to `logs/sentinel_events.jsonl`—never into `.cs/cs_turns.db`.
6. **Grug simplicity & safety:** Prefer flat modules (`<500` LOC target) and `pytest -m cpu` gates over new daemons or heavy abstractions. Never kill live tmux `training` or modify git history without explicit user approval.
