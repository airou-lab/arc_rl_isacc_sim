#!/usr/bin/env python3
"""Splice the V2I intersection into the training pipeline (phase A, oracle comms).

Three files, exact-match replacements; aborts with a clear message if any
anchor is missing (i.e. the file differs from origin/main + additive branch).

  road_manager.py    : instantiate IntersectionManager; drive go_signals + ttc
  observations.py    : slot 2 = normalized permission horizon; publish
                       env.extras["go_signal"] (activates gated rewards)
  arcpro_env_cfg.py  : swap speed/stationary to gated variants; add
                       hold_at_red and red_light terms

Slot semantics follow the existing contract: slot 1 was already go_signal,
slot 2 was already reserved (IDX_GOAL_DIST, zeroed). Parameterized so a
post-sync slot change is a one-line edit here.
"""

import sys

EDITS = {
    "arcproLab/mdp/road_manager.py": [
        (
            "import torch\nimport numpy as np\n",
            "import torch\nimport numpy as np\n\n"
            "try:\n"
            "    from mdp.intersection_manager import IntersectionManager\n"
            "except ImportError:\n"
            "    from intersection_manager import IntersectionManager\n",
        ),
        (
            "        self.turn_tokens = torch.zeros((num_envs, num_agents), device=self.device)\n"
            "        self.go_signals = torch.ones((num_envs, num_agents), device=self.device)\n",
            "        self.turn_tokens = torch.zeros((num_envs, num_agents), device=self.device)\n"
            "        self.go_signals = torch.ones((num_envs, num_agents), device=self.device)\n"
            "\n"
            "        # V2I smart intersection (signalized authority, oracle comms).\n"
            "        # Set to None to restore legacy always-green behavior.\n"
            "        self.intersection = IntersectionManager(\n"
            "            num_envs=num_envs,\n"
            "            green_s=(8.0, 8.0),\n"
            "            yellow_s=2.0,\n"
            "            max_green_s=20.0,\n"
            "            step_dt=0.02,  # control rate: sim dt 0.002 * decimation 10\n"
            "            randomize_phase=True,\n"
            "            device=self.device,\n"
            "        )\n"
            "        # TODO(sync): derive approach groups from spawn/route geometry.\n"
            "        self.agent_groups = torch.zeros((num_envs, num_agents), dtype=torch.long, device=self.device)\n"
            "        self.time_to_change = torch.zeros((num_envs, num_agents), device=self.device)\n",
        ),
        (
            "        # Future: FIFO Queue logic for intersections (Milestone 4)\n"
            "        self.go_signals[:] = 1.0\n",
            "        # V2I smart intersection (Milestone 4): signalized authority.\n"
            "        if self.intersection is not None:\n"
            "            if reset_buf is not None and reset_buf.any():\n"
            "                self.intersection.reset(reset_buf)\n"
            "            self.intersection.step()\n"
            "            go, ttc, _ = self.intersection.get_spat(self.agent_groups)\n"
            "            self.go_signals = go\n"
            "            self.time_to_change = ttc / self.intersection.cycle_s\n"
            "        else:\n"
            "            self.go_signals[:] = 1.0\n",
        ),
    ],
    "arcproLab/mdp/observations.py": [
        (
            "    obs[:, 0] = turn_tokens.view(-1)\n"
            "    obs[:, 1] = go_signals.view(-1)\n"
            "    obs[:, 2] = 0.0  # IDX_GOAL_DIST\n",
            "    obs[:, 0] = turn_tokens.view(-1)\n"
            "    obs[:, 1] = go_signals.view(-1)\n"
            "    # Index 2: V2I permission horizon (time to next signal change, /cycle)\n"
            "    obs[:, 2] = rm.time_to_change.view(-1)\n"
            "    # Publish for reward terms (mdp.intersection_rewards reads extras only)\n"
            "    env.extras[\"go_signal\"] = go_signals.view(-1)\n",
        ),
    ],
    "arcproLab/arcpro_env_cfg.py": [
        (
            "import mdp.observations as mdp_obs, mdp.rewards as mdp_rew, mdp.terminations as mdp_done, mdp.events as mdp_events\n",
            "import mdp.observations as mdp_obs, mdp.rewards as mdp_rew, mdp.terminations as mdp_done, mdp.events as mdp_events\n"
            "import mdp.intersection_rewards as mdp_ir\n",
        ),
        (
            "    speed = RewTerm(func=mdp_rew.speed_reward, weight=20.0)\n",
            "    # Gated on go_signal: forward progress earns nothing while approaching a red\n"
            "    speed = RewTerm(func=mdp_ir.gated_speed_reward, weight=20.0)\n",
        ),
        (
            "    stationary = RewTerm(\n"
            "        func=lambda env: torch.where(torch.norm(env.scene[\"robot\"].data.root_lin_vel_b[:, :2], dim=1) < 0.1, -1.0, 0.0),\n"
            "        weight=5.0\n"
            "    )\n",
            "    stationary = RewTerm(func=mdp_ir.gated_stationary_penalty, weight=5.0)\n"
            "    # V2I intersection compliance\n"
            "    hold_at_red = RewTerm(func=mdp_ir.hold_at_red_reward, weight=10.0)\n"
            "    red_light = RewTerm(func=mdp_ir.red_light_violation, weight=100.0)\n",
        ),
    ],
}


def main() -> int:
    failures = []
    staged = {}
    for path, edits in EDITS.items():
        try:
            text = open(path).read()
        except FileNotFoundError:
            failures.append(f"{path}: file not found (run from repo root)")
            continue
        for i, (old, new) in enumerate(edits, 1):
            n = text.count(old)
            if n != 1:
                failures.append(f"{path}: edit {i} anchor found {n} times (need exactly 1)")
                continue
            text = text.replace(old, new)
        staged[path] = text

    if failures:
        print("SPLICE ABORTED — nothing written:")
        for f in failures:
            print("  -", f)
        return 1

    for path, text in staged.items():
        open(path, "w").write(text)
        print(f"patched: {path}")
    print("Splice applied. Run: python3 -m py_compile " + " ".join(EDITS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
