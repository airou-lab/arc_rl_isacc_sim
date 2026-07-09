#!/usr/bin/env python3
"""Wire StoplightVisual into RoadManager (run AFTER apply_v2i_splice.py).

Anchors target the post-splice road_manager.py. Aborts, writing nothing,
if any anchor is missing.

  __init__            : self.stoplight = None
  _initialize_gates   : record gate prim paths; build stoplights
  update              : sync lamp visibility on phase transitions
"""

import sys

PATH = "arcproLab/mdp/road_manager.py"

EDITS = [
    (
        "        self.time_to_change = torch.zeros((num_envs, num_agents), device=self.device)\n",
        "        self.time_to_change = torch.zeros((num_envs, num_agents), device=self.device)\n"
        "        self.stoplight = None  # built during gate discovery (needs stage)\n",
    ),
    (
        "        gate_positions = []\n"
        "        gate_intents = []\n",
        "        gate_positions = []\n"
        "        gate_intents = []\n"
        "        gate_records = []  # (prim_path, (x, y, z)) for stoplight placement\n",
    ),
    (
        "                gate_positions.append([pos[0], pos[1]])\n"
        "                gate_intents.append(intent)\n",
        "                gate_positions.append([pos[0], pos[1]])\n"
        "                gate_intents.append(intent)\n"
        "                gate_records.append((str(prim.GetPath()), (float(pos[0]), float(pos[1]), float(pos[2]))))\n",
    ),
    (
        "            print(f\"[RoadManager] Discovered {len(gate_positions)} navigation gates.\")\n",
        "            print(f\"[RoadManager] Discovered {len(gate_positions)} navigation gates.\")\n"
        "            try:\n"
        "                from mdp.stoplight_visual import StoplightVisual\n"
        "            except ImportError:\n"
        "                from stoplight_visual import StoplightVisual\n"
        "            self.stoplight = StoplightVisual()\n"
        "            self.stoplight.build(stage, gate_records)\n",
    ),
    (
        "            go, ttc, _ = self.intersection.get_spat(self.agent_groups)\n"
        "            self.go_signals = go\n"
        "            self.time_to_change = ttc / self.intersection.cycle_s\n",
        "            go, ttc, phase = self.intersection.get_spat(self.agent_groups)\n"
        "            self.go_signals = go\n"
        "            self.time_to_change = ttc / self.intersection.cycle_s\n"
        "            if self.stoplight is not None:\n"
        "                self.stoplight.sync(phase[:, 0])\n",
    ),
]


def main() -> int:
    try:
        text = open(PATH).read()
    except FileNotFoundError:
        print(f"ABORTED: {PATH} not found (run from repo root)")
        return 1

    failures = []
    for i, (old, new) in enumerate(EDITS, 1):
        n = text.count(old)
        if n != 1:
            failures.append(f"edit {i}: anchor found {n} times (need exactly 1)")
            continue
        text = text.replace(old, new)

    if failures:
        print("STOPLIGHT SPLICE ABORTED — nothing written:")
        for f in failures:
            print("  -", f)
        print("(Did apply_v2i_splice.py run first?)")
        return 1

    open(PATH, "w").write(text)
    print(f"patched: {PATH}")
    print("Run: python3 -m py_compile " + PATH)
    return 0


if __name__ == "__main__":
    sys.exit(main())
