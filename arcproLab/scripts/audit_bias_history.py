#!/usr/bin/env python3
"""Audit bias history across all clean baseline checkpoints."""

import glob
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checkpoint_probe import evaluate_checkpoint


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    pattern = os.path.join(root, "logs/ppo_skrl_rollback_clean-baseline/**/agent_*.pt")
    ckpts = glob.glob(pattern, recursive=True)

    def parse_step(p):
        m = re.search(r"agent_(\d+)\.pt", p)
        return int(m.group(1)) if m else -1

    clean_ckpts = [c for c in ckpts if "corrupted" not in c]
    clean_ckpts.sort(key=parse_step)

    step_map = {}
    for c in clean_ckpts:
        s = parse_step(c)
        if s not in step_map or os.path.getmtime(c) > os.path.getmtime(step_map[s]):
            step_map[s] = c

    steps = sorted(step_map.keys())
    print(
        f"Found {len(steps)} distinct checkpoint steps from {steps[0]} to {steps[-1]}"
    )

    results = []
    header = f"{'Step':>8} | {'Neutral Steer':>13} | {'Turn Steer':>11} | {'Delta Steer':>11} | {'Turn Throt':>11} | {'Sigma Steer':>11} | {'Sigma Throt':>11}"
    print(header)
    print("-" * len(header))

    for s in steps:
        p = step_map[s]
        res = evaluate_checkpoint(p)
        res["step"] = s
        results.append(res)
        print(
            f"{s:>8} | {res['mu_neutral_steer']:>+13.4f} | {res['mu_turn_steer']:>+11.4f} | "
            f"{res['delta_turn_steer']:>+11.4f} | {res['mu_turn_throt']:>+11.4f} | "
            f"{res['sigma_steer']:>11.4f} | {res['sigma_throt']:>11.4f}"
        )

    out_json = os.path.join(root, "logs/full_checkpoint_probe_history.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved history to {out_json}")


if __name__ == "__main__":
    main()
