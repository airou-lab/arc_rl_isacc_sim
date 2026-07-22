# Phase 0: baseline measurement (read-only)

**Branch:** `off-road-debug` (after commit 115c49a)
**Probe:** `arcproLab/scripts/probe_baseline.py` — changes no behavior
**Run:** headless, `num_envs=1`, no camera, 400-step forward rollout @ throttle 0.5
**Purpose:** capture "before" numbers so later fixes are validated, not asserted.

Two hypotheses had already died against data before this (gate conflation,
gate radius). This harness exists so we stop reasoning from code-reading
alone.

## Config confirmed

```
sim.dt=0.002  decimation=10  -> control_dt=0.0200s (50.0 Hz)
episode_length_s=300.0        -> 15000 control steps/episode
termination terms: ['roadmark_contact', 'height', 'stagnation', 'driving_blind']
```

## Part B — steering kinematics: HEALTHY (hypothesis ruled out)

| cmd | exp_L | exp_R | proc_L | proc_R | real_L | real_R | errL | errR |
|---|---|---|---|---|---|---|---|---|
| -1.00 | -0.6181 | -0.4174 | -0.6181 | -0.4174 | -0.6233 | -0.4159 | -0.0051 | 0.0015 |
| -0.50 | -0.2789 | -0.2264 | -0.2789 | -0.2264 | -0.2729 | -0.2241 | 0.0060 | 0.0023 |
| 0.00 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0011 | 0.0065 | 0.0011 | 0.0065 |
| 0.50 | 0.2264 | 0.2789 | 0.2264 | 0.2789 | 0.2268 | 0.2889 | 0.0003 | 0.0100 |
| 1.00 | 0.4174 | 0.6181 | 0.4174 | 0.6181 | 0.4192 | 0.6246 | 0.0017 | 0.0064 |

The analytic Ackermann expectation matches `processed_actions` **exactly**,
and the articulation tracks it to within ~0.01 rad (0.6°). **The policy
output -> joint movement mapping is correct.** This retires the original
"expected vs real value" concern for steering — it is not the bug.

**But** note `errR` is positive at *every* command including zero
(real_R = +0.0065 rad with a 0.0 command), while `errL` scatters around
zero. A persistent right-knuckle offset under load is the most likely
mechanism for the drift measured in Part C. Consistent with finite joint
stiffness (20 N·m/rad) working against ground reaction torque, with
asymmetric loading between the two sides.

### Drive actuator: torque-limited, slow to accelerate

```
throttle=0.25 -> commanded -10.000 rad/s | actual  -3.415 rad/s
throttle=0.50 -> commanded -20.000 rad/s | actual  -6.908 rad/s
throttle=1.00 -> commanded -40.000 rad/s | actual -10.429 rad/s
```

These samples are **transient, not steady-state** (only 40 steps = 0.8s per
setting), so the ratios overstate the problem — in the 400-step rollout the
car did reach 0.916 m/s against a 1.0 m/s target (92%) at throttle 0.5.
The real finding is that acceleration is slow: **~4 seconds (200 control
steps) to reach 0.9 m/s**, because the drive joints have an **effort limit
of 0.5 N·m** (per the articulation dump). Not necessarily a defect, but it
means `speed_reward` cannot be satisfied quickly, and any policy is
strongly rate-limited in how fast it can respond.

## Part A — observation slot integrity

```
slot                          min          max        range
0 turn_token               0.0000       0.0000       0.0000   <-- constant this run
1 go_signal                1.0000       1.0000       0.0000   <-- DEAD (hardcoded)
2 goal_dist                0.0000       0.0000       0.0000   <-- DEAD (hardcoded)
3 speed                    0.0044       0.9193       0.9149   ok
4 yaw_rate                -0.0355       0.0547       0.0902   ok
5 act_steer                0.0000       0.0000       0.0000   constant (probe artifact)
6 act_throttle             0.5000       0.5000       0.0000   constant (probe artifact)
7 act_brake                0.0000       0.0000       0.0000   constant (probe artifact)
8 lat_err(masked)          0.0000       0.0000       0.0000   DEAD (intentional)
9 head_err(masked)         0.0000       0.0000       0.0000   DEAD (intentional)
10 kappa                   0.0000       0.0000       0.0000   see below
11 distance              -15.2559      -1.6233      13.6325   BROKEN
```

Honest reading of each:

- **Slots 5/6/7 constant is a probe artifact**, not a bug — the probe feeds
  a constant action, so "last action" is correctly constant.
- **Slot 0 was constant 0.0 here only because there were zero episode
  resets** in this run (Part D). The claim that it re-randomizes per
  episode is *not* confirmed by this run and still needs a multi-episode
  probe.
- **Slot 10 (kappa) reading 0.000 throughout CONTRADICTS my prediction.**
  I expected garbage. In fact the spawn region is a genuinely straight
  section of centerline, so 0.0 is *correct* here. The ±1577 garbage
  measured offline is real but lives at the 178 fragment discontinuities
  elsewhere in the file — it does not manifest until the car reaches one.
  This downgrades kappa from "actively poisoning every step" to "a
  landmine that fires at fragment boundaries." Prediction was wrong;
  recorded as such.
- **Slot 11 (distance) — both predicted defects confirmed exactly:**
  - **Sign:** goes *more negative* (-1.62 -> -15.26) while driving
    **forward**. Confirmed inverted.
  - **Scale:** accumulated 13.63 units while true displacement was
    5.44 m. Ratio = **2.505**, matching the predicted 2.5x
    (`0.05` hardcoded vs true control dt `0.02`) to three digits.
  - Also unbounded/monotonic — a non-stationary feature under
    `VecNormalize`.

## Part C — straight-line drift: CONFIRMED

```
displacement along local -X : 5.4433 m
displacement lateral        : -0.0762 m
```

- Positive "along local -X" confirms **local -X is forward**, matching
  `speed_reward`'s assumption. Note the spawn comment in
  `arcpro_env_cfg.py` says "Face North", but with yaw=90° and -X forward
  the car actually travels toward **-Y (South)**. The physics is
  self-consistent; the comment is misleading.
- **Lateral drift: 7.6 cm over 5.44 m (~1.4%) at 0.0 steering.** The
  progression (0.0000, -0.0002, -0.0024, -0.0114, -0.0274, -0.0496) is
  *accelerating*, i.e. a curved path from a small steady yaw bias rather
  than a one-off impulse. This **confirms and quantifies** the open
  `TODO.md` item ("slight left-drift bias when driving perfectly straight
  with 0.0 steering"), and Part B's persistent `errR` offset is the
  leading candidate mechanism.

## Part D — termination causes

```
total episode ends in 400 steps: 0
  roadmark_contact 0 | height 0 | stagnation 0 | driving_blind 0
```

No terminations in 5.4 m of travel. **We still have not observed the
actual road-departure failure.** Reproducing it needs either a trained
policy (none exists — `logs/ppo` is empty, no SB3 `.zip` checkpoint
anywhere) or a much longer rollout that reaches the first curve. Per-term
attribution is now wired and will pay off the moment a real episode ends.

## Net effect on the issue list

| Issue | Status after Phase 0 |
|---|---|
| Steering command -> joint mapping | **RULED OUT** — matches analytically, tracks to 0.6° |
| Slot 11 distance sign + 2.5x scale | **CONFIRMED** empirically, exactly as predicted |
| Left-drift bias at 0.0 steering | **CONFIRMED**, quantified at 7.6cm/5.4m, mechanism identified |
| Slot 10 kappa | **DOWNGRADED** — correct near spawn; a landmine at fragment boundaries, not a per-step poison |
| Slot 0 turn_token noise | **UNPROVEN** — needs a multi-episode run |
| Dead slots 1, 2 | Confirmed hardcoded constants |
| Drive effort limit 0.5 N·m | New observation — ~4s to reach 0.9 m/s |
| Reward gap (no centering incentive) | Still the leading suspect; untested (needs a policy) |
