# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Phase 2 centerline generator (v2). See docs/off-road-debug/05-phase2-centerline-investigation.md
and 06-phase2-centerline-regeneration.md.

Runs ENTIRELY OFFLINE -- pure numpy/scipy against the cached boundary point
cloud (arcproLab/mdp/track_boundaries_1x.npz). No Isaac Sim boot required,
which makes iterating on the track reference fast to debug.

Why this replaces fix_centerline.py / generate_centerline.py
------------------------------------------------------------
Those pair each yellow point with its nearest white point, take the midpoint,
and greedily chain. In dense-marking regions a yellow point's nearest white
neighbour is often NOT the opposite boundary of its own lane, so the
"midpoint" is not a lane center. That is a structural flaw, not a tuning
problem -- it is why the shipped file has 178 fragments (median fragment 2
waypoints) and 29.3% physically-impossible curvature.

This generator instead runs a VIRTUAL LANE-FOLLOWER: it walks forward in
small steps and servos its lateral offset against the yellow centre divider,
exactly like a car tracking a lane. Curves are handled naturally, and short
gaps in the paint (intersections) are coasted through on heading.

Pipeline: trace -> uniform arclength resample -> periodic smooth ->
          yaw by central difference over a window -> curvature.

Deriving yaw over a ~0.7m baseline instead of between adjacent ~1cm points is
what removes the noise: the old file's yaw came from 1mm-quantized points 1cm
apart, so kappa = dyaw/ds amplified quantization noise ~100x.
"""
import argparse
import os
import shutil

import numpy as np
from scipy.spatial import cKDTree

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MDP_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "mdp"))

# --- tuning (see 06-...md for the sweep these came from) ---
GROUND_Z = 0.10        # strip vertical structures caught by the paint name-filter
TARGET_OFFSET = -0.208 # metres right of the yellow line = right-lane center
STEP = 0.05            # tracer step
GAIN = 0.35            # lateral servo gain
HEADING_SMOOTH = 0.30
SPACING = 0.10         # output waypoint spacing
SMOOTH_W = 5           # periodic moving-average window
YAW_W = 7              # +/- samples for the yaw central difference (0.7m baseline)
MAX_PHYSICAL_KAPPA = 1.66  # 1 / (wheelbase/tan(max_steer)) = 1/0.604


def perp_left(h):
    return np.array([-h[1], h[0]])


def trace(ytree, yellow, start, heading, target_off, close_r=0.35,
          min_len_close=8.0, search_r=1.2, max_steps=200000, lost_after_m=12.0):
    p = start.astype(float).copy()
    h = heading / np.linalg.norm(heading)
    path = [p.copy()]
    travelled = 0.0
    no_ref_m = 0.0
    for _ in range(max_steps):
        p_new = p + STEP * h
        lat = perp_left(h)
        dist, idx = ytree.query(p_new, distance_upper_bound=search_r)
        if np.isfinite(dist):
            cur_off = float(np.dot(p_new - yellow[idx], lat))
            p_new = p_new + lat * (target_off - cur_off) * GAIN
            no_ref_m = 0.0
        else:
            no_ref_m += STEP
            if no_ref_m > lost_after_m:
                return np.array(path), False, travelled
        v = p_new - p
        n = np.linalg.norm(v)
        if n < 1e-9:
            return np.array(path), False, travelled
        h = (1 - HEADING_SMOOTH) * h + HEADING_SMOOTH * (v / n)
        h /= np.linalg.norm(h)
        p = p_new
        path.append(p.copy())
        travelled += n
        if travelled > min_len_close and np.linalg.norm(p - start) < close_r:
            return np.array(path), True, travelled
    return np.array(path), False, travelled


def resample_closed(path, spacing):
    p = np.vstack([path, path[0]])
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = int(round(total / spacing))
    s_new = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(s_new, s, p[:, 0]),
                            np.interp(s_new, s, p[:, 1])]), total


def smooth_closed(p, w):
    if w <= 1:
        return p
    k = np.ones(w) / w
    x = np.convolve(np.r_[p[-w:, 0], p[:, 0], p[:w, 0]], k, mode="same")[w:-w]
    y = np.convolve(np.r_[p[-w:, 1], p[:, 1], p[:w, 1]], k, mode="same")[w:-w]
    return np.column_stack([x, y])


def yaw_kappa_closed(p, spacing, w):
    n = len(p)
    i = np.arange(n)
    ip, im = (i + w) % n, (i - w) % n
    yaw = np.arctan2(p[ip, 1] - p[im, 1], p[ip, 0] - p[im, 0])
    yp, ym = yaw[(i + 1) % n], yaw[(i - 1) % n]
    dyaw = np.arctan2(np.sin(yp - ym), np.cos(yp - ym))
    return yaw, dyaw / (2 * spacing)


def main():
    ap = argparse.ArgumentParser(description="Regenerate track_centerline.npy (offline).")
    ap.add_argument("--boundaries", default=os.path.join(MDP_DIR, "track_boundaries_1x.npz"))
    ap.add_argument("--out", default=os.path.join(MDP_DIR, "track_centerline.npy"))
    ap.add_argument("--spawn", type=float, nargs=2, default=[-16.197, 5.50],
                    help="Trace start; must lie on the lane center (matches arcpro_env_cfg spawn).")
    ap.add_argument("--heading", type=float, nargs=2, default=[0.0, -1.0],
                    help="Initial heading. Spawn quat (0.7071,0,0,0.7071)=yaw 90deg and the "
                         "chassis' local -X forward => world -Y.")
    ap.add_argument("--dry-run", action="store_true", help="Validate only; do not write.")
    args = ap.parse_args()

    d = np.load(args.boundaries)
    white = d["white"][d["white"][:, 2] < GROUND_Z][:, :2]
    yellow = d["yellow"][d["yellow"][:, 2] < GROUND_Z][:, :2]
    print(f"[data] ground-level white {len(white)}/{len(d['white'])}  "
          f"yellow {len(yellow)}/{len(d['yellow'])}  (Z<{GROUND_Z} filter)")

    ytree, wtree = cKDTree(yellow), cKDTree(white)
    start = np.array(args.spawn, dtype=float)
    dy0, _ = ytree.query(start)
    dw0, _ = wtree.query(start)
    print(f"[data] start {start.tolist()}: yellow {dy0:.3f}m white {dw0:.3f}m "
          f"-> lane half-width ~{(dy0+dw0)/2:.3f}m")

    raw, closed, travelled = trace(ytree, yellow, start, np.array(args.heading, float), TARGET_OFFSET)
    print(f"[trace] {len(raw)} pts, {travelled:.2f}m, loop_closed={closed}")
    if not closed:
        print("[trace] ERROR: did not close a loop. The windowed search in "
              "TrackManager.compute_errors assumes a continuous route; refusing to write.")
        return 1

    rs, total = resample_closed(raw, SPACING)
    sm = smooth_closed(rs, SMOOTH_W)
    yaw, kappa = yaw_kappa_closed(sm, SPACING, YAW_W)
    out = np.column_stack([sm[:, 0], sm[:, 1], yaw])

    # ---------------- validation ----------------
    seg = np.linalg.norm(np.diff(np.vstack([sm, sm[0]]), axis=0), axis=1)
    dy_, _ = ytree.query(sm)
    dw_, _ = wtree.query(sm)
    ak = np.abs(kappa)
    frag = int((seg > SPACING * 1.5).sum())
    plaus = 100.0 * (ak <= MAX_PHYSICAL_KAPPA).mean()

    print(f"\n{'='*66}\nVALIDATION\n{'='*66}")
    print(f"waypoints            : {len(out)}  (loop {total:.2f}m @ {SPACING}m)")
    print(f"spacing min/max      : {seg.min():.4f} / {seg.max():.4f} m")
    print(f"fragments (>1.5x gap): {frag}")
    print(f"dist to yellow       : med {np.median(dy_):.3f}  p95 {np.percentile(dy_,95):.3f} m")
    print(f"dist to white        : med {np.median(dw_):.3f}  p95 {np.percentile(dw_,95):.3f} m")
    print(f"|kappa| max          : {ak.max():.3f}  (min turn radius {1/max(ak.max(),1e-9):.2f} m)")
    print(f"physically plausible : {plaus:.1f}%  (limit |k|<={MAX_PHYSICAL_KAPPA})")

    checks = {
        "loop closed": closed,
        "zero fragments": frag == 0,
        "uniform spacing": seg.max() <= SPACING * 1.5,
        "centered on yellow (0.15-0.28m)": 0.15 <= np.median(dy_) <= 0.28,
        "centered on white (0.15-0.28m)": 0.15 <= np.median(dw_) <= 0.28,
        "100% physically plausible": plaus >= 99.999,
    }
    print()
    ok = True
    for name, passed in checks.items():
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
        ok &= passed

    if not ok:
        print("\n[FAIL] validation failed; refusing to write.")
        return 1
    print("\n[PASS] all validation checks passed.")

    if args.dry_run:
        print("[dry-run] not writing.")
        return 0

    if os.path.exists(args.out):
        backup = args.out + ".pre_v2.bak"
        if not os.path.exists(backup):
            shutil.copy2(args.out, backup)
            print(f"[backup] previous centerline -> {backup}")
    np.save(args.out, out)
    print(f"[write] {args.out}  shape={out.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
