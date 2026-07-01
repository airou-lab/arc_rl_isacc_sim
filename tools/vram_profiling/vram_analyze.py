#!/usr/bin/env python3
"""vram_analyze.py — turn the sidecar CSVs into proposal-ready numbers.
  peak <gpu.csv> [procs.csv]                        report peak VRAM
  extrapolate --point N MiB [...] [--target 16]     fit fixed+per-env, project
"""
import argparse, csv, glob, sys

def _read_col(path, colname):
    vals = []
    with open(path, newline="") as f:
        reader = csv.reader(f); header = next(reader, None)
        if header is None: return [], None
        idx = None; norm = [h.strip().lower() for h in header]
        for i, h in enumerate(norm):
            if colname in h: idx = i; break
        if idx is None: return [], header
        for row in reader:
            if len(row) <= idx: continue
            cell = row[idx].strip()
            if not cell or cell.lower() in ("[n/a]", "n/a", "-"): continue
            try: vals.append(float(cell))
            except ValueError: continue
    return vals, header

def cmd_peak(args):
    gpu_path = (sorted(glob.glob(args.gpu_csv)) or [args.gpu_csv])[0]
    used, _ = _read_col(gpu_path, "memory.used")
    if not used:
        print(f"No memory.used samples in {gpu_path}", file=sys.stderr); return 1
    peak = max(used)
    print(f"Whole-GPU peak : {peak:8.0f} MiB  ({peak/1024:.2f} GiB)   [{gpu_path}]")
    print(f"Whole-GPU mean : {sum(used)/len(used):8.0f} MiB  ({len(used)} samples)")
    if args.procs_csv:
        proc_path = (sorted(glob.glob(args.procs_csv)) or [args.procs_csv])[0]
        pused, _ = _read_col(proc_path, "gpu_memory")
        if pused:
            ppeak = max(pused)
            print(f"Top-process peak: {ppeak:8.0f} MiB  ({ppeak/1024:.2f} GiB)   [{proc_path}]")
            print("  ^ use this as the headline if the GPU was NOT exclusive.")
        else:
            print(f"(no per-process samples parsed from {proc_path})")
    return 0

def _line_from_two(p):
    (n1, m1), (n2, m2) = p[0], p[1]
    per_env = (m2 - m1) / (n2 - n1); fixed = m1 - per_env * n1
    return fixed, per_env, None

def _least_squares(points):
    k = len(points); sx = sum(n for n,_ in points); sy = sum(m for _,m in points)
    sxx = sum(n*n for n,_ in points); sxy = sum(n*m for n,m in points)
    slope = (k*sxy - sx*sy) / (k*sxx - sx*sx); intercept = (sy - slope*sx)/k
    ybar = sy/k; ss_tot = sum((m-ybar)**2 for _,m in points)
    ss_res = sum((m-(intercept+slope*n))**2 for n,m in points)
    r2 = 1 - ss_res/ss_tot if ss_tot > 0 else 1.0
    return intercept, slope, r2

def cmd_extrapolate(args):
    if not args.point or len(args.point) < 2:
        print("Need at least two --point N MiB pairs.", file=sys.stderr); return 1
    points = sorted((int(n), float(m)) for n, m in args.point)
    if len(points) >= 3:
        fixed, per_env, r2 = _least_squares(points)
        fit = f"least-squares over {len(points)} points (R^2={r2:.4f})"
    else:
        fixed, per_env, _ = _line_from_two(points)
        fit = "two-point line (linearity UNVERIFIED)"
    target = args.target; est = fixed + per_env*target; est_hr = est*(1+args.headroom)
    max_n = max(n for n,_ in points)
    print(f"Fit method     : {fit}")
    print(f"Fixed cost     : {fixed:8.0f} MiB  ({fixed/1024:.2f} GiB)")
    print(f"Per-env cost   : {per_env:8.0f} MiB/env ({per_env/1024:.2f} GiB/env)")
    print("-"*56)
    for n, m in points:
        print(f"  measured n={n:<3d}: {m:8.0f} MiB  ({m/1024:.2f} GiB)  resid {m-(fixed+per_env*n):+6.0f}")
    print("-"*56)
    print(f"Estimate n={target:<3d}: {est:8.0f} MiB  ({est/1024:.2f} GiB)")
    print(f"  + {int(args.headroom*100)}% headroom: {est_hr:8.0f} MiB  ({est_hr/1024:.2f} GiB)")
    print()
    for label, cap in (("48 GB (RTX 6000 Ada / L40S)", 48), ("80 GB (A100 / H100)", 80)):
        verdict = "fits" if est_hr <= cap*1024 else "DOES NOT fit"
        print(f"  {label:<32s}: {verdict} ({est_hr/1024:.1f} / {cap} GiB w/ headroom)")
    if len(points) < 3: print("\nNOTE: two points cannot reveal non-linearity. Add a 3rd point.")
    if target > 2*max_n:
        print(f"\nNOTE: estimating n={target} from points up to n={max_n} is a {target/max_n:.1f}x extrapolation.")
    return 0

def main():
    p = argparse.ArgumentParser(description="Analyze sidecar VRAM CSVs.")
    sub = p.add_subparsers(dest="cmd", required=True)
    pk = sub.add_parser("peak"); pk.add_argument("gpu_csv")
    pk.add_argument("procs_csv", nargs="?", default=None); pk.set_defaults(func=cmd_peak)
    ex = sub.add_parser("extrapolate")
    ex.add_argument("--point", nargs=2, action="append", metavar=("N","MiB"))
    ex.add_argument("--target", type=int, default=16)
    ex.add_argument("--headroom", type=float, default=0.20); ex.set_defaults(func=cmd_extrapolate)
    args = p.parse_args(); sys.exit(args.func(args))

if __name__ == "__main__":
    main()
