#!/usr/bin/env python3
"""Full Monte Carlo grid on the headless stack (same design as
run_paper_experiments.m): E1 x E2 x E3 x E4 x E5 x E6, 5 seeds, ~250 runs.

Writes logs/paper/results_headless.csv with the same columns as the MATLAB
driver (plus a stack column). Deterministic: re-running reproduces it.
Resume-safe: runs already present in the output CSV are skipped, so an
interrupted batch can simply be relaunched with the same command.
"""
import argparse
import csv
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "release", "ros2_bundle", "ros2_swarm_transport")
sys.path.insert(0, os.path.abspath(PKG))

from ros2_swarm_transport.headless_runner import run_mission  # noqa: E402

WORLD = os.path.abspath(os.path.join(
    PKG, "worlds", "swarm_world_dynamic.sdf"))
CONTRACT = os.path.abspath(os.path.join(
    PKG, "worlds", "dynamic_obstacles.json"))

COLS = ["stack", "exp", "algo", "sense_kind", "sigma0", "beams", "seed",
        "world", "formation", "reached", "collisions", "path_length",
        "path_eff", "min_clear", "mean_clear", "total_time", "comp_ms_mean"]


def run_one(exp, algo, kind, sigma0, beams, seed, formation, spacing):
    t0 = time.time()
    rows, S = run_mission(WORLD, CONTRACT, algo=algo, sense=kind,
                          n_beams=beams, sigma0=sigma0, seed=seed,
                          grid_spacing=spacing, formation=formation,
                          verbose=False)
    p2 = [r for r in rows if r["phase"] == 2]
    mean_clr = (sum(r["clearance"] for r in p2) / len(p2)) if p2 else float("nan")
    metrics = {
        "stack": "ros2_headless", "exp": exp, "algo": algo, "sense_kind": kind,
        "sigma0": sigma0, "beams": beams, "seed": seed,
        "world": "swarm_world_dynamic.sdf", "formation": formation,
        "reached": 1 if S["reached_goal"] else 0,
        "collisions": S["collisions"], "path_length": round(S["path_length"], 4),
        "path_eff": round(S["path_efficiency"], 4),
        "min_clear": round(S["min_clearance"], 4),
        "mean_clear": round(mean_clr, 4),
        "total_time": round(S["total_time"], 2),
        "comp_ms_mean": round(S["comp_ms_mean"], 4),
    }
    return metrics, rows, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join(
        HERE, "logs", "paper", "results_headless.csv"))
    ap.add_argument("--seeds", type=int, nargs="*", default=[1, 2, 3, 4, 5])
    ap.add_argument("--save-runs", action="store_true",
                    help="also write per-step logs for overlays/paired figs")
    args = ap.parse_args()

    jobs = []
    # E1: 4 algos x seeds (lidar sigma0=0.02, 288 beams, square)
    for algo in ("apf", "vfh", "pvfh", "pvfh_prob"):
        for s in args.seeds:
            jobs.append(("E1", algo, "lidar", 0.02, 288, s, "square", 1.18))
    # E2: vfh + pvfh_prob x sigma sweep
    for algo in ("vfh", "pvfh_prob"):
        for sg in (0.02, 0.05, 0.10, 0.20, 0.40):
            for s in args.seeds:
                jobs.append(("E2", algo, "lidar", sg, 288, s, "square", 1.18))
    # E3: tof beam sweep (pvfh_prob)
    for nb in (288, 64, 16, 8):
        for s in args.seeds:
            kind = "lidar" if nb >= 288 else "tof"
            jobs.append(("E3", "pvfh_prob", kind, 0.10, nb, s, "square", 1.18))
    # E4: formation shapes (pvfh_prob; line needs reduced spacing)
    for form in ("square", "diamond", "triangle", "line"):
        for s in args.seeds:
            jobs.append(("E4", "pvfh_prob", "lidar", 0.02, 288, s, form, 0.60))
    # E5: crossed grid formation x algorithm (complete 4x4 matrix, s=0.60)
    for form in ("square", "diamond", "triangle", "line"):
        for algo in ("apf", "vfh", "pvfh", "pvfh_prob"):
            for s in args.seeds:
                jobs.append(("E5", algo, "lidar", 0.02, 288, s, form, 0.60))
    # E6: matched lidar vs ToF (sensor is the only factor; the lidar-288
    # rows re-run E2's precision/realistic cells so every E6 summary table
    # is self-contained with its own reference column)
    for kind, nb in (("lidar", 288), ("tof", 16), ("tof", 8)):
        for sg in (0.02, 0.10):
            for algo in ("vfh", "pvfh_prob"):
                for s in args.seeds:
                    jobs.append(("E6", algo, kind, sg, nb, s, "square", 1.18))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    run_dir = os.path.join(os.path.dirname(os.path.abspath(args.out)), "runs")
    if args.save_runs:
        os.makedirs(run_dir, exist_ok=True)
    # resume support: skip (exp, algo, sigma0, beams, seed, formation) rows
    # already present in the output CSV, so a killed batch can be relaunched
    done = set()
    if os.path.exists(args.out):
        with open(args.out) as fh:
            for row in csv.DictReader(fh):
                done.add((row["exp"], row["algo"], float(row["sigma0"]),
                          int(float(row["beams"])), int(float(row["seed"])),
                          row["formation"]))
    mode = "a" if (done and os.path.exists(args.out)) else "w"
    with open(args.out, mode, newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=COLS)
        if mode == "w":
            wr.writeheader()
        t_all = time.time()
        n_done = 0
        for i, (exp, algo, kind, sg, nb, s, form, sp) in enumerate(jobs, 1):
            if (exp, algo, float(sg), int(nb), int(s), form) in done:
                continue
            n_done += 1
            r, rows, wall = run_one(exp, algo, kind, sg, nb, s, form, sp)
            wr.writerow(r)
            fh.flush()
            print(f"[{i}/{len(jobs)}] {exp} {algo} sg={sg} b={nb} "
                  f"f={form} s={s}: reached={r['reached']} "
                  f"clr={r['min_clear']} ({wall:.0f}s)", flush=True)
            if args.save_runs:
                from ros2_swarm_transport.headless_runner import write_csv
                if exp == "E1":
                    name = f"E1_{algo}_s{s}.csv"
                elif exp == "E2":
                    name = f"E2_{algo}_sg{round(sg * 100):03d}_s{s}.csv"
                elif exp == "E3":
                    name = f"E3_b{nb:03d}_s{s}.csv"
                elif exp == "E4":
                    name = f"E4_{form}_s{s}.csv"
                elif exp == "E5":
                    name = f"E5_{algo}_{form}_s{s}.csv"
                else:
                    name = (f"E6_{algo}_{kind}{nb:03d}_"
                            f"sg{round(sg * 100):03d}_s{s}.csv")
                write_csv(rows, os.path.join(run_dir, name))
    print(f"wrote {args.out} ({n_done} new runs, {len(done)} skipped, "
          f"{time.time()-t_all:.0f}s)")


if __name__ == "__main__":
    main()
