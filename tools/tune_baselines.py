#!/usr/bin/env python3
"""Equal-budget parameter tuning of every steering law (step 4 of the review).

    python3 tools/tune_baselines.py --jobs 4             # all laws
    python3 tools/tune_baselines.py --only dwa mppi

Protocol (identical for every law, the proposal included):
  * search space: the law's own knobs, ranges in SPACES below;
  * budget: N_CONFIGS configurations, the published/default values always
    being configuration 0, the rest a seeded random design;
  * training missions: the TRAIN worlds of the learned policy (rand01..05,
    swarm_world_dynamic, plaza) x 1 seed (201..207), randomised traffic
    phase, sigma0 = 0.05; the same missions for every configuration and law
    (common random numbers);
  * objective per mission: completion time (300 s if the goal is not
    reached) + 20 x mover/static contact time (s) + 1 x rejected-step time (s);
    the configuration with the lowest mean wins;
  * evaluation of default vs tuned happens ELSEWHERE (E18), on held-out
    worlds and seeds the tuner never saw.

Outputs: results/tuning/<law>.csv (every configuration and its score) and
results/tuning/best_params.json.
"""
import argparse
import csv
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
W = os.path.join(PKG, "worlds")
OUT = os.path.join(ROOT, "results", "tuning")
sys.path.insert(0, PKG)

TRAIN = [f"rand{k:02d}" for k in range(1, 6)] + ["dynamic", "plaza"]
N_CONFIGS = 24

# (name, low, high, kind) ; kind: f = float, i = int, c = choice list in low
SPACES = {
    "vfh": {"_root": [("react_gap", 0.4, 2.0, "f"), ("threshold", 0.15, 0.5, "f"),
                      ("hysteresis_weight", 0.2, 1.2, "f"), ("smooth_window", [3, 5, 7], None, "c"),
                      ("danger_dist_base", 0.6, 2.0, "f")]},
    "pvfh_prob": {"_root": [("react_gap", 0.4, 2.0, "f"), ("threshold", 0.15, 0.5, "f"),
                            ("hysteresis_weight", 0.2, 1.2, "f"), ("smooth_window", [3, 5, 7], None, "c"),
                            ("danger_dist_base", 0.6, 2.0, "f"), ("rfilter_q", 0.03, 0.2, "f")]},
    "pavfh": {"_root": [("react_gap", 0.4, 2.0, "f"), ("threshold", 0.15, 0.5, "f"),
                        ("hysteresis_weight", 0.2, 1.2, "f"), ("smooth_window", [3, 5, 7], None, "c"),
                        ("danger_dist_base", 0.6, 2.0, "f"), ("rfilter_q", 0.03, 0.2, "f")]},
    "fgm": {"_root": [("fgm_alpha", 0.3, 6.0, "f"), ("react_gap", 0.4, 2.0, "f")]},
    "dwa": {"dwa": [("w_max", 0.6, 2.5, "f"), ("T", 1.0, 3.0, "f"), ("a_h", 0.3, 1.0, "f"),
                    ("a_c", 0.05, 0.8, "f"), ("tau", 0.5, 1.5, "f"), ("clr_cap", 0.5, 2.0, "f"),
                    ("T_adm", 0.6, 1.6, "f")]},
    "mpc": {"mpc": [("seg_T", 0.5, 1.2, "f"), ("d_safe", 0.2, 0.9, "f"), ("w_obs", 1.0, 12.0, "f"),
                    ("lev", 0.6, 2.0, "f"), ("T_hard", 0.6, 1.6, "f"), ("tau", 0.5, 1.5, "f")]},
    "mppi": {"mppi": [("sigma_w", 0.3, 1.5, "f"), ("lam", 0.05, 1.0, "f"), ("T", 1.4, 3.6, "f"),
                      ("d_safe", 0.2, 0.9, "f"), ("w_obs", 1.0, 15.0, "f"), ("w_head", 0.0, 1.0, "f"),
                      ("tau", 0.5, 1.5, "f")]},
}


def paths(w):
    return (os.path.join(W, f"swarm_world_{w}.sdf"),
            os.path.join(W, "dynamic_obstacles.json" if w == "dynamic"
                         else f"dynamic_obstacles_{w}.json"))


def to_vfh_over(algo, flat):
    """Flat sampled dict -> run_mission(vfh_over=...) kwargs."""
    if not flat:
        return {}
    group = next(iter(SPACES[algo]))
    if group == "_root":
        return dict(flat)
    d = dict(flat)
    if algo == "mpc" and "lev" in d:
        s = d.pop("lev")
        d["levels"] = tuple(np.round(np.array([-1.2, -0.6, 0.0, 0.6, 1.2]) * s, 4))
    return {group: d}


def sample_configs(algo, n, seed=0):
    rng = np.random.default_rng([seed, sum(map(ord, algo))])
    space = next(iter(SPACES[algo].values()))
    out = [{}]                                  # configuration 0: defaults
    for _ in range(n - 1):
        c = {}
        for name, lo, hi, kind in space:
            if kind == "c":
                c[name] = lo[int(rng.integers(len(lo)))]
            elif kind == "i":
                c[name] = int(rng.integers(lo, hi + 1))
            else:
                c[name] = round(float(rng.uniform(lo, hi)), 4)
        out.append(c)
    return out


def mission_cost(S):
    t = S["total_time"] if S["reached_goal"] else 300.0
    return t + 20.0 * 0.1 * S["collisions"] + 1.0 * 0.1 * S["n_reject"]


def eval_one(args):
    algo, ci, flat, w, seed = args
    from ros2_swarm_transport.headless_runner import run_mission
    sdf, js = paths(w)
    try:
        _, S = run_mission(sdf, js, algo=algo, seed=seed, sigma0=0.05,
                           verbose=False, phase=True, timesteps=1500,
                           vfh_over=to_vfh_over(algo, flat))
        return dict(algo=algo, ci=ci, world=w, seed=seed, cost=mission_cost(S),
                    reached=int(S["reached_goal"]), time=S["total_time"],
                    contact=S["collisions"], reject=S["n_reject"])
    except Exception as e:                      # noqa: BLE001
        return dict(algo=algo, ci=ci, world=w, seed=seed, cost=400.0, reached=0,
                    time=float("nan"), contact=-1, reject=-1, error=str(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--n", type=int, default=N_CONFIGS)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    best_path = os.path.join(OUT, "best_params.json")
    best = json.load(open(best_path)) if os.path.exists(best_path) else {}
    for algo in (a.only or list(SPACES)):
        cfgs = sample_configs(algo, a.n)
        jobs = [(algo, ci, c, w, 201 + k) for ci, c in enumerate(cfgs)
                for k, w in enumerate(TRAIN)]
        t0 = time.time()
        with ProcessPoolExecutor(a.jobs) as ex:
            R = list(ex.map(eval_one, jobs))
        rows = []
        for ci, c in enumerate(cfgs):
            rr = [r for r in R if r["ci"] == ci]
            rows.append(dict(ci=ci, params=json.dumps(c), mean_cost=np.mean([r["cost"] for r in rr]),
                             reached=sum(r["reached"] for r in rr), n=len(rr),
                             mean_time=np.nanmean([r["time"] for r in rr]),
                             contact_steps=sum(max(r["contact"], 0) for r in rr),
                             reject_steps=sum(max(r["reject"], 0) for r in rr)))
        with open(os.path.join(OUT, f"{algo}.csv"), "w", newline="") as fh:
            wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
        b = min(rows, key=lambda r: r["mean_cost"])
        best[algo] = dict(params=json.loads(b["params"]), vfh_over=to_vfh_over(algo, json.loads(b["params"])),
                          mean_cost=b["mean_cost"], default_cost=rows[0]["mean_cost"],
                          configs=len(rows), missions_per_config=len(TRAIN),
                          wall_s=round(time.time() - t0, 1))
        json.dump(best, open(best_path, "w"), indent=2, default=lambda o: list(o) if isinstance(o, tuple) else float(o))
        print(f"{algo}: default {rows[0]['mean_cost']:.1f} -> best {b['mean_cost']:.1f} "
              f"(config {b['ci']}) in {time.time() - t0:.0f} s", flush=True)


if __name__ == "__main__":
    main()
