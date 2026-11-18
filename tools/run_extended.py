#!/usr/bin/env python3
"""Extended experiments E15-E18 (Python headless stack, randomised traffic phase).

    python3 tools/run_extended.py --exp E15 --jobs 4
    python3 tools/run_extended.py --exp E15,E16,E17,E18 --jobs 4
    python3 tools/run_extended.py --exp E16 --list

  E15 warehouse layouts   8 worlds (2 reference + 6 generated, 30x20 m to
                          80x50 m, racks / narrow aisles / block stacking)
                          x 8 laws (incl. MPPI and the learned policy) x 10 seeds
  E16 team size           3, 4, 6, 9 robots; fixed ~1.2 m payload and larger
                          payloads; 3 worlds x 5 laws x 10 seeds
  E17 safety monitor      ground truth (audit: truth decides, monitor scored in
                          the shadow) vs geometric sensor monitor vs learned
                          conformal monitor; 2 noise levels x 4 laws x 4 worlds x 6
                          seeds; plus a k_sigma / v_obs sweep of the sensor monitor
  E19 PA-VFH ablation    PA-VFH (neural occupancy) vs PA-VFH (no NN, analytic
                          kernel, same instantaneous scan) vs PA-VFH-K (former
                          P-VFH, contract-predicted scan) vs MPC / MPPI; 3 noise
                          levels x 5 worlds x 6 seeds
  E18 baseline tuning     default vs tuned parameters (tools/tune_baselines.py)
                          on held-out worlds/seeds, 6 laws x 10 worlds x 5 seeds

Every run uses the randomised traffic phase (independent stream keyed on the
seed) and the corrected (exact) collision gate. Seeds 1..10 are evaluation
seeds; training and tuning used seeds >= 101. Output: results/extended/<exp>.csv,
one row per run, failed runs included (reached = 0, error column filled on
an exception). Resumable.
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
W = os.path.join(PKG, "worlds")
OUT = os.path.join(ROOT, "results", "extended")
sys.path.insert(0, PKG)

COLS = ["exp", "key", "world", "algo", "formation", "spacing", "n_robots", "seed",
        "sigma0", "gate_source", "tuned", "k_sigma", "v_obs", "phase", "t0_offset",
        "reached", "total_time", "transport_time", "path_length", "path_eff",
        "collisions", "coll_static", "coll_mover", "n_reject", "n_escape",
        "min_clear", "mean_clear", "comp_ms_mean", "mon_checks", "mon_reject",
        "mon_false_accept", "mon_false_reject", "wall", "error"]

LAYOUTS = ["dynamic", "awswh", "wh_s_racks", "wh_m_racks", "wh_l_racks",
           "wh_xl_racks", "wh_m_narrow", "wh_m_blockstack"]
HELD_OUT = [f"rand{k:02d}" for k in range(6, 11)] + ["maze", "awswh", "wh_m_racks",
                                                     "wh_m_blockstack", "wh_l_racks"]


def paths(w):
    return (os.path.join(W, f"swarm_world_{w}.sdf"),
            os.path.join(W, "dynamic_obstacles.json" if w == "dynamic"
                         else f"dynamic_obstacles_{w}.json"))


def job(exp, world, algo, seed, formation="square", spacing=1.18, n_robots=None,
        sigma0=0.02, gate_source="truth", tuned=0, k_sigma=3.0, v_obs=1.0):
    key = (f"{world}|{algo}|{formation}|{spacing:g}|{n_robots}|s{seed}|sg{sigma0:g}|"
           f"{gate_source}|t{tuned}|k{k_sigma:g}|v{v_obs:g}")
    return dict(exp=exp, key=key, world=world, algo=algo, seed=seed, formation=formation,
                spacing=spacing, n_robots=n_robots, sigma0=sigma0, gate_source=gate_source,
                tuned=tuned, k_sigma=k_sigma, v_obs=v_obs)


def build(exp):
    J = []
    if exp == "E15":
        for w in LAYOUTS:
            for a in ("vfh", "fgm", "dwa", "mpc", "mppi", "pvfh", "pvfh_prob", "nn",
                      "pavfh_nonn", "pavfh"):
                for s in range(1, 11):
                    J.append(job(exp, w, a, s))
    elif exp == "E16":
        teams = [("triangle", 1.18, 3), ("square", 1.18, 4), ("grid3x3", 0.59, 9),
                 ("grid2x3", 1.18, 6), ("grid3x3", 1.0, 9)]
        for w in ("dynamic", "wh_m_racks", "wh_l_racks"):
            for f, sp, k in teams:
                for a in ("dwa", "mpc", "mppi", "pvfh_prob", "nn", "pavfh"):
                    for s in range(1, 11):
                        J.append(job(exp, w, a, s, formation=f, spacing=sp, n_robots=k))
    elif exp == "E17":
        for w in ("dynamic", "awswh", "wh_m_racks", "rand08"):
            for gs in ("audit", "sensor", "learned"):
                for sg in (0.02, 0.10):
                    for a in ("dwa", "mpc", "mppi", "pvfh_prob", "pavfh"):
                        for s in range(1, 7):
                            J.append(job(exp, w, a, s, sigma0=sg, gate_source=gs))
        for w in ("dynamic", "awswh"):                     # monitor margin sweep
            for k in (1.0, 2.0, 3.0, 4.0):
                for v in (0.5, 1.0):
                    for s in range(1, 7):
                        J.append(job("E17s", w, "mpc", s, sigma0=0.10,
                                     gate_source="sensor", k_sigma=k, v_obs=v))
    elif exp == "E18":
        for w in HELD_OUT:
            for a in ("vfh", "pvfh_prob", "fgm", "dwa", "mpc", "mppi", "pavfh"):
                for t in (0, 1):
                    for s in range(1, 6):
                        J.append(job(exp, w, a, s, sigma0=0.05, tuned=t))
    elif exp == "E20":                 # pre-registered confirmation of E19 (docs/PREREGISTRATION_E20.md)
        for w in ("dynamic", "awswh", "wh_m_racks", "rand08", "maze"):
            for a in ("pavfh", "pavfh_nonn"):
                for sg in (0.02, 0.10, 0.20):
                    for s in range(7, 31):
                        J.append(job(exp, w, a, s, sigma0=sg))
    elif exp == "E19":
        for w in ("dynamic", "awswh", "wh_m_racks", "rand08", "maze"):
            for a in ("pavfh", "pavfh_nonn", "pvfh_prob", "mpc", "mppi"):
                for sg in (0.02, 0.10, 0.20):
                    for s in range(1, 7):
                        J.append(job(exp, w, a, s, sigma0=sg))
    else:
        raise ValueError(exp)
    return J


_BEST = None


def best_params():
    global _BEST
    if _BEST is None:
        p = os.path.join(ROOT, "results", "tuning", "best_params.json")
        _BEST = json.load(open(p)) if os.path.exists(p) else {}
    return _BEST


def run(j):
    from ros2_swarm_transport.headless_runner import run_mission
    sdf, js = paths(j["world"])
    r = {c: j.get(c, "") for c in COLS}
    r.update(phase=1, n_robots=j["n_robots"] or "")
    t0 = time.perf_counter()
    try:
        over = {}
        if j["tuned"]:
            over = best_params()[j["algo"]]["vfh_over"]
        _, S = run_mission(sdf, js, algo=j["algo"], seed=j["seed"], sigma0=j["sigma0"],
                           formation=j["formation"], grid_spacing=j["spacing"],
                           n_robots=j["n_robots"], verbose=False, phase=True,
                           timesteps=2000, gate_source=j["gate_source"],
                           mon_k_sigma=j["k_sigma"], mon_v_obs=j["v_obs"],
                           vfh_over=over)
        g = S.get("gate_stats") or {}
        r.update(n_robots=S["n_robots"], t0_offset=S["t0_offset"],
                 reached=int(S["reached_goal"]), total_time=S["total_time"],
                 transport_time=S["transport_time"], path_length=S["path_length"],
                 path_eff=S["path_efficiency"], collisions=S["collisions"],
                 coll_static=S["coll_static"], coll_mover=S["coll_mover"],
                 n_reject=S["n_reject"], n_escape=S["n_escape"],
                 min_clear=S["min_clearance"], mean_clear=S["mean_clearance"],
                 comp_ms_mean=S["comp_ms_mean"], mon_checks=g.get("checks", ""),
                 mon_reject=g.get("reject", ""), mon_false_accept=g.get("false_accept", ""),
                 mon_false_reject=g.get("false_reject", ""))
    except Exception as e:                       # noqa: BLE001
        r.update(reached=0, error=f"{type(e).__name__}: {e}"[:200].replace(",", ";"))
    r["wall"] = round(time.perf_counter() - t0, 2)
    return r


def fmt(v):
    if isinstance(v, float):
        return "NaN" if math.isnan(v) else f"{v:.6g}"
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="E15")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for exp in a.exp.split(","):
        J = build(exp)
        out = os.path.join(OUT, f"{exp}.csv")
        done = set()
        if os.path.exists(out):
            with open(out) as fh:
                done = {row["exp"] + "|" + row["key"] for row in csv.DictReader(fh)}
        else:
            with open(out, "w") as fh:
                fh.write(",".join(COLS) + "\n")
        todo = [j for j in J if j["exp"] + "|" + j["key"] not in done]
        print(f"{exp}: {len(J)} runs, {len(todo)} to do -> {out}", flush=True)
        if a.list:
            continue
        t0 = time.time()
        with ProcessPoolExecutor(a.jobs) as ex, open(out, "a") as fh:
            futs = [ex.submit(run, j) for j in todo]
            for i, f in enumerate(as_completed(futs), 1):
                r = f.result()
                fh.write(",".join(fmt(r.get(c, "")) for c in COLS) + "\n")
                fh.flush()
                if r.get("error"):
                    print(f"  RUN FAILED {r['key']}: {r['error']}", flush=True)
                if i % 50 == 0 or i == len(futs):
                    print(f"  [{i}/{len(futs)}] {time.time() - t0:.0f} s", flush=True)


if __name__ == "__main__":
    main()
