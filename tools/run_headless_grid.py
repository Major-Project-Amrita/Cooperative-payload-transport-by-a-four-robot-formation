"""The paper's Monte Carlo grid (E1-E3, E5-E14) on the Python headless stack.

The manifest is a line-for-line port of build_jobs in
run_paper_experiments.m: same cells, same seeds, same `key` and `run_name`
strings, identical configurations simulated once and emitted under every
experiment label that shares them. Output columns equal the MATLAB
results_all.csv, with stack = ros2_headless, so make_paper_figures.py and
tools/cross_stack.py read both.

    python3 tools/run_headless_grid.py --jobs 10                 # full grid
    python3 tools/run_headless_grid.py --mode smoke              # 1 seed/cell
    python3 tools/run_headless_grid.py --only E2 --jobs 10
    python3 tools/run_headless_grid.py --gate exact --tag exactgate --no-runs

Resumable: rows already in the output CSV are skipped.
"""
import argparse
import csv
import glob
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
WORLDS = os.path.join(PKG, "worlds")
sys.path.insert(0, PKG)

COLS = ["stack", "exp", "algo", "sense_kind", "sigma0", "beams", "seed",
        "world", "formation", "spacing_used", "mover_scale", "adl", "contract",
        "loc_sigma", "comm_drop", "n_movers", "kin", "reached", "collisions",
        "coll_static", "coll_mover", "n_reject", "n_escape", "path_length",
        "path_eff", "min_clear", "mean_clear", "total_time", "transport_time",
        "total_turn", "comp_ms_mean", "max_robot_speed", "run_name", "key"]


def g(x):
    return f"{x:g}"


def build_jobs(mode="full", nseeds=None, only=None):
    ALL = ["apf", "vfh", "fgm", "dwa", "mpc", "pvfh", "pvfh_filt", "pvfh_prob"]
    LADDER = ["vfh", "pvfh", "pvfh_filt", "pvfh_prob"]
    FORMS = ["square", "diamond", "triangle", "line"]
    SIG = [0.02, 0.05, 0.10, 0.20, 0.40]
    W0, C0 = "swarm_world_dynamic.sdf", "dynamic_obstacles.json"
    smoke = mode == "smoke"
    n = {"quick": 5, "smoke": 1}.get(mode, 10)
    if nseeds:
        n = nseeds
    seeds = [7] if smoke else list(range(1, n + 1))
    seeds2 = [7] if smoke else list(range(1, 2 * n + 1))
    J = []

    def add(ex, algo, kind, sg, nb, sd, form, sp, w, c, ms, adl, loc=0, drop=0,
            nmov=-1, kin=0):
        if only and ex != only:
            return
        key = (f"{algo}|{kind}|{g(sg)}|{nb}|s{sd}|{form}|{g(sp)}|"
               f"{w.replace('.sdf', '')}|{c.replace('.json', '')}|m{g(ms)}|"
               f"a{adl}|l{g(loc)}|d{g(drop)}|n{nmov}|k{kin}")
        J.append(dict(exp=ex, algo=algo, kind=kind, sigma0=sg, beams=nb,
                      seed=sd, formation=form, spacing=sp, world=w, contract=c,
                      mover_scale=ms, adl=adl, loc=loc, drop=drop, nmov=nmov,
                      kin=kin, key=key))

    for a in (["pvfh_prob", "dwa", "fgm"] if smoke else ALL):
        for sd in seeds:
            add("E1", a, "lidar", 0.02, 288, sd, "square", 1.18, W0, C0, 1, 1)
    for a in (["vfh", "pvfh_prob"] if smoke else LADDER):
        for sg in ([0.02, 0.40] if smoke else SIG):
            for sd in seeds2:
                add("E2", a, "lidar", sg, 288, sd, "square", 1.18, W0, C0, 1, 1)
    for a in (["pvfh_prob"] if smoke else ["pvfh", "pvfh_prob"]):
        for nb in ([8] if smoke else [288, 64, 16, 8]):
            for sd in seeds:
                add("E3", a, "lidar" if nb >= 288 else "tof", 0.10, nb, sd,
                    "square", 1.18, W0, C0, 1, 1)
    for f in (["line"] if smoke else FORMS):
        for a in (["pvfh_prob"] if smoke else ALL):
            for sd in seeds:
                add("E5", a, "lidar", 0.02, 288, sd, f, 0.60, W0, C0, 1, 1)
    K6 = [("tof", 16)] if smoke else [("lidar", 288), ("tof", 16), ("tof", 8)]
    for kind, nb in K6:
        for sg in ([0.10] if smoke else [0.02, 0.10]):
            for a in (["vfh"] if smoke else LADDER):
                for sd in seeds:
                    add("E6", a, kind, sg, nb, sd, "square", 1.18, W0, C0, 1, 1)
    for w, c in [("swarm_world_plaza.sdf", "dynamic_obstacles_plaza.json"),
                 ("swarm_world_maze.sdf", "dynamic_obstacles_maze.json")]:
        for a in (["pvfh_prob"] if smoke else ALL):
            for sg in ([0.02] if smoke else [0.02, 0.20]):
                for sd in seeds:
                    add("E7", a, "lidar", sg, 288, sd, "square", 1.18, w, c, 1, 1)
    A5 = ["pvfh_prob"] if smoke else ["vfh", "dwa", "mpc", "pvfh", "pvfh_prob"]
    A4 = ["pvfh_prob"] if smoke else ["dwa", "mpc", "pvfh", "pvfh_prob"]
    for ms in ([2.0] if smoke else [0.5, 1.0, 1.5, 2.0]):
        for a in A5:
            for sd in seeds:
                add("E8", a, "lidar", 0.02, 288, sd, "square", 1.18, W0, C0, ms, 1)
    for a in (["pvfh_prob"] if smoke else LADDER):
        for sd in seeds:
            add("E9", a, "lidar", 0.02, 288, sd, "line", 0.60, W0, C0, 1, 0)
    for lc in ([0.15] if smoke else [0, 0.05, 0.15, 0.30]):
        for a in A4:
            for sd in seeds:
                add("E10", a, "lidar", 0.02, 288, sd, "square", 1.18, W0, C0,
                    1, 1, lc)
    CD = "dynamic_obstacles_dense.json"
    for nm in ([0, 6] if smoke else [0, 1, 3, 6]):
        for a in A5:
            for sd in seeds:
                add("E11", a, "lidar", 0.02, 288, sd, "square", 1.18, W0, CD,
                    1, 1, 0, 0, nm)
    for dr in ([0.3] if smoke else [0, 0.1, 0.3, 0.5]):
        for a in A4:
            for sd in seeds:
                add("E12", a, "lidar", 0.02, 288, sd, "square", 1.18, W0, C0,
                    1, 1, 0, dr)
    RW = sorted(os.path.basename(p) for p in
                glob.glob(os.path.join(WORLDS, "swarm_world_rand*.sdf")))
    if smoke:
        RW = RW[:1]
    for w in RW:
        cj = w.replace("swarm_world_", "dynamic_obstacles_").replace(".sdf", ".json")
        for a in (["pvfh_prob"] if smoke else ALL):
            for sg in ([0.02] if smoke else [0.02, 0.20]):
                for sd in seeds[:3]:
                    add("E13", a, "lidar", sg, 288, sd, "square", 1.18, w, cj, 1, 1)
    for a in A5:
        for sd in seeds:
            add("E14", a, "lidar", 0.02, 288, sd, "square", 1.18, W0, C0, 1, 1,
                0, 0, -1, 1)
    return J


def run_name(j):
    w = j["world"].replace("swarm_world_", "").replace(".sdf", "")
    n = 99 if j["nmov"] < 0 else j["nmov"]
    return (f"{j['algo']}_{j['kind']}{j['beams']:03d}_sg{round(j['sigma0']*100):03d}_"
            f"{j['formation']}_sp{round(j['spacing']*100):03d}_{w}_"
            f"m{round(j['mover_scale']*100):03d}_a{j['adl']}_"
            f"l{round(j['loc']*100):03d}_d{round(j['drop']*100):02d}_n{n}_"
            f"k{j['kin']}_s{j['seed']}")


def run_one(args):
    j, gate, runs_dir = args
    from ros2_swarm_transport.headless_runner import run_mission, write_csv
    name = run_name(j)
    r = dict(stack="ros2_headless", exp=j["exp"], algo=j["algo"],
             sense_kind=j["kind"], sigma0=j["sigma0"], beams=j["beams"],
             seed=j["seed"], world=j["world"], formation=j["formation"],
             spacing_used=j["spacing"], mover_scale=j["mover_scale"],
             adl=j["adl"], contract=j["contract"], loc_sigma=j["loc"],
             comm_drop=j["drop"], n_movers=j["nmov"], kin=j["kin"], reached=0,
             run_name=name, key=j["key"])
    t0 = time.perf_counter()
    try:
        rows, S = run_mission(
            os.path.join(WORLDS, j["world"]), os.path.join(WORLDS, j["contract"]),
            algo=j["algo"], sense=j["kind"], n_beams=j["beams"],
            sigma0=j["sigma0"], seed=j["seed"], grid_spacing=j["spacing"],
            formation=j["formation"], mover_scale=j["mover_scale"],
            max_movers=None if j["nmov"] < 0 else j["nmov"], verbose=False,
            timesteps=3000, adl=bool(j["adl"]), kin=bool(j["kin"]),
            loc_sigma_pos=j["loc"], loc_sigma_yaw=j["loc"] * math.radians(20),
            comm_drop=j["drop"], gate=gate)
        if runs_dir:
            write_csv(rows, os.path.join(runs_dir, name + ".csv"))
        r.update(reached=int(S["reached_goal"]), collisions=S["collisions"],
                 coll_static=S["coll_static"], coll_mover=S["coll_mover"],
                 n_reject=S["n_reject"], n_escape=S["n_escape"],
                 path_length=S["path_length"], path_eff=S["path_efficiency"],
                 min_clear=S["min_clearance"], mean_clear=S["mean_clearance"],
                 total_time=S["total_time"], transport_time=S["transport_time"],
                 total_turn=S["total_turning"], comp_ms_mean=S["comp_ms_mean"],
                 max_robot_speed=S["max_robot_speed"])
    except Exception as e:                                # noqa: BLE001
        r["error"] = f"{type(e).__name__}: {e}"
    r["wall"] = time.perf_counter() - t0
    return r


def fmt(v):
    if isinstance(v, str):
        return v
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "NaN"
    return f"{v:.6g}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mode", default="full", choices=["full", "quick", "smoke"])
    ap.add_argument("--only", default=None, help="one experiment, e.g. E2")
    ap.add_argument("--seeds", type=int, default=None)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--gate", default="matlab", choices=["matlab", "exact"],
                    help="matlab = replicate the reference gate (default); "
                         "exact = corrected separating-axis test")
    ap.add_argument("--tag", default="", help="suffix of the output files")
    ap.add_argument("--no-runs", action="store_true",
                    help="do not save the per-step logs")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    J = build_jobs(a.mode, a.seeds, a.only.upper() if a.only else None)
    uniq = {}
    for j in J:
        uniq.setdefault(j["key"], j)
    if a.list:
        for i, j in enumerate(J, 1):
            print(f"{i:4d}  {j['exp']:<3s} {j['key']}")
        print(f"{len(J)} jobs, {len(uniq)} unique configurations")
        return
    suffix = ("_" + a.tag) if a.tag else ""
    if a.mode == "smoke":
        suffix += "_smoke"
    out = os.path.join(ROOT, "results", f"results_headless{suffix}.csv")
    runs_dir = None if a.no_runs else os.path.join(ROOT, "results", f"runs{suffix}")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if runs_dir:
        os.makedirs(runs_dir, exist_ok=True)

    done = set()
    if os.path.exists(out):
        with open(out) as fh:
            for row in csv.DictReader(fh):
                done.add(row["exp"] + "|" + row["key"])
    else:
        with open(out, "w") as fh:
            fh.write(",".join(COLS) + "\n")
    todo = [j for k, j in uniq.items()
            if any(x["key"] == k and x["exp"] + "|" + k not in done for x in J)]
    labels = {}
    for j in J:
        labels.setdefault(j["key"], []).append(j["exp"])
    print(f"{len(J)} jobs, {len(uniq)} unique configurations, {len(todo)} to run, "
          f"{a.jobs} processes, gate={a.gate} -> {out}")
    t0 = time.perf_counter()
    nfail = 0
    with ProcessPoolExecutor(a.jobs) as ex, open(out, "a") as fh:
        futs = [ex.submit(run_one, (j, a.gate, runs_dir)) for j in todo]
        for i, f in enumerate(as_completed(futs), 1):
            r = f.result()
            if "error" in r:
                nfail += 1
                print(f"  RUN FAILED ({r['run_name']}): {r['error']}", flush=True)
            for e in labels[r["key"]]:
                if e + "|" + r["key"] in done:
                    continue
                r["exp"] = e
                fh.write(",".join(fmt(r.get(c)) for c in COLS) + "\n")
            fh.flush()
            if i % 100 == 0 or i == len(futs):
                print(f"  [{i}/{len(futs)}] {time.perf_counter() - t0:.0f} s", flush=True)
    print(f"done: {len(todo)} simulations in {time.perf_counter() - t0:.0f} s wall, "
          f"{nfail} failed")


if __name__ == "__main__":
    main()
