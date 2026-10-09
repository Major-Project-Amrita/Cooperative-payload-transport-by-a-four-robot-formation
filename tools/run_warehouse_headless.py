"""The validation warehouse on the Python headless stack.

  set A  8 laws x seeds 1-5, sigma0 = 0.02, reference gate, limits off
         (the configuration of the MATLAB warehouse batch, matlab_ref/wh_batch.m)
  set B  8 laws x seeds 1-10, exact gate, motion limits off and on

    python3 tools/run_warehouse_headless.py --jobs 4
Writes results/warehouse_headless.csv (resumable).
"""
import argparse, csv, os, sys
from concurrent.futures import ProcessPoolExecutor
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, PKG)
W = os.path.join(PKG, "worlds")
ALGOS = ["apf", "vfh", "fgm", "dwa", "mpc", "pvfh", "pvfh_filt", "pvfh_prob"]
COLS = ["set", "algo", "seed", "gate", "kin", "reached", "total_time", "path_length",
        "path_efficiency", "min_clearance", "collisions", "coll_static", "coll_mover",
        "n_reject", "n_escape", "max_robot_speed", "comp_ms_mean"]


def one(j):
    from ros2_swarm_transport.headless_runner import run_mission
    _, S = run_mission(f"{W}/swarm_world_awswh.sdf", f"{W}/dynamic_obstacles_awswh.json",
                       algo=j["algo"], seed=j["seed"], gate=j["gate"], kin=j["kin"], verbose=False)
    r = dict(j)
    r.update(reached=int(S["reached_goal"]), **{k: S[k] for k in COLS[6:]})
    return r


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=4)
    a = ap.parse_args()
    J = [dict(set="A", algo=al, seed=s, gate="matlab", kin=False) for al in ALGOS for s in range(1, 6)]
    J += [dict(set="B", algo=al, seed=s, gate="exact", kin=k) for al in ALGOS
          for s in range(1, 11) for k in (False, True)]
    out = os.path.join(ROOT, "results", "warehouse_headless.csv")
    done = set()
    if os.path.exists(out):
        done = {(r["set"], r["algo"], r["seed"], r["kin"]) for r in csv.DictReader(open(out))}
    else:
        open(out, "w").write(",".join(COLS) + "\n")
    J = [j for j in J if (j["set"], j["algo"], str(j["seed"]), str(j["kin"])) not in done]
    print(len(J), "missions")
    with ProcessPoolExecutor(a.jobs) as ex, open(out, "a") as fh:
        for r in ex.map(one, J):
            fh.write(",".join(str(r[c]) for c in COLS) + "\n")
            fh.flush()
    print("wrote", out)
