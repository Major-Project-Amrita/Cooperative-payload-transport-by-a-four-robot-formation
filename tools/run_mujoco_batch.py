"""The MuJoCo physics-validation batch (resumable: finished runs are skipped).

  V1  laws x payload modes, square, AWS warehouse, motion limits on
  V2  formation shapes x payload modes (MPC and PA-VFH-K)
  V3  motion limits off (the kinematic model's default) vs on
  V4  seeds 1-5 (MPC and PA-VFH-K, carried plate)
  V5  the MATLAB study's own warehouse (swarm_world_dynamic) under physics
  V6  formation feedback off (open-loop frame)
  P1  replication: 5 planners x 3 payload modes x seeds 1-10, randomised
      traffic phase, AWS warehouse, motion limits on       -> results/mujoco_phase/
  P2  robot-parameter sweep, one factor at a time from nominal (chassis mass,
      wheel speed limit, wheel friction, payload mass, wheel force limit),
      MPC and PA-VFH-K, carried plate, seeds 1-5, randomised phase
      (the planner keeps its nominal model: this measures model mismatch)

    python3 tools/run_mujoco_batch.py            # everything (~1 h, 1 core)
    python3 tools/run_mujoco_batch.py --videos   # + the showcase videos
"""
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, PKG)
from ros2_swarm_transport import mujoco_validation as mv

OUT = os.path.join(ROOT, "results", "mujoco")
OUT_PHASE = os.path.join(ROOT, "results", "mujoco_phase")
W = os.path.join(PKG, "worlds")
J = []


def _run_job(t):
    j, out, vid = t
    kw = {}
    if j["world"]:
        kw = dict(world_sdf=os.path.join(W, j["world"]),
                  contract=os.path.join(W, "dynamic_obstacles.json"))
    try:
        mv.run(algo=j["algo"], formation=j["formation"], spacing=j["spacing"],
               payload=j["payload"], seed=j["seed"], kin=j["kin"], out_dir=out,
               hold_err=j["hold"], phase=j["phase"], robot=j["robot"],
               name=j["name"], video=vid, verbose=False, **kw)
    except Exception as e:                       # noqa: BLE001
        print(f"RUN FAILED {j['name']}: {type(e).__name__}: {e}", flush=True)
        with open(os.path.join(out, j["name"] + ".failed"), "w") as fh:
            fh.write(f"{type(e).__name__}: {e}\n")


def add(tag, algo, formation, spacing, payload, seed=1, kin=True, world=None, video=False,
        hold=0.6, phase=False, robot=None, rtag=""):
    name = f"{tag}_{algo}_{formation}_{payload}_s{seed}" + ("" if kin else "_nokin") \
        + ("" if hold else "_nohold") + (f"_{rtag}" if rtag else "")
    J.append(dict(name=name, algo=algo, formation=formation, spacing=spacing,
                  payload=payload, seed=seed, kin=kin, world=world, video=video, tag=tag,
                  hold=hold, phase=phase, robot=robot or {}))

for a in ("mpc", "dwa", "pvfh_prob", "pvfh", "vfh"):
    for p in ("virtual", "on_top", "behind"):
        add("V1", a, "square", 1.18, p, video=(a in ("mpc", "pvfh_prob") and p != "virtual"))
for a in ("mpc", "pvfh_prob"):
    for f in ("diamond", "triangle", "line"):
        for p in ("on_top", "behind"):
            add("V2", a, f, 0.60, p, video=(a == "mpc" and p == "on_top"))
for a in ("mpc", "pvfh_prob"):
    for p in ("virtual", "on_top"):
        add("V3", a, "square", 1.18, p, kin=False)
for a in ("mpc", "pvfh_prob"):
    for s in (2, 3, 4, 5):
        add("V4", a, "square", 1.18, "on_top", seed=s)
for a in ("mpc", "pvfh_prob"):
    for p in ("virtual", "on_top"):
        add("V5", a, "square", 1.18, p, world="swarm_world_dynamic.sdf")
for a in ("mpc", "pvfh_prob"):          # V6: open-loop frame (no formation feedback)
    add("V6", a, "square", 1.18, "on_top", hold=0.0)

for a in ("mpc", "mppi", "dwa", "pvfh_prob", "nn", "pavfh"):     # P1: replication
    for p in ("virtual", "on_top", "behind"):
        for s in range(1, 11):
            add("P1", a, "square", 1.18, p, seed=s, phase=True)
SWEEP = [("mass3", dict(chassis_mass=3.0)), ("mass12", dict(chassis_mass=12.0)),
         ("vmax08", dict(v_wheel_max=0.8)), ("vmax16", dict(v_wheel_max=1.6)),
         ("fric06", dict(wheel_friction=0.6)), ("load05", dict(payload_scale=0.5)),
         ("load20", dict(payload_scale=2.0)), ("force1", dict(force_max=1.0))]
for a in ("mpc", "pvfh_prob", "pavfh"):                  # P2: robot sweep
    for rt, rb in SWEEP:
        for s in range(1, 6):
            add("P2", a, "square", 1.18, "on_top", seed=s, phase=True, robot=rb, rtag=rt)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", action="store_true")
    ap.add_argument("--only", default=None, help="V1..V5")
    ap.add_argument("--jobs", type=int, default=1)
    a = ap.parse_args()
    todo = []
    for j in J:
        if a.only and j["tag"] not in a.only.split(","):
            continue
        out = OUT_PHASE if j["tag"].startswith("P") else OUT
        os.makedirs(out, exist_ok=True)
        vid = os.path.join(out, j["name"] + ".mp4") if (a.videos and j["video"]) else None
        if os.path.exists(os.path.join(out, j["name"] + ".json")) and (vid is None or os.path.exists(vid)):
            continue
        todo.append((j, out, vid))
    if a.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(a.jobs) as ex:
            list(ex.map(_run_job, todo))
        sys.exit(0)
    for j, out, vid in todo:
        kw = {}
        if j["world"]:
            kw = dict(world_sdf=os.path.join(W, j["world"]),
                      contract=os.path.join(W, "dynamic_obstacles.json"))
        try:
            mv.run(algo=j["algo"], formation=j["formation"], spacing=j["spacing"],
                   payload=j["payload"], seed=j["seed"], kin=j["kin"], out_dir=out,
                   hold_err=j["hold"], phase=j["phase"], robot=j["robot"],
                   name=j["name"], video=vid, **kw)
        except Exception as e:                       # noqa: BLE001
            print(f"RUN FAILED {j['name']}: {type(e).__name__}: {e}", flush=True)
