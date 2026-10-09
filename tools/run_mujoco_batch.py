"""The MuJoCo physics-validation batch (resumable: finished runs are skipped).

  V1  laws x payload modes, square, AWS warehouse, motion limits on
  V2  formation shapes x payload modes (MPC and P-VFH)
  V3  motion limits off (the kinematic model's default) vs on
  V4  seeds 1-5 (MPC and P-VFH, carried plate)
  V5  the MATLAB study's own warehouse (swarm_world_dynamic) under physics
  V6  formation feedback off (open-loop frame)

    python3 tools/run_mujoco_batch.py            # everything (~1 h, 1 core)
    python3 tools/run_mujoco_batch.py --videos   # + the showcase videos
"""
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, PKG)
from ros2_swarm_transport import mujoco_validation as mv

OUT = os.path.join(ROOT, "results", "mujoco")
W = os.path.join(PKG, "worlds")
J = []
def add(tag, algo, formation, spacing, payload, seed=1, kin=True, world=None, video=False,
        hold=0.6):
    name = f"{tag}_{algo}_{formation}_{payload}_s{seed}" + ("" if kin else "_nokin") \
        + ("" if hold else "_nohold")
    J.append(dict(name=name, algo=algo, formation=formation, spacing=spacing,
                  payload=payload, seed=seed, kin=kin, world=world, video=video, tag=tag,
                  hold=hold))

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

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", action="store_true")
    ap.add_argument("--only", default=None, help="V1..V5")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for j in J:
        if a.only and j["tag"] != a.only:
            continue
        vid = os.path.join(OUT, j["name"] + ".mp4") if (a.videos and j["video"]) else None
        if os.path.exists(os.path.join(OUT, j["name"] + ".json")) and (vid is None or os.path.exists(vid)):
            continue
        kw = {}
        if j["world"]:
            kw = dict(world_sdf=os.path.join(W, j["world"]),
                      contract=os.path.join(W, "dynamic_obstacles.json"))
        try:
            mv.run(algo=j["algo"], formation=j["formation"], spacing=j["spacing"],
                   payload=j["payload"], seed=j["seed"], kin=j["kin"], out_dir=OUT,
                   hold_err=j["hold"],
                   name=j["name"], video=vid, **kw)
        except Exception as e:                       # noqa: BLE001
            print(f"RUN FAILED {j['name']}: {type(e).__name__}: {e}", flush=True)
