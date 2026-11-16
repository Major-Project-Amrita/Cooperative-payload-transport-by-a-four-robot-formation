#!/usr/bin/env python3
"""MuJoCo physics validation THROUGH ROS 2 (run inside a ROS 2 Humble environment).

    python3 tools/run_ros2_physics.py --batch R1 --jobs 3     # lockstep, vs in-process P3
    python3 tools/run_ros2_physics.py --batch R2 --jobs 3     # free-running, real sensors

R1  the P3 physics configuration (0.10 m gate margin, random traffic phase,
    AWS warehouse, motion limits on) for 6 laws x 3 payload modes x seeds 1-5,
    with the MuJoCo plant and the formation controller as two ROS 2 processes
    exchanging /swarm/state and /swarm/command over DDS in lockstep. Every run
    is compared with the in-process run of the same configuration
    (results/mujoco_phase/P3_*.json): the physics must be identical.
R2  the full deployment stack: the MuJoCo plant runs FREE (wall-clock 10 Hz),
    takes /robotN/cmd_vel, publishes /robotN/odom and /robotN/scan from the
    physical robots' poses; the controller (robots.launch.py configuration:
    PA-VFH, sensor monitor, watchdog) steers from the scans only. Payload on
    top and virtual, seeds 1-5. Contacts are measured by the physics engine.

Each job gets its own ROS_DOMAIN_ID, so parallel jobs never see each other's
topics. Output: results/ros_physics/<batch>/<run>.json and summary.csv.
Resumable.
"""
import argparse
import csv
import json
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
OUT = os.path.join(ROOT, "results", "ros_physics")
LAWS = ("mpc", "mppi", "dwa", "pvfh_prob", "nn", "pavfh")
KEYS = ["reached_goal", "total_time", "n_reject", "coll_mover", "formation_rms",
        "formation_err_max", "contact_steps_static", "contact_steps_mover", "pin_gap_max",
        "payload_tilt_max_deg"]


def jobs(batch):
    J = []
    if batch == "R1":
        for a in LAWS:
            for p in ("virtual", "on_top", "behind"):
                for s in range(1, 6):
                    J.append(dict(batch=batch, algo=a, payload=p, seed=s,
                                  name=f"R1_{a}_square_{p}_s{s}"))
    elif batch == "R2":
        for p in ("on_top", "virtual"):
            for s in range(1, 6):
                J.append(dict(batch=batch, algo="pavfh", payload=p, seed=s,
                              name=f"R2_pavfh_square_{p}_s{s}"))
    elif batch == "DEMO":                       # MuJoCo footage of the ROS 2 deployment
        vd = os.path.join(OUT, "DEMO")
        J.append(dict(batch=batch, algo="pavfh", payload="on_top", seed=3,
                      name="DEMO_deploy_on_top_s3", video=os.path.join(vd, "deploy_mujoco.mp4")))
        J.append(dict(batch=batch, algo="pavfh", payload="on_top", seed=1, drop=20.0,
                      name="DEMO_watchdog_on_top_s1", video=os.path.join(vd, "watchdog_mujoco.mp4")))
    return J


def ros_env(domain):
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(domain)
    e["PYTHONPATH"] = PKG + os.pathsep + e.get("PYTHONPATH", "")
    e.setdefault("MUJOCO_GL", "egl")
    return e


def run_r1(j, domain, out):
    args = ["--ros-args", "-p", f"algo:={j['algo']}", "-p", f"payload:={j['payload']}",
            "-p", f"seed:={j['seed']}", "-p", "traffic_phase:=true", "-p", "gate_margin:=0.1"]
    env = ros_env(domain)
    t0 = time.time()
    plant = subprocess.Popen(["python3", "-m", "ros2_swarm_transport.mujoco_plant"] + args,
                             cwd=PKG, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        subprocess.run(["python3", "-m", "ros2_swarm_transport.formation_controller"] + args +
                       ["-p", f"output_dir:={out}", "-p", f"run_name:={j['name']}"],
                       cwd=PKG, env=env, timeout=1800, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    finally:
        try:
            plant.wait(timeout=30)
        except subprocess.TimeoutExpired:
            plant.kill()
    p = os.path.join(out, j["name"] + ".json")
    if not os.path.exists(p):
        return dict(name=j["name"], error="no result")
    a = json.load(open(p))
    ref_p = os.path.join(ROOT, "results", "mujoco_phase",
                         f"P3_{j['algo']}_square_{j['payload']}_s{j['seed']}.json")
    row = dict(name=j["name"], algo=j["algo"], payload=j["payload"], seed=j["seed"],
               wall_s=round(time.time() - t0, 1))
    row.update({k: a.get(k) for k in KEYS})
    if os.path.exists(ref_p):
        b = json.load(open(ref_p))
        # the container (MuJoCo 3.17, numpy 1.26) and the host (MuJoCo 4.0.1,
        # numpy 2.x) differ in the last floating-point digits: compare with a
        # strict tolerance and record the largest relative difference
        diff, worst = [], 0.0
        for k in KEYS:
            if k not in b:
                continue
            x, y = a.get(k), b.get(k)
            if isinstance(x, (int, float)) and isinstance(y, (int, float)) and not isinstance(x, bool):
                rel = abs(x - y) / max(abs(y), 1e-9)
                worst = max(worst, rel)
                if rel > 1e-6:
                    diff.append(k)
            elif x != y:
                diff.append(k)
        row["identical_to_inprocess"] = not diff
        row["max_rel_diff"] = worst
        row["differs_in"] = ";".join(diff)
    return row


def run_r2(j, domain, out):
    env = ros_env(domain)
    t0 = time.time()
    W = os.path.join(PKG, "worlds")
    common = ["-p", f"payload:={j['payload']}", "-p", f"seed:={j['seed']}",
              "-p", "traffic_phase:=true", "-p", "gate_margin:=0.1"]
    plant = subprocess.Popen(["python3", "-m", "ros2_swarm_transport.mujoco_plant", "--ros-args",
                              "-p", "lockstep:=false", "-p", "publish_scans:=true",
                              "-p", "sense_sigma0:=0.02", "-p", f"output_dir:={out}",
                              "-p", f"run_name:={j['name']}", "-p", "timeout_s:=400.0",
                              "-p", "algo:=pavfh"] + common
                             + (["-p", f"video:={j['video']}"] if j.get("video") else [])
                             + (["-p", f"scan_drop_after:={j['drop']}"] if j.get("drop") is not None else []),
                             cwd=PKG, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ctrl = subprocess.Popen(["python3", "-m", "ros2_swarm_transport.formation_controller",
                             "--ros-args", "-p", "lockstep:=false", "-p", "use_scans:=true",
                             "-p", "algo:=pavfh", "-p", "gate_source:=sensor",
                             "-p", "max_v:=0.6", "-p", "max_w:=1.0",
                             "-p", f"use_sim_time:={'true' if j.get('video') else 'false'}"] + common,
                            cwd=PKG, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        plant.wait(timeout=900)
    except subprocess.TimeoutExpired:
        plant.kill()
    ctrl.terminate()
    try:
        ctrl.wait(timeout=10)
    except subprocess.TimeoutExpired:
        ctrl.kill()
    p = os.path.join(out, j["name"] + ".json")
    if not os.path.exists(p):
        return dict(name=j["name"], error="no result")
    a = json.load(open(p))
    row = dict(name=j["name"], algo="pavfh", payload=j["payload"], seed=j["seed"],
               wall_s=round(time.time() - t0, 1))
    row.update(a)
    return row


def _ref_one(j):
    """In-process physics run of an R1 configuration, in THIS environment."""
    import sys
    sys.path.insert(0, PKG)
    from ros2_swarm_transport.mujoco_validation import run
    out = os.path.join(OUT, "R1_ref")
    if not os.path.exists(os.path.join(out, j["name"] + ".json")):
        run(algo=j["algo"], payload=j["payload"], seed=j["seed"], phase=True, gate_margin=0.1,
            out_dir=out, name=j["name"], verbose=False)
    return j["name"]


def compare_same_env():
    """R1 (ROS 2 lockstep) vs R1_ref (in-process, same container): exact equality."""
    n = same = 0
    for j in jobs("R1"):
        rp = os.path.join(OUT, "R1", j["name"] + ".row.json")
        a_p = os.path.join(OUT, "R1", j["name"] + ".json")
        b_p = os.path.join(OUT, "R1_ref", j["name"] + ".json")
        if not (os.path.exists(rp) and os.path.exists(a_p) and os.path.exists(b_p)):
            continue
        a, b = json.load(open(a_p)), json.load(open(b_p))
        diff = [k for k in KEYS if a.get(k) != b.get(k)]
        row = json.load(open(rp))
        row["identical_same_env"] = not diff
        row["differs_same_env"] = ";".join(diff)
        json.dump(row, open(rp, "w"))
        n += 1
        same += int(not diff)
    print(f"R1 vs in-process in the same environment: {same}/{n} identical on all metrics")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", default="R1")
    ap.add_argument("--jobs", type=int, default=3)
    a = ap.parse_args()
    if a.batch == "R1REF":
        from concurrent.futures import ProcessPoolExecutor
        os.makedirs(os.path.join(OUT, "R1_ref"), exist_ok=True)
        with ProcessPoolExecutor(a.jobs) as ex:
            list(ex.map(_ref_one, jobs("R1")))
        compare_same_env()
        return
    out = os.path.join(OUT, a.batch)
    os.makedirs(out, exist_ok=True)
    J = [j for j in jobs(a.batch) if not os.path.exists(os.path.join(out, j["name"] + ".row.json"))]
    print(f"{a.batch}: {len(J)} runs to do", flush=True)
    fn = run_r1 if a.batch == "R1" else run_r2          # R2 and DEMO: free-running

    import queue
    domains = queue.Queue()
    for d in range(a.jobs):
        domains.put(11 + d)              # one DDS domain per concurrent job

    def work(ij):
        i, j = ij
        dom = domains.get()
        try:
            r = fn(j, dom, out)
        finally:
            domains.put(dom)
        json.dump(r, open(os.path.join(out, j["name"] + ".row.json"), "w"))
        print(json.dumps({k: r.get(k) for k in ("name", "reached_goal", "reached", "total_time",
                                                 "contact_steps_static", "contact_steps_mover",
                                                 "identical_to_inprocess", "differs_in", "error")}),
              flush=True)
        return r
    with ThreadPoolExecutor(a.jobs) as ex:
        list(ex.map(work, list(enumerate(J))))
    rows = [json.load(open(os.path.join(out, f))) for f in sorted(os.listdir(out))
            if f.endswith(".row.json")]
    if rows:
        cols = sorted({k for r in rows for k in r})
        with open(os.path.join(out, "summary.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
