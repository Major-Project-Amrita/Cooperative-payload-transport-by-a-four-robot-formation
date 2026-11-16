#!/usr/bin/env python3
"""Closed-loop check of the REAL-SENSOR mode of the ROS 2 controller node
(use_scans:=true), without a ROS 2 installation.

    python3 tools/ros_sensor_closedloop.py [--algo pavfh] [--gate-source sensor]
                                           [--seeds 1 2 3] [--sigma 0.02]

The real FormationController node runs free-running (lockstep:=false) on a
stand-in rclpy (tools/ros_lockstep_stub.py). Each control tick an ideal
unicycle integrates the node's /<robot>/cmd_vel, and every robot publishes
/<robot>/odom-equivalent poses and a /<robot>/scan LaserScan (360 beams,
range noise sigma) ray-cast from ITS OWN pose against the static world, the
movers where they are now, and its three team-mates. The controller sees
obstacles only through those LaserScans. Contacts are scored afterwards
against the ground truth (statics + movers, the benchmark's own contact
test), which the controller never reads for steering or for the gate
decision. Traffic phase is randomised per seed.

Writes results/ros_sensor_closedloop.json; exit 0 if every mission reaches
the goal.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, PKG)
import ros_lockstep_stub as stub  # noqa: E402


def robot_scan(i, pos, world, movers_now, r, n=360, rmax=8.0, sigma=0.02, rng=None):
    from ros2_swarm_transport.geometry import lidar_scan
    from ros2_swarm_transport.vfh import _movers_only_scan
    rr, ang = lidar_scan(pos[i], world, n, rmax)
    if movers_now:
        rr = np.minimum(rr, _movers_only_scan(pos[i], movers_now, n, rmax, ang))
    # team-mates (circles of radius r): the controller must filter these
    for j in range(len(pos)):
        if j == i:
            continue
        d = pos[j] - pos[i]
        b = d @ np.vstack([np.cos(ang), np.sin(ang)])
        c = d @ d - r * r
        disc = b * b - c
        hit = (disc >= 0) & (b > 0)
        t = np.where(hit, b - np.sqrt(np.maximum(disc, 0)), np.inf)
        rr = np.minimum(rr, t)
    rr = rr + (rng.normal(0, sigma, rr.shape) if rng is not None else 0.0)
    rr = np.where(rr >= rmax - 1e-6, np.inf, rr)       # no return
    return rr, ang


def mission(algo, gsrc, seed, sigma, world_sdf, contract, max_steps):
    from ros2_swarm_transport.formation_controller import FormationController
    stub.BUS.clear(), stub.QUEUE.clear(), stub.PARAMS.clear()
    stub.PARAMS.update(world_sdf=world_sdf, contract=contract, kin=False, lockstep=False,
                       payload="virtual", algo=algo, gate_source=gsrc, seed=seed,
                       hold_err=0.0, use_scans=True, traffic_phase=True, sense_sigma0=max(sigma, 0.02))
    node = FormationController()
    S, F = node.S, node.F
    world, movers, r = S["world"], S["movers"], S["P"]["robot_radius"]
    pos = F["starts"].copy()
    d = F["assigned_slots"] - pos
    yaw = np.arctan2(d[:, 1], d[:, 0])
    cmds = {}
    node.cmd = lambda name, v, w: cmds.__setitem__(name, (v, w))
    rng = np.random.default_rng([seed, 4242])
    from ros2_swarm_transport.geometry import movers_distance
    from ros2_swarm_transport.transport import eval_movers
    contact = contact_s = 0
    min_gap = math.inf
    for k in range(max_steps):
        t = node.t
        mk = [m.eval_at(t) for m in movers]
        for i, n in enumerate(node.robots):
            node.pos[n], node.yaw[n] = pos[i].copy(), float(yaw[i])
        for i, n in enumerate(node.robots):
            rr, ang = robot_scan(i, pos, world, mk, r, sigma=sigma, rng=rng)
            m = stub.Msg()
            m.ranges, m.angle_min = list(rr - 0.0), float(ang[0] - yaw[i])
            m.angle_increment = float(ang[1] - ang[0])
            m.range_min, m.range_max = 0.05, 8.0
            for cb in stub.BUS[f"/{n}/scan"]:
                cb(m)
        cmds.clear()
        node.step()
        if node.phase == 3:
            break
        for i, n in enumerate(node.robots):
            v, w = cmds.get(n, (0.0, 0.0))
            yaw[i] += w * node.dt
            pos[i] += v * node.dt * np.array([np.cos(yaw[i]), np.sin(yaw[i])])
        # ground-truth scoring only (the benchmark's own contact test)
        mt = eval_movers(movers, node.t + node.dt)
        d_s = float(world.distance(pos).min())
        d = min(d_s, float(movers_distance(pos, mt).min())) if mt else d_s
        min_gap = min(min_gap, d - r)
        contact += int(d <= r)
        contact_s += int(d_s <= r)
    return dict(algo=algo, gate_source=gsrc, seed=seed, sigma=sigma,
                reached=bool(node.ctrl.reached_all), sim_time=round(node.t, 1),
                steps=k + 1, contact_steps=contact, contact_steps_static=contact_s,
                min_gap=round(min_gap, 3), traffic_phase=True,
                gate_stats=(node.ctrl.leader.monitor.stats
                            if node.ctrl.leader.monitor is not None else None),
                cloud_points=int(len(node.cloud.pts)), self_filtered=int(node.cloud.n_self))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", default="pavfh")
    ap.add_argument("--gate-source", default="sensor")
    ap.add_argument("--seeds", type=int, nargs="*", default=[1, 2, 3])
    ap.add_argument("--sigma", type=float, default=0.02)
    ap.add_argument("--world", default="swarm_world_dynamic.sdf")
    ap.add_argument("--contract", default="dynamic_obstacles.json")
    ap.add_argument("--max-steps", type=int, default=3000)
    a = ap.parse_args()
    stub.install_stub()
    W = os.path.join(PKG, "worlds")
    rows = []
    for s in a.seeds:
        R = mission(a.algo, a.gate_source, s, a.sigma, os.path.join(W, a.world),
                    os.path.join(W, a.contract), a.max_steps)
        print(json.dumps(R))
        rows.append(R)
    out = os.path.join(ROOT, "results", "ros_sensor_closedloop.json")
    prev = json.load(open(out)) if os.path.exists(out) else []
    keep = [p for p in prev if (p["algo"], p["gate_source"], p["seed"], p["sigma"], p.get("world"))
            not in {(r["algo"], r["gate_source"], r["seed"], r["sigma"], a.world) for r in rows}]
    for r in rows:
        r["world"] = a.world
    json.dump(keep + rows, open(out, "w"), indent=1)
    return 0 if all(r["reached"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
