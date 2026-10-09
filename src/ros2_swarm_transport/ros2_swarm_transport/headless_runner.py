"""Headless mission runner: Simulation8_dynamic.m without ROS or a simulator.

Runs the full two-phase mission (forming + rigid transport) with the same
kinematic loop, geometry, steering dispatch (all eight laws) and motion
contract as the corrected MATLAB reference, in-process. The mission logic
lives in transport.py; this file wires it to a world, logs the shared CSV
schema and computes the run summary of summarize_run.m.

CLI:
    python3 -m ros2_swarm_transport.headless_runner \
        --world worlds/swarm_world_dynamic.sdf \
        --contract worlds/dynamic_obstacles.json \
        --algo pvfh_prob --seed 1 --output logs/ros2_dynamic.csv
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

from .assign_slots import assign_slots
from .geometry import (formation_offsets, load_dynamic_obstacles,
                       movers_distance, world_from_sdf)
from .transport import (FormingPhase, TransportLeader, eval_movers,
                        make_params, predicted_min_mover_gap, rot2d, wrap)
from .vfh import ALGOS, VfhConfig

_formation_offsets = formation_offsets   # back-compat alias


def contract_meta(contract_path):
    """Optional scenario fields of a contract: start point, scatter radius."""
    if not contract_path or not os.path.exists(contract_path):
        return {}
    with open(contract_path) as fh:
        doc = json.load(fh)
    return {k: doc[k] for k in ("start", "scatter_radius") if k in doc}


def contract_waypoints(contract_path):
    """contract_waypoints.m: the optional global route of a contract."""
    if not contract_path or not os.path.exists(contract_path):
        return None
    with open(contract_path) as fh:
        doc = json.load(fh)
    wp = doc.get("waypoints")
    if not wp:
        return None
    return np.asarray(wp, dtype=float).reshape(-1, 2)


def load_scenario(world_path, contract_path, mover_scale=1.0, max_movers=None,
                  robot_height=0.35):
    """World + movers exactly as Simulation8_dynamic loads them."""
    world = world_from_sdf(world_path, robot_height=robot_height,
                           movers_json=contract_path)
    movers = []
    if contract_path:
        movers, _ = load_dynamic_obstacles(contract_path)
    if max_movers is not None and max_movers >= 0:
        movers = movers[:max_movers]
    for m in movers:
        m.speed *= mover_scale
    movers = [m.eval_at(0.0) for m in movers]
    return world, movers


def plan_formation(formation, spacing, start, first_goal, scatter_center=None,
                   scatter_radius=2.2):
    """Slots, start ring and the Hungarian robot->slot assignment."""
    offsets = formation_offsets(formation, 3 if formation == "triangle" else 4,
                                spacing)
    K = len(offsets)
    start = np.asarray(start, dtype=float)
    d = np.asarray(first_goal, dtype=float) - start
    heading0 = math.atan2(d[1], d[0])
    target_slots = (rot2d(heading0) @ offsets.T).T + start
    centre = start if scatter_center is None else np.asarray(scatter_center, float)
    th = 2 * math.pi * np.arange(K) / K
    starts = centre + scatter_radius * np.column_stack([np.cos(th), np.sin(th)])
    cost = np.linalg.norm(starts[:, None, :] - target_slots[None, :, :], axis=2)
    perm = assign_slots(cost)
    return {"offsets_body": offsets, "robot_offsets": offsets[perm],
            "assigned_slots": target_slots[perm], "starts": starts,
            "heading0": heading0, "K": K}


def run_mission(world_path, contract_path, algo="pvfh", sense="lidar",
                n_beams=288, sigma0=0.02, k_rng=0.03, fov_deg=60.0,
                max_range=float("inf"), seed=None, grid_spacing=1.18,
                formation="square", goal=None, start=None, waypoints=None,
                scatter_radius=None,
                mover_scale=1.0, max_movers=None, pred_lead=0.5, pred_dt=0.1,
                baseline_horizon=True, verbose=True, world=None, movers=None,
                **params):
    """Run the full mission; returns (rows, summary). rows are CSV dicts.
    **params overrides transport.DEFAULTS (V, kin, adl, loc_sigma_pos, ...).
    `world` / `movers` may be passed directly instead of the file paths."""
    P = make_params(**params)
    rng = np.random.default_rng(seed)
    if world is None:
        world, movers = load_scenario(world_path, contract_path, mover_scale,
                                      max_movers)
    movers = movers or []
    cfg = VfhConfig(n_beams=n_beams, pred_lead=pred_lead, pred_dt=pred_dt,
                    baseline_horizon=baseline_horizon,
                    sense={"kind": sense, "sigma0": sigma0, "k_rng": k_rng,
                           "fov_deg": fov_deg, "max_range": max_range})
    dt, r = P["dt"], P["robot_radius"]

    if waypoints is None:
        waypoints = contract_waypoints(contract_path)
    if waypoints is None:
        waypoints = np.array([goal if goal is not None else world.goal], float)
    waypoints = np.asarray(waypoints, dtype=float).reshape(-1, 2)
    meta = contract_meta(contract_path)
    if start is None:
        start = meta.get("start", [-12.0, 0.0])
    start = np.asarray(start, dtype=float)
    if scatter_radius is None:
        scatter_radius = meta.get("scatter_radius", 2.2)

    F = plan_formation(formation, grid_spacing, start, waypoints[0],
                       scatter_radius=scatter_radius)
    K, robot_offsets = F["K"], F["robot_offsets"]

    rows = []

    def log_row(t, phase, c, yaw, dist, clr, rpos, x):
        row = {"t": t, "phase": phase, "leader_x": c[0], "leader_y": c[1],
               "leader_yaw": yaw, "dist_to_goal": dist, "clearance": clr}
        for i in range(K):
            row[f"r{i+1}_x"] = rpos[i, 0]
            row[f"r{i+1}_y"] = rpos[i, 1]
        row.update(x)
        rows.append(row)

    def nearest_mover_vel(pts, mt):
        if not mt:
            return 0.0, 0.0
        d = np.array([[np.linalg.norm(p - m.c) - m.r for m in mt] for p in pts])
        k = int(np.unravel_index(d.argmin(), d.shape)[1])
        return float(mt[k].v[0]), float(mt[k].v[1])

    t0_wall = time.perf_counter()
    # ---------------- Phase 1: forming ----------------
    pos = F["starts"].copy()
    d0 = F["assigned_slots"] - pos
    heading = np.arctan2(d0[:, 1], d0[:, 0])
    forming = FormingPhase(world, movers, F["assigned_slots"], P, cfg, algo, rng)
    sim_t = 0.0
    for _ in range(P["forming_max_steps"]):
        sim_t += dt
        prev = pos
        pos, heading, comp_ms = forming.step(pos, heading, sim_t)
        mt = eval_movers(movers, sim_t)
        clr = float(world.distance(pos).min())
        gp = gn = 99.0
        if mt:
            clr = min(clr, float(movers_distance(pos, mt).min()))
            for i in range(K):
                a, b = predicted_min_mover_gap(pos[i], (pos[i] - prev[i]) / dt,
                                               movers, sim_t, dt, P["lookahead"])
                gp, gn = min(gp, a), min(gn, b)
        vx, vy = nearest_mover_vel(pos, mt)
        log_row(sim_t, 1, pos.mean(axis=0), float(heading.mean()),
                float(forming.errs.max()), clr, pos,
                {"mover_vx": vx, "mover_vy": vy, "speed_scale": 1.0,
                 "mover_gap": min(gn, 99.0), "mover_gap_pred": min(gp, 99.0),
                 "comp_ms": comp_ms, "reject": 0, "escape": 0,
                 "clearance_static": clr})
        if forming.formed:
            break
    if not forming.formed and verbose:
        print(f"WARNING: formation not converged "
              f"(max slot error {float(forming.errs.max()):.3f} m)",
              file=sys.stderr)

    # ---------------- Phase 2: rigid transport ----------------
    leader = TransportLeader(world, movers, robot_offsets, P, cfg, algo, rng,
                             pos.mean(axis=0), F["heading0"])
    dock_resid = (rot2d(leader.heading).T @ (pos - leader.slots()).T).T
    dock_steps = 0
    reached_all = True
    for wp in waypoints:
        leader.set_goal(wp)
        reached = False
        for _ in range(P["timesteps"]):
            sim_t += dt
            st = leader.step(sim_t)
            corners = leader.slots()
            if P["dock_tau"] > 0 and dock_steps * dt < 6 * P["dock_tau"]:
                dock_steps += 1
                corners = corners + (rot2d(leader.heading) @ dock_resid.T).T * \
                    math.exp(-dock_steps * dt / P["dock_tau"])
            vx, vy = nearest_mover_vel([leader.pos], st["movers"])
            log_row(sim_t, 2, leader.pos, leader.heading, st["dist"],
                    st["clearance"], corners,
                    {"mover_vx": vx, "mover_vy": vy,
                     "speed_scale": st["v_scale"], "mover_gap": st["near"],
                     "mover_gap_pred": st["near_pre"], "comp_ms": st["comp_ms"],
                     "reject": st["reject"], "escape": st["escape"],
                     "clearance_static": st["clearance_static"]})
            if st["reached"]:
                reached = True
                break
        if not reached:
            reached_all = False
            if verbose:
                print(f"WARNING: waypoint {wp} NOT reached "
                      f"(dist {st['dist']:.2f} m)", file=sys.stderr)

    S = summarize_run(rows, K, dt, r, reached_all)
    S["wall_s"] = time.perf_counter() - t0_wall
    return rows, S


def summarize_run(rows, K, dt, robot_radius, reached):
    """summarize_run.m: per-run aggregates over phase 2."""
    nan = float("nan")
    p2 = [r for r in rows if r["phase"] == 2]
    S = {"reached_goal": bool(reached),
         "total_time": rows[-1]["t"] if rows else 0.0,
         "transport_time": len(p2) * dt}
    if not p2:
        S.update(path_length=0.0, path_efficiency=0.0, min_clearance=nan,
                 mean_clearance=nan, collisions=0, coll_static=0, coll_mover=0,
                 n_reject=0, n_escape=0, max_robot_speed=nan, comp_ms_mean=nan,
                 total_turning=0.0, final_dist=nan)
        return S
    xy = np.array([[r["leader_x"], r["leader_y"]] for r in p2])
    seg = np.hypot(*np.diff(xy, axis=0).T) if len(xy) > 1 else np.zeros(0)
    S["path_length"] = float(seg.sum())
    S["path_efficiency"] = float(np.hypot(*(xy[-1] - xy[0]))) / max(S["path_length"], 2.2e-16)
    clr = np.array([r["clearance"] for r in p2])
    clr_s = np.array([r["clearance_static"] for r in p2])
    hit, hs = clr <= robot_radius, clr_s <= robot_radius
    S["min_clearance"] = float(clr.min())
    S["mean_clearance"] = float(clr.mean())
    S["collisions"] = int(hit.sum())
    S["coll_static"] = int(hs.sum())
    S["coll_mover"] = int((hit & ~hs).sum())
    S["n_reject"] = int(sum(r["reject"] > 0 for r in p2))
    e = np.array([r["escape"] > 0 for r in p2])
    S["n_escape"] = int((e & ~np.concatenate([[False], e[:-1]])).sum())
    S["max_robot_speed"] = nan
    if len(p2) > 70:
        rx = np.array([[r[f"r{i+1}_x"] for i in range(K)] for r in p2[60:]])
        ry = np.array([[r[f"r{i+1}_y"] for i in range(K)] for r in p2[60:]])
        S["max_robot_speed"] = float(np.hypot(np.diff(rx, axis=0),
                                              np.diff(ry, axis=0)).max() / dt)
    comp = [r["comp_ms"] for r in p2 if r["comp_ms"] > 0]
    S["comp_ms_mean"] = float(np.mean(comp)) if comp else nan
    yaw = np.unwrap([r["leader_yaw"] for r in p2])
    S["total_turning"] = float(np.abs(np.diff(yaw)).sum())
    S["final_dist"] = p2[-1]["dist_to_goal"]
    return S


CSV_TAIL = ["mover_vx", "mover_vy", "speed_scale", "mover_gap",
            "mover_gap_pred", "comp_ms", "reject", "escape",
            "clearance_static"]


def write_csv(rows, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    hdr = ["t", "phase", "leader_x", "leader_y", "leader_yaw", "dist_to_goal",
           "clearance"]
    K = sum(1 for k in rows[0] if k.startswith("r") and k.endswith("_x")) if rows else 0
    hdr += [f"r{i+1}_x" for i in range(K)] + [f"r{i+1}_y" for i in range(K)]
    hdr += CSV_TAIL
    with open(path, "w") as fh:
        fh.write(",".join(hdr) + "\n")
        for r in rows:
            fh.write(",".join(f"{float(r.get(h, 0.0)):.6f}" for h in hdr) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="worlds/swarm_world_dynamic.sdf")
    ap.add_argument("--contract", default="worlds/dynamic_obstacles.json")
    ap.add_argument("--algo", default="pvfh", choices=list(ALGOS))
    ap.add_argument("--sense", default="lidar", choices=["lidar", "tof"])
    ap.add_argument("--n-beams", type=int, default=288)
    ap.add_argument("--sigma0", type=float, default=0.02)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--grid-spacing", type=float, default=1.18)
    ap.add_argument("--robot-radius", type=float, default=0.18)
    ap.add_argument("--formation", default="square",
                    choices=["square", "line", "triangle", "diamond", "v"])
    ap.add_argument("--mover-scale", type=float, default=1.0)
    ap.add_argument("--max-movers", type=int, default=None,
                    help="use only the first N movers of the contract")
    ap.add_argument("--loc-sigma", type=float, default=0.0,
                    help="localisation sigma_pos (m); sigma_yaw = 20 deg per m")
    ap.add_argument("--comm-drop", type=float, default=0.0)
    ap.add_argument("--kin", action="store_true",
                    help="load-carrying motion limits (E14)")
    ap.add_argument("--no-adl", action="store_true",
                    help="disable the anti-deadlock layer (E9)")
    ap.add_argument("--gate", default="exact", choices=["exact", "matlab"],
                    help="separating-axis test of the safety gate: exact, or "
                         "bug-compatible with the MATLAB reference")
    ap.add_argument("--timesteps", type=int, default=3000)
    ap.add_argument("--start", type=float, nargs=2, default=None)
    ap.add_argument("--goal", type=float, nargs=2, default=None)
    ap.add_argument("--output", default="logs/ros2_dynamic.csv")
    a = ap.parse_args(argv)

    rows, S = run_mission(
        a.world, a.contract, algo=a.algo, sense=a.sense, n_beams=a.n_beams,
        sigma0=a.sigma0, seed=a.seed, grid_spacing=a.grid_spacing,
        formation=a.formation, mover_scale=a.mover_scale,
        max_movers=a.max_movers, start=a.start, goal=a.goal,
        robot_radius=a.robot_radius, loc_sigma_pos=a.loc_sigma,
        loc_sigma_yaw=a.loc_sigma * math.radians(20.0), comm_drop=a.comm_drop,
        kin=a.kin, adl=not a.no_adl, timesteps=a.timesteps, gate=a.gate)
    write_csv(rows, a.output)
    print(f"wrote {a.output}: reached={S['reached_goal']} "
          f"t={S['total_time']:.1f} s path={S['path_length']:.2f} m "
          f"eff={S['path_efficiency']:.2f} minclr={S['min_clearance']:.3f} m "
          f"coll={S['collisions']} (static {S['coll_static']}, "
          f"mover {S['coll_mover']}) reject={S['n_reject']} "
          f"escape={S['n_escape']} vmax={S['max_robot_speed']:.2f} m/s "
          f"comp={S['comp_ms_mean']:.2f} ms wall={S['wall_s']:.1f} s")
    return 0 if S["reached_goal"] else 1


if __name__ == "__main__":
    sys.exit(main())
