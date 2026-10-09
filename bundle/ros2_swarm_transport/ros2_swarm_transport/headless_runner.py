"""Headless mission runner: the ROS 2-side equivalent of Simulation8_dynamic.m
without ROS or Gazebo.

Runs the full two-phase mission (forming + rigid transport) with the SAME
kinematic loop, the SAME geometry, the SAME steering dispatch (all four
algorithm variants) and the SAME motion contract as the MATLAB simulation,
but entirely in-process. Use it to generate ros2_*.csv logs for compare_runs
and to reproduce the paper's algorithm comparison from the Python stack.

The rigid-transport leader advances exactly like the MATLAB leader (unicycle
kinematics, heading low-pass, weighted APF+steering blend, speed slowdown on
the predicted mover gap, t/t+dt whole-body commit). The four robots are
slaves of the frame (as in MATLAB's kinematic model); a --servo flag instead
P-controllers real robot states to their slots, which is what the Gazebo
stack does and what formation_rms measures.

CLI:
    python3 -m ros2_swarm_transport.headless_runner \
        --world worlds/swarm_world_dynamic.sdf \
        --contract worlds/dynamic_obstacles.json \
        --algo pvfh_prob --sense tof --n-beams 8 --seed 3 \
        --output logs/ros2_dynamic.csv

Writes the shared CSV schema (same columns as swarm_logger.py / MATLAB).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

from .geometry import (eval_mover_pose, eval_mover_vel, formation_clearance,
                       formation_offsets, load_dynamic_obstacles,
                       world_from_sdf)
from .vfh import VfhConfig, steering_point, steering_rigid


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


_formation_offsets = formation_offsets   # back-compat alias


def run_mission(world_path, contract_path, algo="pvfh", sense="lidar",
                n_beams=288, sigma0=0.02, k_rng=0.03, fov_deg=60.0,
                max_range=float("inf"), seed=None, dt=0.1,
                grid_size=2, grid_spacing=1.18, robot_radius=0.18,
                V=0.6, forming_V=0.5, w_goal=1.0, w_vfh=2.0, w_att=1.0,
                max_turn_rate=0.3, forming_max_turn_rate=0.5, alpha=0.35,
                forming_tolerance=0.3, forming_max_steps=2500,
                forming_w_slot=2.0, forming_w_repulse=1.5,
                forming_repulse_dist=1.2, forming_slow_radius=1.5,
                goal=None, start=None, timesteps=6000,
                goal_reach_dist=0.5, slow_margin=0.25, slow_min=0.4,
                lookahead=1.0, avoid_vel_frac=0.25, pred_lead=0.5,
                pred_dt=0.1, servo=False, verbose=True,
                formation="square"):
    """Run the full mission; returns (rows, summary). rows are CSV dicts."""
    rng = np.random.default_rng(seed)
    # movers_json: the mover models exist in the SDF only so Gazebo can
    # teleport them; their geometry comes from the JSON contract. Without the
    # skip, each mover's SPAWN pose would enter the static map as a phantom
    # obstacle (the same SkipModels contract the MATLAB side uses).
    world = world_from_sdf(world_path, movers_json=contract_path)
    movers, _ = load_dynamic_obstacles(contract_path)
    nM = len(movers)

    cfg = VfhConfig(n_beams=n_beams, pred_lead=pred_lead, pred_dt=pred_dt,
                    sense={"kind": sense, "sigma0": sigma0, "k_rng": k_rng,
                           "fov_deg": fov_deg, "max_range": max_range})
    eps = 1e-6

    if goal is None:
        goal = np.array(world.goal, dtype=float)
    if start is None:
        start = np.array([-12.0, 0.0])
    goal = np.asarray(goal, dtype=float)
    start = np.asarray(start, dtype=float)

    # formation shape -> slot count and circumradius (max slot distance from
    # the centroid); works for every supported shape.
    offsets = _formation_offsets(formation, 3 if formation == "triangle" else 4,
                                 grid_spacing)
    K = len(offsets)
    half_diag = float(np.linalg.norm(offsets, axis=1).max())

    # start ring + Hungarian slot assignment (identical to MATLAB)
    from .assign_slots import assign_slots
    heading0 = math.atan2(goal[1] - start[1], goal[0] - start[0])
    R0 = np.array([[math.cos(heading0), -math.sin(heading0)],
                   [math.sin(heading0), math.cos(heading0)]])
    target_slots = (R0 @ offsets.T).T + start
    starts = start + 2.2 * np.column_stack([
        np.cos(2 * math.pi * np.arange(K) / K),
        np.sin(2 * math.pi * np.arange(K) / K)])
    cost = np.linalg.norm(starts[:, None, :] - target_slots[None, :, :], axis=2)
    perm = assign_slots(cost)
    offsets = offsets[perm]
    robot_offsets = offsets.copy()

    rows = []

    def log_row(t, phase, c, theta, dist, clr, pos):
        row = {
            "t": t, "phase": phase,
            "leader_x": c[0], "leader_y": c[1], "leader_yaw": theta,
            "dist_to_goal": dist, "clearance": clr,
        }
        for i in range(K):
            row[f"r{i+1}_x"] = pos[i, 0]
            row[f"r{i+1}_y"] = pos[i, 1]
        rows.append(row)

    def eval_all(t):
        return [m.eval_at(t) for m in movers]

    def wm_with(mt):
        return world.with_movers(mt)

    # ---------------- Phase 1: forming ----------------
    pos = starts.copy()
    theta = np.zeros(K)
    for i in range(K):
        d0 = target_slots[perm[i]] - pos[i]
        theta[i] = math.atan2(d0[1], d0[0])
    # NOTE: perm maps robot -> slot; target_slots[perm[i]] is robot i's slot.

    sim_t = 0.0
    formed = False
    t0_wall = time.perf_counter()
    comp_ms = 0.0

    for step in range(forming_max_steps):
        sim_t += dt
        mt = eval_all(sim_t)
        comp_ms = 0.0
        new_pos = pos.copy()
        # slots in world frame (static frame at start for forming)
        slots = (R0 @ offsets.T).T + start
        errs = np.linalg.norm(slots - pos, axis=1)
        for i in range(K):
            e_i = float(errs[i])
            # APF: slot attraction + mate repulsion (cascaded input)
            f = forming_w_slot * (slots[i] - pos[i]) / (e_i + eps)
            for j in range(K):
                if j == i:
                    continue
                dv = pos[i] - pos[j]
                d = float(np.linalg.norm(dv))
                if d < forming_repulse_dist:
                    f += forming_w_repulse * (forming_repulse_dist - d) * dv / (d + eps)
            virtual = pos[i] + f
            vec, cms = steering_point(pos[i], virtual, forming_w_slot, wm_with(mt),
                                      mt, sim_t, robot_radius, cfg, algo, eps,
                                      theta[i], rng)
            comp_ms += cms
            h_raw = math.atan2(vec[1], vec[0])
            h_des = theta[i] + alpha * wrap(h_raw - theta[i])
            dh = max(-forming_max_turn_rate, min(forming_max_turn_rate,
                                                 wrap(h_des - theta[i])))
            theta[i] = wrap(theta[i] + dh)
            v = forming_V * min(1.0, e_i / forming_slow_radius)
            cand = pos[i] + v * np.array([math.cos(theta[i]),
                                          math.sin(theta[i])]) * dt
            clr, hit = formation_clearance(cand, theta[i], np.zeros((1, 2)),
                                           wm_with(mt), robot_radius, "slots")
            if not hit:
                new_pos[i] = cand
            pos = new_pos
            errs = np.linalg.norm(slots - pos, axis=1)

        corners = pos  # scattered robots, log their own poses
        clr_min = float(world.distance(pos).min()) - robot_radius
        dist_cg = float(np.linalg.norm(pos.mean(axis=0) - goal))
        log_row(sim_t, 1, pos.mean(axis=0), float(np.arctan2(
            np.sin(theta).mean(), np.cos(theta).mean())), dist_cg,
            max(clr_min, 0.0), pos)
        if float(errs.max()) < forming_tolerance:
            formed = True
            break

    if not formed and verbose:
        print(f"WARNING: formation not converged "
              f"(max slot error {float(errs.max()):.3f} m)", file=sys.stderr)

    # ---------------- Phase 2: rigid transport ----------------
    c = pos.mean(axis=0)
    theta_l = heading0
    if servo:
        servo_pos = pos.copy()
        servo_yaw = theta.copy()
    reached_all = True
    comp_hist = []
    stuck = 0
    escape_step = 0.4
    stuck_limit = 5
    escape_commit = 0          # steps of committed escape translation left
    escape_heading = 0.0       # heading the commit slides along
    COMMIT_STEPS = 40          # 40 * V * dt = 2.4 m of slide per escape
    last_escape_heading = None  # diversify successive escape probes
    # Progress-window stall detector: the rejected-step counter above cannot
    # see a stall made of ACCEPTED micro-steps (oscillating between two
    # shelves accepts every step yet nets zero progress -- the E5 line
    # pivot). Measure net progress along the start->goal axis instead.
    u_sg = (goal - start) / max(float(np.linalg.norm(goal - start)), 1e-9)
    progress_win = []          # dot(c, u_sg) history, one entry per step
    PROG_WINDOW = 600          # 600 steps = 60 s of history
    PROG_MIN = 0.30            # m of net axis-progress expected per window
    last_progress_probe = -10**9
    # The anti-deadlock layer (progress-window probe + committed escape
    # slide) belongs to the PROPOSED transport stack, not to the baselines:
    # apf must keep its textbook local-minimum failure mode (E1's baseline
    # demonstration, identical on the MATLAB reference), while the VFH-family
    # laws -- whose inflated kernels can otherwise pivot forever in front of
    # a gap -- get the commit mechanism. Matches Simulation8_dynamic.m, where
    # the escape machinery serves the histogram stack.
    ALGO_HAS_ANTIDEADLOCK = algo.lower() != "apf"

    for wp_goal in [goal]:
        reached = False
        for step in range(timesteps):
            sim_t += dt
            mt = eval_all(sim_t)
            wm = wm_with(mt)

            progress_win.append(float(np.dot(c, u_sg)))
            if len(progress_win) > PROG_WINDOW:
                progress_win.pop(0)
            if (ALGO_HAS_ANTIDEADLOCK
                    and len(progress_win) >= PROG_WINDOW
                    and step - last_progress_probe > COMMIT_STEPS
                    and progress_win[-1] - progress_win[0] < PROG_MIN):
                # 60 s without net progress: force the escape probe + commit
                # regardless of whether steps were accepted or rejected.
                last_progress_probe = step
                goal_brg = math.atan2(wp_goal[1] - c[1], wp_goal[0] - c[0])
                cands = []
                for k in range(36):
                    for sgn in (1, -1):
                        h_try = wrap(theta_l + sgn * 2 * math.pi * k / 36)
                        cands.append((abs(wrap(h_try - goal_brg)), h_try))
                cands.sort(key=lambda t: t[0])
                for _, h_try in cands:
                    if (last_escape_heading is not None
                            and abs(wrap(h_try - last_escape_heading)) < 0.5):
                        continue        # diversify: don't repeat the last one
                    c_try = c + escape_step * np.array([
                        math.cos(h_try), math.sin(h_try)])
                    _, hit_try = formation_clearance(
                        c_try, h_try, robot_offsets, wm, robot_radius,
                        "payload")
                    if not hit_try:
                        escape_heading = h_try
                        last_escape_heading = h_try
                        escape_commit = COMMIT_STEPS
                        theta_l = h_try
                        break

            vec_goal = (wp_goal - c) / (np.linalg.norm(wp_goal - c) + eps)
            v_self = V * np.array([math.cos(theta_l), math.sin(theta_l)])
            margin = half_diag + robot_radius

            if escape_commit > 0:
                # COMMITTED ESCAPE: slide along the heading the probe
                # validated (re-validated every step against the moving
                # world). Overriding steering for a few steps is what turns
                # the escape rotation into actual progress -- without this,
                # an inflated-kernel steering law can reject every proposal
                # forever while the frame rotates in place (E5 line pivot).
                cand = c + V * dt * np.array([
                    math.cos(escape_heading), math.sin(escape_heading)])
                _, hit_n = formation_clearance(cand, escape_heading,
                                               robot_offsets, wm, robot_radius,
                                               "payload")
                mt_next = eval_all(sim_t + dt)
                _, hit_p = formation_clearance(cand, escape_heading,
                                               robot_offsets, wm_with(mt_next),
                                               robot_radius, "payload")
                if hit_n or hit_p:
                    escape_commit = 0     # blocked mid-slide; resume steering
                else:
                    c = cand
                    theta_l = escape_heading
                    escape_commit -= 1
                    stuck = 0
            else:
                vec_steer, cms = steering_rigid(c, wp_goal, w_att, world, mt, sim_t,
                                                margin, cfg, algo, eps, theta_l,
                                                v_self, rng)
                comp_hist.append(cms)
                total = w_goal * vec_goal + w_vfh * vec_steer
                h_raw = math.atan2(total[1], total[0])

                # speed slowdown on the predicted mover gap
                gap_pre, gap_now = _predicted_gap(c, v_self, movers, sim_t, dt,
                                                  lookahead)
                thr = slow_margin + robot_radius
                fastest = max((float(np.linalg.norm(m.v)) for m in mt), default=0.0)
                if nM and gap_pre < thr and fastest / V > avoid_vel_frac:
                    scale = max(slow_min, min(1.0, 0.8 * gap_pre / thr))
                else:
                    scale = 1.0

                h_des = theta_l + alpha * wrap(h_raw - theta_l)
                dw = max(-max_turn_rate, min(max_turn_rate, wrap(h_des - theta_l)))
                theta_new = wrap(theta_l + dw)
                cand = c + scale * V * np.array([math.cos(theta_new),
                                                 math.sin(theta_new)]) * dt

                clr_n, hit_n = formation_clearance(cand, theta_new, robot_offsets,
                                                   wm, robot_radius, "payload")
                mt_next = eval_all(sim_t + dt)
                clr_p, hit_p = formation_clearance(cand, theta_new, robot_offsets,
                                                   wm_with(mt_next), robot_radius,
                                                   "payload")
                if not hit_n and not hit_p:
                    c = cand
                    theta_l = theta_new
                    stuck = 0
                else:
                    # Step rejected. After stuck_limit consecutive rejections,
                    # probe escape headings: the first heading (sorted by
                    # deviation from the goal bearing) along which the WHOLE body
                    # can translate one escape_step right now. This is the
                    # headless equivalent of find_escape_heading_dyn (without the
                    # mover hold-horizon refinement): a body blocked ahead is not
                    # itself in collision, so the probe must test TRANSLATION.
                    stuck += 1
                    if stuck > stuck_limit:
                        # Probe escape headings over BOTH senses, ordered by
                        # deviation from the GOAL bearing (this is what the old
                        # comment claimed and the old loop never did -- it probed
                        # from the current heading, so the frame could spin).
                        # The probe then COMMITS: the next COMMIT_STEPS steps
                        # translate along the validated heading (re-validated
                        # each step), so a rotated hull actually clears the
                        # contested zone instead of re-asking a steering law
                        # that keeps rejecting it -- that re-ask loop was the
                        # endless pivot of the E5 line formation.
                        goal_brg = math.atan2(wp_goal[1] - c[1], wp_goal[0] - c[0])
                        cands = []
                        for k in range(36):
                            for sgn in (1, -1):
                                h_try = wrap(theta_l + sgn * 2 * math.pi * k / 36)
                                dev = abs(wrap(h_try - goal_brg))
                                cands.append((dev, k, sgn, h_try))
                        cands.sort(key=lambda t: t[0])
                        found_h = None
                        for _, _, _, h_try in cands:
                            c_try = c + escape_step * np.array([
                                math.cos(h_try), math.sin(h_try)])
                            _, hit_try = formation_clearance(
                                c_try, h_try, robot_offsets, wm, robot_radius,
                                "payload")
                            if not hit_try:
                                found_h = h_try
                                break
                        if found_h is not None:
                            escape_heading = found_h
                            last_escape_heading = found_h
                            theta_l = found_h
                            escape_commit = COMMIT_STEPS
                            stuck = 0
                        else:
                            stuck = stuck_limit   # retry next step

            if servo:
                # P-servo the real robots to their frame slots
                R = np.array([[math.cos(theta_l), -math.sin(theta_l)],
                              [math.sin(theta_l), math.cos(theta_l)]])
                slots = (R @ robot_offsets.T).T + c
                v_frame = scale * V
                for i in range(K):
                    e = slots[i] - servo_pos[i]
                    en = float(np.linalg.norm(e))
                    if en < 1e-6:
                        continue
                    e_hat = e / en
                    v_cmd = v_frame * e_hat[0] + min(2.0 * en, 0.6)
                    h_err = wrap(math.atan2(e[1], e[0]) - servo_yaw[i])
                    w_cmd = max(-max_turn_rate, min(max_turn_rate, 1.5 * h_err))
                    servo_yaw[i] = wrap(servo_yaw[i] + w_cmd * dt)
                    servo_pos[i] = servo_pos[i] + v_cmd * np.array([
                        math.cos(servo_yaw[i]), math.sin(servo_yaw[i])]) * dt
                log_pos = servo_pos
            else:
                R = np.array([[math.cos(theta_l), -math.sin(theta_l)],
                              [math.sin(theta_l), math.cos(theta_l)]])
                log_pos = (R @ robot_offsets.T).T + c

            clr_now, _ = formation_clearance(c, theta_l, robot_offsets, wm,
                                             robot_radius, "payload")
            dist = float(np.linalg.norm(wp_goal - c))
            log_row(sim_t, 2, c, theta_l, dist, clr_now, log_pos)
            if dist < goal_reach_dist:
                reached = True
                break

        if not reached:
            reached_all = False

    # ---------------- summary ----------------
    p2 = [r for r in rows if r["phase"] == 2]
    S = {
        "reached_goal": reached_all,
        "total_time": rows[-1]["t"] if rows else 0.0,
        "transport_time": len(p2) * dt,
        "path_length": _path_len(p2),
        "min_clearance": min((r["clearance"] for r in p2), default=float("nan")),
        "collisions": sum(1 for r in p2 if r["clearance"] <= robot_radius),
        "comp_ms_mean": float(np.mean(comp_hist)) if comp_hist else 0.0,
        "wall_s": time.perf_counter() - t0_wall,
    }
    S["path_efficiency"] = _path_eff(p2)
    return rows, S


def _path_len(p2):
    if len(p2) < 2:
        return 0.0
    a = np.array([[r["leader_x"], r["leader_y"]] for r in p2])
    return float(np.hypot(*np.diff(a, axis=0).T).sum())


def _path_eff(p2):
    if len(p2) < 2:
        return 0.0
    a = np.array([[r["leader_x"], r["leader_y"]] for r in p2])
    straight = float(np.hypot(*(a[-1] - a[0])))
    return straight / max(_path_len(p2), 1e-9)


def _predicted_gap(c, v_self, movers, t_now, dt, lookahead):
    if not movers:
        return 99.0, 99.0
    gap_now = _mover_gap(c, [m.eval_at(t_now) for m in movers])
    worst = gap_now
    T = dt
    while T <= lookahead + 1e-9:
        mf = [m.eval_at(t_now + T) for m in movers]
        g = _mover_gap(c + v_self * T, mf) - float(np.linalg.norm(v_self)) * T
        worst = min(worst, g)
        T += dt
    return worst, gap_now


def _mover_gap(q, movers):
    g = float("inf")
    for m in movers:
        if m.shape == "circle":
            dk = np.hypot(q[0] - m.c[0], q[1] - m.c[1]) - m.r
        else:
            dx = abs(q[0] - m.c[0]) - m.size[0] / 2
            dy = abs(q[1] - m.c[1]) - m.size[1] / 2
            dk = np.hypot(max(dx, 0), max(dy, 0)) + min(max(dx, dy), 0)
        g = min(g, float(dk))
    return g


def write_csv(rows, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    hdr = ["t", "phase", "leader_x", "leader_y", "leader_yaw", "dist_to_goal",
           "clearance"]
    K = 0
    if rows:
        K = sum(1 for k in rows[0] if k.endswith("_x"))
    hdr += [f"r{i+1}_x" for i in range(K)] + [f"r{i+1}_y" for i in range(K)]
    hdr += ["mover_vx", "mover_vy", "speed_scale", "mover_gap",
            "mover_gap_pred", "comp_ms"]
    with open(path, "w") as fh:
        fh.write(",".join(hdr) + "\n")
        for r in rows:
            vals = [r[h] if h in r else 0.0 for h in hdr]
            fh.write(",".join(f"{float(v):.6f}" for v in vals) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="worlds/swarm_world_dynamic.sdf")
    ap.add_argument("--contract", default="worlds/dynamic_obstacles.json")
    ap.add_argument("--algo", default="pvfh",
                    choices=["apf", "vfh", "pvfh", "pvfh_prob"])
    ap.add_argument("--sense", default="lidar", choices=["lidar", "tof"])
    ap.add_argument("--n-beams", type=int, default=288)
    ap.add_argument("--sigma0", type=float, default=0.02)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--grid-spacing", type=float, default=1.18)
    ap.add_argument("--robot-radius", type=float, default=0.18)
    ap.add_argument("--servo", action="store_true",
                    help="P-servo real robot states to their slots")
    ap.add_argument("--formation", default="square",
                    choices=["square", "line", "triangle", "diamond", "v"],
                    help="slot layout (square = 2x2 payload reference)")
    ap.add_argument("--output", default="logs/ros2_dynamic.csv")
    args = ap.parse_args(argv)

    rows, S = run_mission(args.world, args.contract, algo=args.algo,
                          sense=args.sense, n_beams=args.n_beams,
                          sigma0=args.sigma0, seed=args.seed,
                          grid_spacing=args.grid_spacing,
                          robot_radius=args.robot_radius, servo=args.servo,
                          formation=args.formation)
    write_csv(rows, args.output)
    print(f"wrote {args.output}: reached={S['reached_goal']} "
          f"path={S['path_length']:.2f} m eff={S['path_efficiency']:.2f} "
          f"minclr={S['min_clearance']:.3f} m collisions={S['collisions']} "
          f"comp={S['comp_ms_mean']:.2f} ms/step wall={S['wall_s']:.1f} s")
    return 0 if S["reached_goal"] else 1


if __name__ == "__main__":
    sys.exit(main())
