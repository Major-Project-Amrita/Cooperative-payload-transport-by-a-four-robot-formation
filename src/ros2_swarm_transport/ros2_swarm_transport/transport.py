"""The two mission phases as step-able objects (Simulation8_dynamic.m port).

FormingPhase   -- phase 1: every robot seeks its slot (APF virtual goal ->
                  steering dispatch), convoy queueing behind compact static
                  obstacles, stuck escape, back-off, t / t+dt commit.
TransportLeader -- phase 2: the rigid frame. Atomic pose commit with
                  rotate-only fallback and wedge recovery, steered (rate-
                  limited) escape commits, anti-deadlock progress window,
                  mover slowdown, localisation error, command loss, the
                  load-carrying motion limits, blend fade.

Both are pure kinematics on numpy state, with no ROS and no simulator: the
headless runner, the ROS 2 controller and the MuJoCo physics validation all
drive the SAME objects, so there is one implementation of the mission logic.
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import formation_clearance, movers_distance
from .vfh import VfhConfig, steering_point, steering_rigid


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot2d(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


DEFAULTS = dict(
    dt=0.1, robot_radius=0.18, V=0.6, w_goal=1.0, w_vfh=2.0, w_att=1.0,
    max_turn_rate=0.3, alpha=0.35, goal_reach_dist=0.5, timesteps=3000,
    stuck_escape_steps=5, escape_step=0.4, eps=1e-6, blend_fade=True,
    collision_mode="payload",
    gate="exact",          # "matlab" = bug-compatible SAT (see geometry.py)
    # dynamic obstacles
    slow_margin=0.25, slow_min=0.4, hold_time=1.5, lookahead=1.0,
    avoid_vel_frac=0.25,
    # robustness perturbations
    loc_sigma_pos=0.0, loc_sigma_yaw=0.0, comm_drop=0.0,
    # load-carrying motion limits (E14), off by default
    kin=False, kin_v_robot_max=1.0, kin_yaw_acc_max=2.5, kin_lat_acc_max=0.8,
    kin_v_floor_frac=0.5, kin_brake_gain=0.8,
    # anti-deadlock layer
    adl=True, adl_window=150, adl_min_progress=0.30, adl_commit_steps=40,
    adl_diversify=0.5,
    # forming
    forming_tolerance=0.3, forming_max_steps=2500, forming_w_slot=2.0,
    forming_w_repulse=1.5, forming_repulse_dist=1.2,
    forming_max_turn_rate=0.5, forming_V=0.5, forming_slow_radius=1.5,
    forming_alpha=0.35, forming_stuck_escape_steps=5,
    forming_backoff_dist=0.3, dock_tau=1.0,
    convoy_trigger_dist=2.0, convoy_exit_dist=3.0, convoy_max_obstacle=2.0,
    convoy_min_group=2, convoy_spacing=1.0, trail_max_len=4000,
)


def make_params(**over):
    P = dict(DEFAULTS)
    unknown = set(over) - set(P)
    if unknown:
        raise KeyError(f"unknown mission parameters: {sorted(unknown)}")
    P.update(over)
    return P


# ---------------------------------------------------------------------------
# shared helpers
# ---------------------------------------------------------------------------

def eval_movers(movers, t):
    return [m.eval_at(t) for m in movers]


def step_safe(cand, heading, world, movers, t_now, dt, robot_radius, mode,
              offsets, movers_now=None, sat="exact"):
    """step_safe_dyn.m: the body fits at cand now AND at t_now + dt."""
    if movers_now is None:
        movers_now = eval_movers(movers, t_now)
    _, hit = formation_clearance(cand, heading, offsets, world, robot_radius,
                                 mode, movers=movers_now, sat=sat)
    if hit:
        return False
    if movers:
        _, hit = formation_clearance(cand, heading, offsets, world,
                                     robot_radius, mode,
                                     movers=eval_movers(movers, t_now + dt),
                                     sat=sat)
    return not hit


def mover_gap(q, movers):
    if not movers:
        return float("inf")
    return float(movers_distance(np.atleast_2d(q), movers).min())


def predicted_min_mover_gap(q, v_self, movers, t_now, dt, lookahead):
    """predicted_min_mover_gap.m: worst gap over the lookahead, with the
    self-displacement subtracted. (99, 99) when there are no movers."""
    if not movers:
        return 99.0, 99.0
    gap_now = mover_gap(q, eval_movers(movers, t_now))
    worst = gap_now
    speed = float(np.linalg.norm(v_self))
    T = dt
    while T <= lookahead + 1e-9:
        g = mover_gap(q + v_self * T, eval_movers(movers, t_now + T)) - speed * T
        worst = min(worst, g)
        T += dt
    return worst, gap_now


def find_escape_heading(pos, world, movers, t_now, offsets, robot_radius,
                        preferred, num_probe, mode, step, T_hold, dt,
                        avoid_heading=None, avoid_width=0.5, sat="exact"):
    """find_escape_heading_dyn.m: headings ordered by deviation from
    `preferred`; the first along which the body can translate one `step`
    (pointing that way) and hold the pose for T_hold while the movers move."""
    angles = -math.pi + 2 * math.pi * np.arange(num_probe) / num_probe
    devs = np.abs(wrap(angles - preferred))
    order = np.argsort(devs, kind="stable")
    n_hold = int(round(T_hold / dt))
    hold = [eval_movers(movers, t_now + j * dt) for j in range(n_hold + 1)] \
        if movers else [[]]
    for oi in order:
        th = float(angles[oi])
        if avoid_heading is not None and abs(wrap(th - avoid_heading)) < avoid_width:
            continue
        cand = pos + step * np.array([math.cos(th), math.sin(th)])
        ok = True
        for mk in hold:
            _, hit = formation_clearance(cand, th, offsets, world,
                                         robot_radius, mode, movers=mk,
                                         sat=sat)
            if hit:
                ok = False
                break
        if ok:
            return True, th
    return False, float(preferred)


# ---------------------------------------------------------------------------
# phase 1
# ---------------------------------------------------------------------------

class FormingPhase:
    def __init__(self, world, movers, slots, P, cfg: VfhConfig, algo, rng):
        self.world, self.movers, self.P = world, movers, P
        self.slots = np.asarray(slots, dtype=float)
        self.cfg, self.algo, self.rng = cfg, algo, rng
        K = len(self.slots)
        self.K = K
        ext = np.array([2 * float(max(b.h)) for b in world.boxes]
                       + [2 * c.r for c in world.circles])
        self.convoy_ok = ext <= P["convoy_max_obstacle"]
        self.n_static = len(ext)
        self.stuck = np.zeros(K, dtype=int)
        self.in_convoy = np.full(K, -1)
        self.trail = [np.zeros((0, 2)) for _ in range(self.n_static)]
        self.errs = np.full(K, np.inf)

    def step(self, pos, heading, sim_t):
        """One 10 Hz forming step. Returns (new_pos, new_heading, comp_ms)."""
        P, world = self.P, self.world
        K, r, dt, eps = self.K, P["robot_radius"], P["dt"], P["eps"]
        mt = eval_movers(self.movers, sim_t)
        wm = world.with_movers(mt) if mt else world
        new_pos = pos.copy()
        heading = heading.copy()
        comp_ms = 0.0

        nearest_d, nearest_id = wm.distance_owner(pos)
        new_in = np.full(K, -1)
        for c in np.where(self.convoy_ok)[0]:
            elig = np.where(((nearest_id == c) & (nearest_d < P["convoy_trigger_dist"]))
                            | ((self.in_convoy == c) & (nearest_d < P["convoy_exit_dist"])))[0]
            if len(elig) >= P["convoy_min_group"]:
                new_in[elig] = c
        self.in_convoy = new_in
        leader_of = np.full(K, -1)
        for c in set(new_in[new_in >= 0].tolist()):
            members = np.where(new_in == c)[0]
            leader_of[members] = members.min()

        for i in range(K):
            d_slot = self.slots[i] - pos[i]
            goal_h = math.atan2(d_slot[1], d_slot[0])
            e_i = float(np.linalg.norm(d_slot))
            speed = P["forming_V"] * min(1.0, e_i / P["forming_slow_radius"])
            c = int(self.in_convoy[i])
            if c >= 0 and leader_of[i] != i:
                lead = int(leader_of[i])
                followers = [m for m in np.where(self.in_convoy == c)[0] if m != lead]
                d_lead = [float(np.linalg.norm(pos[f] - pos[lead])) for f in followers]
                ranked = [followers[k] for k in np.argsort(d_lead, kind="stable")]
                rank = ranked.index(i) + 1
                tgt = _point_along_trail_behind(
                    self.trail[c], rank * P["convoy_spacing"], pos[lead], pos[i])
                h_raw = math.atan2(tgt[1] - pos[i, 1], tgt[0] - pos[i, 0])
                speed = P["forming_V"]
            elif self.stuck[i] > P["forming_stuck_escape_steps"]:
                found, eh = find_escape_heading(
                    pos[i], world, self.movers, sim_t, np.zeros((1, 2)), r,
                    goal_h, 36, "slots", P["forming_backoff_dist"],
                    P["hold_time"], dt)
                h_raw = eh if found else float(heading[i])
            else:
                f = P["forming_w_slot"] * d_slot / (e_i + eps)
                rep = np.zeros(2)
                for j in range(K):
                    if j == i or (c >= 0 and self.in_convoy[j] == c):
                        continue
                    dv = pos[i] - pos[j]
                    d = float(np.linalg.norm(dv))
                    if d < P["forming_repulse_dist"]:
                        rep += dv / (d + eps) * (P["forming_repulse_dist"] - d)
                virtual = pos[i] + f + P["forming_w_repulse"] * rep
                vec, cms = steering_point(
                    pos[i], virtual, P["forming_w_slot"], world, self.movers,
                    sim_t, r, self.cfg, self.algo, eps, float(heading[i]),
                    self.rng, V=P["forming_V"])
                comp_ms += cms
                h_raw = math.atan2(vec[1], vec[0])

            h_des = wrap(heading[i] + P["forming_alpha"] * wrap(h_raw - heading[i]))
            dh = max(-P["forming_max_turn_rate"],
                     min(P["forming_max_turn_rate"], wrap(h_des - heading[i])))
            heading[i] = wrap(heading[i] + dh)
            u = np.array([math.cos(heading[i]), math.sin(heading[i])])
            cand = pos[i] + speed * u * dt
            if step_safe(cand, heading[i], world, self.movers, sim_t, dt, r,
                         "slots", np.zeros((1, 2)), mt):
                new_pos[i] = cand
                self.stuck[i] = 0
            else:
                self.stuck[i] += 1
                back = pos[i] - P["forming_backoff_dist"] * u
                if float(wm.distance(back[None, :])[0]) > r:
                    new_pos[i] = back
            if c >= 0 and leader_of[i] == i:
                self.trail[c] = np.vstack([self.trail[c], new_pos[i]])[-P["trail_max_len"]:]

        self.errs = np.linalg.norm(new_pos - self.slots, axis=1)
        return new_pos, heading, comp_ms

    @property
    def formed(self):
        return float(self.errs.max()) < self.P["forming_tolerance"]


def _point_along_trail_behind(trail, arc, leader_pos, from_pos):
    if len(trail) < 2:
        d = leader_pos - from_pos
        dn = float(np.linalg.norm(d))
        if dn < 1e-6:
            return leader_pos - np.array([arc, 0.0])
        return leader_pos - arc * d / dn
    remaining = arc
    pt = trail[-1]
    for k in range(len(trail) - 1, 0, -1):
        seg = trail[k - 1] - trail[k]
        L = float(np.linalg.norm(seg))
        if L < 1e-9:
            continue
        if remaining <= L:
            return trail[k] + (remaining / L) * seg
        remaining -= L
        pt = trail[k - 1]
    return pt


# ---------------------------------------------------------------------------
# phase 2
# ---------------------------------------------------------------------------

class TransportLeader:
    def __init__(self, world, movers, robot_offsets, P, cfg: VfhConfig, algo,
                 rng, pos, heading):
        self.world, self.movers, self.P = world, movers, P
        self.offsets = np.asarray(robot_offsets, dtype=float)
        self.cfg, self.algo, self.rng = cfg, algo.lower(), rng
        self.half_diag = float(np.linalg.norm(self.offsets, axis=1).max())
        self.pos = np.asarray(pos, dtype=float).copy()
        self.heading = float(heading)
        self.stuck = 0
        self.yaw_rate_prev = 0.0
        self.escape_commit = 0
        self.escape_heading = 0.0
        self.last_escape = None
        self.last_cmd = None
        self.goal = None

    def set_goal(self, goal):
        """Start a waypoint leg: reset the progress window on the new axis."""
        self.goal = np.asarray(goal, dtype=float)
        d = self.goal - self.pos
        self.prog_origin = self.pos.copy()
        self.prog_axis = d / max(float(np.linalg.norm(d)), self.P["eps"])
        self.prog_hist = []
        self.last_probe_step = -10 ** 9
        self.t_leg = 0

    def _safe(self, pos, heading, sim_t, mt):
        P = self.P
        return step_safe(pos, heading, self.world, self.movers, sim_t, P["dt"],
                         P["robot_radius"], P["collision_mode"], self.offsets,
                         mt, P["gate"])

    def step(self, sim_t):
        """One 10 Hz transport step at absolute time sim_t. Returns a dict of
        the step's flags and metrics; self.pos / self.heading are updated."""
        P, eps, dt, V = self.P, self.P["eps"], self.P["dt"], self.P["V"]
        self.t_leg += 1
        t = self.t_leg
        goal = self.goal
        mt = eval_movers(self.movers, sim_t)
        margin = self.half_diag + P["robot_radius"]
        vec_goal = (goal - self.pos) / (float(np.linalg.norm(goal - self.pos)) + eps)
        goal_h = math.atan2(vec_goal[1], vec_goal[0])

        # ---- anti-deadlock: progress-window stall detector ----
        self.prog_hist.append(float(np.dot(self.pos - self.prog_origin, self.prog_axis)))
        if len(self.prog_hist) > P["adl_window"]:
            self.prog_hist.pop(0)
        want_probe = self.stuck > P["stuck_escape_steps"]
        avoid_h = None
        if (P["adl"] and self.escape_commit == 0
                and len(self.prog_hist) >= P["adl_window"]
                and t - self.last_probe_step > P["adl_commit_steps"]
                and self.prog_hist[-1] - self.prog_hist[0] < P["adl_min_progress"]):
            want_probe = True
            avoid_h = self.last_escape

        if self.escape_commit == 0 and want_probe:
            found, eh = find_escape_heading(
                self.pos, self.world, self.movers, sim_t, self.offsets,
                P["robot_radius"], goal_h, 36, P["collision_mode"],
                P["escape_step"], P["hold_time"], dt, avoid_h,
                P["adl_diversify"], P["gate"])
            self.last_probe_step = t
            if found:
                self.escape_heading = eh
                self.last_escape = eh
                self.escape_commit = P["adl_commit_steps"] if P["adl"] else 1
            else:
                self.stuck = 0

        comp_ms, near, near_pre, esc = 0.0, 99.0, 99.0, 0
        v_scale = 1.0
        if self.escape_commit > 0:
            h_raw = self.escape_heading
            esc = 1
            turn_gain = 1.0                 # rate limit only, no low-pass
        elif want_probe:
            h_raw = self.heading            # probe found nothing: hold
            turn_gain = P["alpha"]
        else:
            v_self = V * np.array([math.cos(self.heading), math.sin(self.heading)])
            goal_b, vec_goal_b = goal, vec_goal
            if P["loc_sigma_pos"] > 0 or P["loc_sigma_yaw"] > 0:
                e_pos = P["loc_sigma_pos"] * self.rng.normal(size=2)
                e_yaw = P["loc_sigma_yaw"] * self.rng.normal()
                rel = rot2d(-e_yaw) @ (goal - (self.pos + e_pos))
                goal_b = self.pos + rel
                vec_goal_b = rel / max(float(np.linalg.norm(rel)), eps)
            vec_steer, comp_ms = steering_rigid(
                self.pos, goal_b, P["w_att"], self.world, self.movers, sim_t,
                margin, self.cfg, self.algo, eps, self.heading, v_self,
                self.rng, V=V)
            w_goal = P["w_goal"]
            ns = float(np.linalg.norm(vec_steer))
            if P["blend_fade"] and self.algo != "apf" and ns > eps:
                w_goal = P["w_goal"] * max(0.0, float(np.dot(vec_goal_b, vec_steer)) / ns)
            total = w_goal * vec_goal_b + P["w_vfh"] * vec_steer
            h_raw = math.atan2(total[1], total[0])

            near_pre, near = predicted_min_mover_gap(
                self.pos, v_self, self.movers, sim_t, dt, P["lookahead"])
            thr = P["slow_margin"] + P["robot_radius"]
            fastest = max((float(np.linalg.norm(m.v)) for m in mt), default=0.0)
            if self.movers and near_pre < thr and fastest / V > P["avoid_vel_frac"]:
                v_scale = max(P["slow_min"], min(1.0, 0.8 * near_pre / thr))
            turn_gain = P["alpha"]
            if (P["comm_drop"] > 0 and self.last_cmd is not None
                    and self.rng.random() < P["comm_drop"]):
                h_raw, v_scale = self.last_cmd
            else:
                self.last_cmd = (h_raw, v_scale)

        h_des = wrap(self.heading + turn_gain * wrap(h_raw - self.heading))
        dh = max(-P["max_turn_rate"], min(P["max_turn_rate"], wrap(h_des - self.heading)))
        if P["kin"]:
            w = dh / dt
            e_head = abs(wrap(h_raw - self.heading))
            w_brk = P["kin_brake_gain"] * math.sqrt(2 * P["kin_yaw_acc_max"] * e_head)
            v_flr = min(v_scale * V, P["kin_v_floor_frac"] * V)
            w_cap = min(w_brk, (P["kin_v_robot_max"] - v_flr) / max(self.half_diag, eps))
            w = max(min(w, w_cap), -w_cap)
            dw = P["kin_yaw_acc_max"] * dt
            w = max(min(w, self.yaw_rate_prev + dw), self.yaw_rate_prev - dw)
            v_lim = max(v_flr, P["kin_v_robot_max"] - abs(w) * self.half_diag)
            if abs(w) > eps:
                v_lim = min(v_lim, P["kin_lat_acc_max"] / abs(w))
            v_scale = min(v_scale, v_lim / V)
            dh = w * dt
        cand_h = wrap(self.heading + dh)
        h_before = self.heading
        u = np.array([math.cos(cand_h), math.sin(cand_h)])
        new_pos = self.pos + v_scale * V * u * dt

        # ---- atomic pose commit ----
        if self._safe(new_pos, cand_h, sim_t, mt):
            self.pos, self.heading = new_pos, cand_h
            self.stuck = 0
            rej = 0
        elif abs(dh) > 0 and self._safe(self.pos, cand_h, sim_t, mt):
            self.heading = cand_h
            self.stuck += 1
            rej = 1
        else:
            self.stuck += 1
            rej = 2
            if self.stuck > P["stuck_escape_steps"]:
                back = self.pos - 0.5 * V * dt * np.array(
                    [math.cos(self.heading), math.sin(self.heading)])
                other_h = wrap(self.heading - dh)
                if self._safe(back, cand_h, sim_t, mt):
                    self.pos, self.heading, rej = back, cand_h, 3
                elif self._safe(back, self.heading, sim_t, mt):
                    self.pos, rej = back, 3
                elif abs(dh) > 0 and self._safe(self.pos, other_h, sim_t, mt):
                    self.heading, rej = other_h, 3
        if self.escape_commit > 0:
            self.escape_commit -= 1
            if rej == 2:
                self.escape_commit = 0
            if rej == 0:
                self.stuck = 0
        self.yaw_rate_prev = wrap(self.heading - h_before) / dt

        clr, _ = formation_clearance(self.pos, self.heading, self.offsets,
                                     self.world, P["robot_radius"],
                                     P["collision_mode"], movers=mt,
                                     sat=P["gate"])
        clr_s, _ = formation_clearance(self.pos, self.heading, self.offsets,
                                       self.world, P["robot_radius"],
                                       P["collision_mode"], sat=P["gate"])
        dist = float(np.linalg.norm(goal - self.pos))
        return {"reject": rej, "escape": esc, "v_scale": v_scale,
                "comp_ms": comp_ms, "near": min(near, 99.0),
                "near_pre": min(near_pre, 99.0), "clearance": clr,
                "clearance_static": clr_s, "dist": dist,
                "reached": dist < P["goal_reach_dist"], "movers": mt,
                "yaw_rate": self.yaw_rate_prev}

    def slots(self):
        return (rot2d(self.heading) @ self.offsets.T).T + self.pos


# ---------------------------------------------------------------------------
# robot-level tracking laws (used by the physics validation and the ROS node)
# ---------------------------------------------------------------------------

def slot_tracking_cmd(p, yaw, p_ref, v_ref, kp=2.0, kw=4.0, v_max=1.0, w_max=3.0):
    """Unicycle command that tracks a moving reference point: desired planar
    velocity u = v_ref + kp (p_ref - p)."""
    return velocity_cmd(yaw, v_ref + _clip_norm(kp * (p_ref - p), 0.5), kw,
                        v_max, w_max)


def _clip_norm(v, m):
    """Bound a correction so position feedback cannot saturate the wheels
    (two saturated wheels have no steering authority left)."""
    n = float(np.linalg.norm(v))
    return v if n <= m else v * (m / n)


def velocity_cmd(yaw, u, kw=4.0, v_max=1.0, w_max=3.0):
    """Unicycle command for a desired planar velocity u: drive its component
    along the body axis (cos^2 taper, so a misaligned robot turns before it
    pushes) and turn toward it, backing up instead of spinning round when it
    points behind."""
    un = float(np.linalg.norm(u))
    if un < 0.02:
        return 0.0, 0.0
    err = wrap(math.atan2(u[1], u[0]) - yaw)
    fwd = 1.0
    if abs(err) > math.pi / 2:
        err, fwd = wrap(err + math.pi), -1.0
    v = fwd * un * max(0.0, math.cos(err)) ** 2
    return (max(-v_max, min(v_max, v)), max(-w_max, min(w_max, kw * err)))


# ---------------------------------------------------------------------------
# the whole mission as one controller: poses in, wheel-level commands out
# ---------------------------------------------------------------------------

class MissionController:
    """Forming -> docking -> pick-up -> transport, for real (or simulated)
    robots. step() takes the measured robot poses and returns one (v, w)
    command per robot. It owns no simulator and no middleware: the MuJoCo
    validation calls it in-process and the ROS 2 node calls it from a topic
    callback, so both run the identical mission.

    payload: "virtual" (no load), "on_top" (a plate pinned to every robot:
    the robots must then move as ONE rigid twist) or "behind" (towed sled).
    hold_err: formation feedback - the kinematic frame waits (at most
    HOLD_MAX steps in a row) when the robots are more than this far behind.
    """
    HOLD_MAX = 20
    DOCK_TOL, DOCK_YAW, DOCK_MAX = 0.03, 0.08, 120

    def __init__(self, world, movers, F, waypoints, P, cfg, algo, rng,
                 payload="virtual", hold_err=0.6):
        self.P, self.F, self.payload, self.hold_err = P, F, payload, hold_err
        self.K, self.dt = F["K"], P["dt"]
        self.waypoints = [np.asarray(w, dtype=float) for w in waypoints]
        self.forming = FormingPhase(world, movers, F["assigned_slots"], P, cfg, algo, rng)
        self.leader = TransportLeader(world, movers, F["robot_offsets"], P, cfg, algo, rng,
                                      F["assigned_slots"].mean(axis=0), F["heading0"])
        self.mode = "forming"            # forming | docking | transport | done
        self.n_form = self.n_dock = self.n_leg = self.n_hold = 0
        self.wp, self.new_leg = 0, True
        self.formed, self.reached_all, self.dock_err = False, None, float("nan")
        self.rigid = None                # plate-frame offset of the frame centre
        self.st, self.held = None, False
        self.hold_run = self.hold_cool = 0

    @property
    def phase(self):
        return 1 if self.mode in ("forming", "docking") else 2

    def slots(self):
        return self.F["assigned_slots"] if self.mode == "forming" else self.leader.slots()

    def step(self, sim_t, pos, yaw, plate=None):
        """One control tick at time sim_t (commands hold until sim_t + dt).
        pos (K,2), yaw (K,) measured; plate = (xy, yaw) of the carried body
        or None. Returns {"cmds": [(v, w)]*K, "attach": bool, "comp_ms"}.
        attach=True asks the plant to pick the load up BEFORE applying cmds."""
        P, K, dt = self.P, self.K, self.dt
        out = {"cmds": [(0.0, 0.0)] * K, "attach": False, "comp_ms": 0.0}
        h0 = self.F["heading0"]

        if self.mode == "forming":
            errs = np.linalg.norm(pos - self.F["assigned_slots"], axis=1)
            done = self.n_form > 0 and float(errs.max()) < P["forming_tolerance"]
            if done or self.n_form >= P["forming_max_steps"]:
                self.formed, self.mode = bool(done), "docking"
            else:
                new_pos, new_yaw, out["comp_ms"] = self.forming.step(pos, yaw, sim_t + dt)
                cmds = []
                for i in range(K):
                    u = np.array([math.cos(new_yaw[i]), math.sin(new_yaw[i])])
                    cmds.append((float(np.dot(new_pos[i] - pos[i], u)) / dt,
                                 wrap(new_yaw[i] - yaw[i]) / dt))
                out["cmds"] = cmds
                self.n_form += 1
                return out

        if self.mode == "docking":
            slots = self.leader.slots()
            errs = np.linalg.norm(pos - slots, axis=1)
            aligned = np.abs(wrap(yaw - h0)) < self.DOCK_YAW
            if (float(errs.max()) < self.DOCK_TOL and aligned.all()) \
                    or self.n_dock >= self.DOCK_MAX:
                self.dock_err = float(errs.max())
                self.mode, out["attach"] = "transport", self.payload != "virtual"
                if self.payload == "on_top" and plate is not None:
                    c, s = math.cos(plate[1]), math.sin(plate[1])
                    Rp = np.array([[c, -s], [s, c]])
                    self.rigid = (Rp.T @ (self.leader.pos - np.asarray(plate[0])),
                                  wrap(plate[1] - self.leader.heading))
            else:
                cmds = []
                for i in range(K):
                    if errs[i] > self.DOCK_TOL:
                        cmds.append(slot_tracking_cmd(pos[i], yaw[i], slots[i],
                                                      np.zeros(2), v_max=0.3))
                    else:
                        cmds.append((0.0, max(-1.5, min(1.5, 3.0 * wrap(h0 - yaw[i])))))
                out["cmds"] = cmds
                self.n_dock += 1
                return out

        if self.mode == "transport":
            leader = self.leader
            if self.new_leg:
                leader.set_goal(self.waypoints[self.wp])
                self.new_leg, self.n_leg = False, 0
            prev_slots, prev_h, prev_c = leader.slots(), leader.heading, leader.pos.copy()
            e_now = float(np.linalg.norm(pos - prev_slots, axis=1).max())
            held = (self.hold_err > 0 and e_now > self.hold_err and self.st is not None
                    and self.hold_run < self.HOLD_MAX and self.hold_cool == 0)
            if held:
                self.hold_run += 1
                if self.hold_run == self.HOLD_MAX:
                    self.hold_cool = self.HOLD_MAX
                self.n_hold += 1
                self.st = dict(self.st, reject=0, escape=0, comp_ms=0.0, reached=False)
            else:
                self.hold_run, self.hold_cool = 0, max(0, self.hold_cool - 1)
                self.st = leader.step(sim_t + dt)
            self.held = held
            slots = leader.slots()
            if self.rigid is None or plate is None:
                v_ref = (slots - prev_slots) / dt
                out["cmds"] = [slot_tracking_cmd(pos[i], yaw[i], slots[i], v_ref[i])
                               for i in range(K)]
            else:
                # Pinned to one rigid plate: every robot gets v + w x r of ONE
                # twist, fed back on the plate's pose error against the frame.
                c, s = math.cos(plate[1]), math.sin(plate[1])
                c_p = np.asarray(plate[0]) + np.array([[c, -s], [s, c]]) @ self.rigid[0]
                th_p = wrap(plate[1] - self.rigid[1])
                v_tw = (leader.pos - prev_c) / dt + _clip_norm(1.5 * (leader.pos - c_p), 0.5)
                w_tw = wrap(leader.heading - prev_h) / dt + \
                    max(-0.6, min(0.6, 2.0 * wrap(leader.heading - th_p)))
                cmds = []
                for i in range(K):
                    rr = pos[i] - c_p
                    cmds.append(velocity_cmd(yaw[i], v_tw + w_tw * np.array([-rr[1], rr[0]])))
                out["cmds"] = cmds
            out["comp_ms"] = self.st["comp_ms"]
            self.n_leg += 1
            if self.st["reached"]:
                self.wp += 1
                self.new_leg = True
                if self.wp >= len(self.waypoints):
                    self.mode, self.reached_all = "finishing", True
            elif self.n_leg >= P["timesteps"]:
                self.mode, self.reached_all = "finishing", False
            return out

        self.mode = "done"                    # commands stay zero
        return out
