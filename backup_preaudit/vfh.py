"""p-VFH local navigation, ported from pvfh_scan.m and pvfh_predict.m.

Same algorithm, same knobs, same fallbacks. pvfh_predict adds the dynamic
obstacle extension: movers are raycast at their contracted future poses over
the prediction horizon and each beam keeps the nearest blocking instance.
The static sensing path is byte-equivalent to pvfh_scan.m - do not add
self-motion shifts here (heading-coupled perception causes limit cycles,
found and removed during MATLAB-side development).
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import eval_mover_pose, lidar_scan


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _smooth_hist(x, w, circular=True):
    w = max(1, int(round(w)))
    if w == 1:
        return x
    n, half = len(x), w // 2
    k = np.ones(2 * half + 1) / (2 * half + 1)
    if circular:
        pad = np.concatenate([x[-half:], x, x[:half]]) if half > 0 else x
        full = np.convolve(pad, k, mode="same")
        return full[half:half + n]
    y = np.empty(n)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        y[i] = x[lo:hi].mean()
    return y


class VfhConfig:
    """Mirror of cfg.vfh (+ the dynamic + sensing knobs). Defaults = MATLAB."""
    def __init__(self, num_sectors=72, sigma=1.0, sense_range_base=4.0,
                 danger_dist_base=1.2, turn_limit=math.pi / 4, threshold=0.3,
                 smooth_window=5, avoid_gain=5.0, hysteresis_weight=0.6,
                 avoid_falloff_power=3.0, circular_smooth=True,
                 enlarge=True, auto_sigma=True, react_gap=1.0,
                 n_beams=288, pred_lead=0.5, pred_dt=0.1,
                 sense=None, p_unknown=0.05):
        self.sense = dict(sense) if sense else {
            "kind": "lidar", "sigma0": 0.02, "k_rng": 0.03,
            "fov_deg": 60.0, "max_range": float("inf")}
        self.p_unknown = p_unknown
        self.num_sectors = num_sectors
        self.sigma = sigma
        self.sense_range_base = sense_range_base
        self.danger_dist_base = danger_dist_base
        self.turn_limit = turn_limit
        self.threshold = threshold
        self.smooth_window = smooth_window
        self.avoid_gain = avoid_gain
        self.hysteresis_weight = hysteresis_weight
        self.avoid_falloff_power = avoid_falloff_power
        self.circular_smooth = circular_smooth
        self.enlarge = enlarge
        self.auto_sigma = auto_sigma
        self.react_gap = react_gap
        self.n_beams = n_beams
        self.pred_lead = pred_lead
        self.pred_dt = pred_dt


# ---------------------------------------------------------------------------
# shared sensing front end + noise model (scan_ranges_dyn.m + sensor_noise.m)
# ---------------------------------------------------------------------------

def _beam_fan(v: VfhConfig):
    """Beam-fan geometry: full circle for LiDAR, limited FoV arc for ToF."""
    kind = v.sense.get("kind", "lidar")
    if kind == "tof":
        fov = math.radians(float(v.sense.get("fov_deg", 60.0)))
        return -fov / 2.0, fov / 2.0
    return -math.pi, math.pi


def scan_ranges_dyn(pos, world, movers, t_now, nb, max_sense, v: VfhConfig):
    """Clean scan: statics + movers over the prediction horizon (all algos)."""
    amin, amax = _beam_fan(v)
    ranges, angles = lidar_scan(pos, world, nb, max_sense, amin, amax)
    if movers and v.pred_lead > 0:
        samples = sorted(set(list(np.arange(0, v.pred_lead + 1e-9, v.pred_dt))
                             + [v.pred_lead]))
        for dt in samples:
            mk = [m.eval_at(t_now + dt) for m in movers]
            rk = _movers_only_scan(pos, mk, nb, max_sense, angles)
            ranges = np.minimum(ranges, rk)
    return ranges, angles


def sensor_noise(ranges, v: VfhConfig, rng=None):
    """Apply the range-noise model (sensor_noise.m port)."""
    kind = v.sense.get("kind", "lidar")
    s0 = float(v.sense.get("sigma0", 0.02))
    kr = float(v.sense.get("k_rng", 0.03))
    max_r = v.sense.get("max_range", float("inf"))
    rng = np.random.default_rng() if rng is None else rng
    r = np.asarray(ranges, dtype=float)
    if kind == "tof":
        s = s0 + kr * np.maximum(r, 0.0)
    else:
        s = np.full_like(r, s0)
    r_noisy = r + rng.normal(0.0, 1.0, size=r.shape) * s
    drop = r_noisy >= max_r
    r_noisy = np.where(drop, max_r, r_noisy)
    return r_noisy, np.isfinite(r_noisy), drop


def _per_beam_sigma(ranges, v: VfhConfig):
    kind = v.sense.get("kind", "lidar")
    s0 = float(v.sense.get("sigma0", 0.02))
    kr = float(v.sense.get("k_rng", 0.03))
    if kind == "tof":
        return s0 + kr * np.maximum(ranges, 0.0)
    return np.full_like(np.asarray(ranges, dtype=float), s0)


def _erf(x):
    return np.array([math.erf(xi) for xi in np.asarray(x, dtype=float).ravel()]).reshape(np.shape(x))


def expected_kernel(d_meas, s_b, sigma_hist):
    """Posterior-expected deterministic danger kernel, eq. (6) of pvfh_prob.m:

        u_b = E[exp(-d_true^2 / (2 sigma_hist^2))],  d_true ~ N(d_meas, s_b^2)
            = sqrt(sh2/(sh2+s2)) * exp(-d_meas^2 / (2 (sh2+s2)))

    Exact deterministic limit (s_b -> 0 gives env(d_meas)), bounded, and
    monotonically inflated by sensor noise. Vectorised over d_meas/s_b.
    """
    d = np.asarray(d_meas, dtype=float)
    s2 = np.maximum(np.asarray(s_b, dtype=float), 1e-12) ** 2
    sh2 = float(sigma_hist) ** 2
    return np.sqrt(sh2 / (sh2 + s2)) * np.exp(-(d * d) / (2.0 * (sh2 + s2)))


def apf_obstacle_repulsion(ranges, angles, danger_dist, falloff_power, eps):
    """APF obstacle reflex on a scan (apf_obstacle_repulsion.m port)."""
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    close = ranges < danger_dist
    if not close.any():
        return np.zeros(2)
    r, aB = ranges[close], angles[close]
    w = 1.0 / (r ** falloff_power + eps)
    dth = 2 * math.pi / len(ranges)
    return np.array([-np.sum(w * np.cos(aB)) * dth,
                     -np.sum(w * np.sin(aB)) * dth])


# ---------------------------------------------------------------------------
# P-VFH: probabilistic occupancy + APF reflex (pvfh_prob.m port)
# ---------------------------------------------------------------------------

def pvfh_prob(pos, world, movers, t_now, goal, margin, v: VfhConfig, eps,
              prev_heading, rng=None):
    """Probabilistic VFH steering (pvfh_prob.m port -- see that file for the
    full derivation). Stage 1: two per-beam layers computed from the SAME
    noisy scan -- a blockage-probability risk layer p_b = Phi((d_block -
    d_meas)/s_b) and the posterior-expected deterministic kernel u_b =
    E[exp(-d_true^2/(2 sigma_hist^2))], d_true ~ N(d_meas, s_b^2); the
    selection histogram is built from u (max-pooled, VFH+ propagated),
    threshold/valley/hysteresis as in the deterministic stack. Stage 2: APF
    obstacle reflex on the same noisy scan."""
    S = v.num_sectors
    max_sense = v.sense_range_base + margin
    mr = v.sense.get("max_range", float("inf"))
    if math.isfinite(mr):
        max_sense = min(max_sense, mr)
    danger_dist = v.danger_dist_base + margin
    nb = v.n_beams if v.n_beams else 4 * S
    r_geom = margin if v.enlarge else 0.0
    d_block = r_geom + float(v.react_gap)   # reaction envelope, matches auto_sigma
    p_unk = float(v.p_unknown)

    # the deterministic kernel width from the auto_sigma contract
    sigma_hist = v.sigma
    if v.auto_sigma:
        sigma_hist = (r_geom + float(v.react_gap)) / math.sqrt(
            2 * math.log(1.0 / v.threshold))

    # ---- 1. sense + noise (once; rng=None -> unseeded, like MATLAB's global
    # stream when the caller has not seeded it) ----
    clean_ranges, angles = scan_ranges_dyn(pos, world, movers, t_now, nb,
                                           max_sense, v)
    ranges, valid, drop = sensor_noise(clean_ranges, v, rng)
    sig_b = _per_beam_sigma(ranges, v)

    # ---- 2. per-beam layers ----
    # Risk layer: p_b = Phi((d_block - d_meas)/s_b).
    # Selection layer: u_b = E[env(d_true)] =
    #   sqrt(sh2/(sh2+s2)) * exp(-d_meas^2 / (2 (sh2+s2))),  sh = sigma_hist.
    d_meas = np.where(valid, ranges, max_sense)
    z = (d_block - d_meas) / np.maximum(sig_b, 1e-6)
    p_beam = 0.5 * (1.0 + _erf(z / math.sqrt(2.0)))
    p_beam = np.where(valid, p_beam, 0.0)
    p_beam = np.where(drop, np.maximum(p_beam, p_unk), p_beam)

    sh2 = sigma_hist * sigma_hist
    u_beam = expected_kernel(d_meas, sig_b, sigma_hist)
    u_beam = np.where(valid, u_beam, p_unk)          # no return -> floor
    u_beam = np.where(drop, np.maximum(u_beam, p_unk), u_beam)

    # ---- 3. sector pooling: noisy-OR risk, MAX kernel ----
    sector_angles = -math.pi + (2 * math.pi) * np.arange(S) / S
    dsec = 2 * math.pi / S
    sec_idx = np.clip(((angles + math.pi) / (2 * math.pi) * S).astype(int),
                      0, S - 1)
    p_sec = np.zeros(S)
    u_sec = np.zeros(S)
    seen = np.zeros(S, dtype=bool)
    for si, pb, ub in zip(sec_idx, p_beam, u_beam):
        if pb > 0:
            p_sec[si] = 1.0 - (1.0 - p_sec[si]) * (1.0 - pb)
            seen[si] = True
        u_sec[si] = max(u_sec[si], ub)
    p_sec = np.where(~seen, p_unk, p_sec)
    u_sec = np.where(~seen, p_unk, u_sec)

    # ---- 4. VFH+ enlarged-obstacle propagation (kernel values) ----
    hist = u_sec.copy()
    if r_geom > 0:
        for si in range(S):
            if u_sec[si] <= p_unk:
                continue
            sel = (sec_idx == si) & valid
            if not sel.any():
                continue
            d = float(ranges[sel].min())
            gamma = math.asin(min(1.0, r_geom / max(d, r_geom)))
            kb = int(math.ceil(gamma / dsec))
            for off in range(-kb, kb + 1):
                idx = (si + off) % S
                hist[idx] = max(hist[idx], u_sec[si])

    hist = _smooth_hist(hist, v.smooth_window, v.circular_smooth)

    # ---- 5. threshold / valley / hysteresis ----
    candidates = np.where(hist < v.threshold)[0]
    if len(candidates) == 0:
        candidates = np.array([int(np.argmin(hist))])
    goal_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
    dev = np.abs([_wrap_pi(sector_angles[i] - goal_heading) for i in candidates])
    feasible = candidates[dev <= v.turn_limit]
    if len(feasible) == 0:
        feasible = candidates
    sh = sector_angles[feasible]
    costs = np.abs([_wrap_pi(x - goal_heading) for x in sh]) + \
        v.hysteresis_weight * np.abs([_wrap_pi(x - prev_heading) for x in sh])
    chosen = sh[int(np.argmin(costs))]
    vec = np.array([math.cos(chosen), math.sin(chosen)])

    # ---- 6. APF obstacle reflex on the same noisy scan ----
    vec = vec + v.avoid_gain * apf_obstacle_repulsion(
        ranges, angles, danger_dist, v.avoid_falloff_power, eps)

    nv = float(np.linalg.norm(vec))
    if nv < eps:
        vec = np.array([math.cos(prev_heading), math.sin(prev_heading)])
    else:
        vec = vec / nv
    return vec


def pvfh_core(ranges, angles, pos, goal, margin, v: VfhConfig, eps, prev_heading):
    """Histogram build + valley search + repulsion. Shared by both entries."""
    S = v.num_sectors
    max_sense = v.sense_range_base + margin
    danger_dist = v.danger_dist_base + margin

    # ---- polar obstacle density, nearest surface per sector ----
    sector_angles = -math.pi + (2 * math.pi) * np.arange(S) / S   # open interval
    dsec = 2 * math.pi / S
    sec_idx = np.clip(((angles + math.pi) / (2 * math.pi) * S).astype(int), 0, S - 1)

    occ = ranges < max_sense
    rmin = np.full(S, np.inf)
    for si, r in zip(sec_idx[occ], ranges[occ]):
        rmin[si] = min(rmin[si], r)

    r_eff = margin if v.enlarge else 0.0
    sigma = v.sigma
    if v.auto_sigma:
        d_react = r_eff + v.react_gap
        sigma = d_react / math.sqrt(2 * math.log(1 / v.threshold))

    hist = np.zeros(S)
    for si in np.where(rmin < max_sense)[0]:
        d = rmin[si]
        hist[si] = max(hist[si], math.exp(-(d * d) / (2 * sigma * sigma)))
        if r_eff > 0:
            gamma = math.asin(min(1.0, r_eff / max(d, r_eff)))
            kb = int(math.ceil(gamma / dsec))
            for off in range(-kb, kb + 1):
                idx = (si + off) % S
                hist[idx] = max(hist[idx], math.exp(-(d * d) / (2 * sigma * sigma)))

    hist = _smooth_hist(hist, v.smooth_window, v.circular_smooth)

    # ---- candidate valleys, goal-bearing restriction, hysteresis ----
    candidates = np.where(hist < v.threshold)[0]
    if len(candidates) == 0:
        candidates = np.array([int(np.argmin(hist))])

    goal_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
    dev = np.abs([_wrap_pi(sector_angles[i] - goal_heading) for i in candidates])
    feasible = candidates[dev <= v.turn_limit]
    if len(feasible) == 0:
        feasible = candidates

    sh = sector_angles[feasible]
    costs = np.abs([_wrap_pi(x - goal_heading) for x in sh]) + \
        v.hysteresis_weight * np.abs([_wrap_pi(x - prev_heading) for x in sh])
    chosen = sh[int(np.argmin(costs))]

    vec = np.array([math.cos(chosen), math.sin(chosen)])

    # ---- close-range repulsion off the raw scan ----
    close = ranges < danger_dist
    if close.any():
        r, aB = ranges[close], angles[close]
        wgt = 1.0 / (r ** v.avoid_falloff_power + eps)
        dth = 2 * math.pi / len(ranges)
        vx = -np.sum(wgt * np.cos(aB)) * dth
        vy = -np.sum(wgt * np.sin(aB)) * dth
        vec = vec + v.avoid_gain * np.array([vx, vy])

    nv = float(np.linalg.norm(vec))
    if nv < eps:
        vec = np.array([math.cos(prev_heading), math.sin(prev_heading)])
    else:
        vec = vec / nv
    return vec, hist, sigma


def pvfh_scan(pos, world, goal, margin, v: VfhConfig, eps, prev_heading):
    """pvfh_scan.m port: instantaneous scan of the (static) world."""
    max_sense = v.sense_range_base + margin
    nb = v.n_beams if v.n_beams else 4 * v.num_sectors
    amin, amax = _beam_fan(v)
    ranges, angles = lidar_scan(pos, world, nb, max_sense, amin, amax)
    vec, hist, sigma = pvfh_core(ranges, angles, pos, goal, margin, v, eps,
                                 prev_heading)
    return vec


def pvfh_predict(pos, world, movers, v_self, t_now, goal, margin,
                 v: VfhConfig, eps, prev_heading):
    """pvfh_predict.m port: static scan + movers at their future poses.

    Beams are raycast against the static world exactly as pvfh_scan does,
    then against each mover at its contracted poses t_now .. t_now+pred_lead
    (sampled at pred_dt); each beam keeps the nearest blocking instance.
    v_self is accepted for interface parity and logged, not used to shift the
    sensor position.
    """
    max_sense = v.sense_range_base + margin
    nb = v.n_beams if v.n_beams else 4 * v.num_sectors
    amin, amax = _beam_fan(v)
    ranges, angles = lidar_scan(pos, world, nb, max_sense, amin, amax)

    if movers and v.pred_lead > 0:
        samples = sorted(set(list(np.arange(0, v.pred_lead + 1e-9, v.pred_dt)) +
                             [v.pred_lead]))
        for dt in samples:
            mk = [m.eval_at(t_now + dt) for m in movers]
            rk = _movers_only_scan(pos, mk, nb, max_sense, angles)
            ranges = np.minimum(ranges, rk)

    vec, hist, sigma = pvfh_core(ranges, angles, pos, goal, margin, v, eps,
                                 prev_heading)
    return vec


def _movers_only_scan(pos, movers, n_beams, max_range, angles):
    """Raycast against movers only (no statics), for min-combining."""
    ranges = np.full(n_beams, float(max_range))
    cx, sy = np.cos(angles), np.sin(angles)
    EPS = 1e-12
    for m in movers:
        ox, oy = pos[0] - m.c[0], pos[1] - m.c[1]
        if m.shape == "circle":
            bq = ox * cx + oy * sy
            cq = ox * ox + oy * oy - m.r ** 2
            disc = bq * bq - cq
            ok = disc >= 0
            sq = np.sqrt(np.maximum(disc, 0))
            t = -bq - sq
            tfar = -bq + sq
            t = np.where(t < 0, tfar, t)
            hit = ok & (t > 0) & (t < ranges)
            ranges = np.where(hit, t, ranges)
        else:
            hx, hy = m.size[0] / 2, m.size[1] / 2
            ldx = np.where(np.abs(cx) < EPS, EPS, cx)
            ldy = np.where(np.abs(sy) < EPS, EPS, sy)
            t1x = (-hx - ox) / ldx
            t2x = (hx - ox) / ldx
            t1y = (-hy - oy) / ldy
            t2y = (hy - oy) / ldy
            tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
            tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
            t = np.where(tmin < 0, tmax, tmin)
            hit = (tmax >= np.maximum(tmin, 0)) & (t > 0) & (t < ranges)
            ranges = np.where(hit, t, ranges)
    return ranges


# ---------------------------------------------------------------------------
# algorithm dispatch (steering_rigid.m / steering_point.m ports)
# ---------------------------------------------------------------------------

def steering_rigid(pos, goal, w_att, world, movers, t_now, margin, cfg: VfhConfig,
                   algo, eps, prev_heading, v_self, rng=None):
    """Transport-phase dispatch: returns (vec, comp_ms) for algo in
    {'apf', 'vfh', 'pvfh', 'pvfh_prob'}. Same semantics as the MATLAB
    steering_rigid(). w_att is informational for 'apf' (the caller's
    vec_goal already carries the attraction)."""
    import time
    t0 = time.perf_counter()
    algo = algo.lower()
    if algo == "apf":
        # obstacle reflex only; the caller's vec_goal carries the attraction
        vec = _apf_reflex(pos, world, movers, t_now, margin, cfg, eps, rng)
    elif algo == "vfh":
        ranges, angles = _vfh_baseline_scan(pos, world, movers, t_now, margin,
                                            cfg, rng)
        vec = _vfh_from_scan(ranges, angles, pos, goal, margin, cfg, eps,
                             prev_heading)
    elif algo == "pvfh":
        vec = pvfh_predict(pos, world, movers, v_self, t_now, goal, margin,
                           cfg, eps, prev_heading)
    elif algo == "pvfh_prob":
        vec = pvfh_prob(pos, world, movers, t_now, goal, margin, cfg, eps,
                        prev_heading, rng)
    else:
        raise ValueError(f"unknown algo {algo!r}")
    return vec, 1000.0 * (time.perf_counter() - t0)


def steering_point(pos, goal, w_att, world, movers, t_now, robot_radius,
                   cfg: VfhConfig, algo, eps, prev_heading, rng=None):
    """Forming-phase dispatch (single robot, point body). The caller passes
    the APF virtual goal as `goal`. For the pure-APF baseline the attraction
    toward that virtual goal is added HERE (cascaded architecture) -- unlike
    the transport phase, where vec_goal is blended outside the dispatcher."""
    import time
    t0 = time.perf_counter()
    algo = algo.lower()
    if algo == "apf":
        g = np.asarray(goal, dtype=float)
        p = np.asarray(pos, dtype=float)
        vec = w_att * (g - p) / (float(np.linalg.norm(g - p)) + eps)
        vec = vec + cfg.avoid_gain * _apf_reflex(
            pos, world, movers, t_now, robot_radius, cfg, eps, rng)
        nv = float(np.linalg.norm(vec))
        if nv > eps:
            vec = vec / nv
    else:
        vec, _ = steering_rigid(pos, goal, w_att, world, movers, t_now,
                                robot_radius, cfg, algo, eps, prev_heading,
                                None, rng)
    return vec, 1000.0 * (time.perf_counter() - t0)


def _apf_reflex(pos, world, movers, t_now, margin, cfg: VfhConfig, eps, rng):
    """Normalised APF obstacle reflex for the pure-APF baseline."""
    max_sense = cfg.sense_range_base + margin
    mr = cfg.sense.get("max_range", float("inf"))
    if math.isfinite(mr):
        max_sense = min(max_sense, mr)
    danger = cfg.danger_dist_base + margin
    nb = cfg.n_beams if cfg.n_beams else 4 * cfg.num_sectors
    amin, amax = _beam_fan(cfg)
    clean, angles = lidar_scan(pos, world, nb, max_sense, amin, amax)
    if movers and cfg.pred_lead > 0:
        samples = sorted(set(list(np.arange(0, cfg.pred_lead + 1e-9, cfg.pred_dt))
                             + [cfg.pred_lead]))
        for dt in samples:
            mk = [m.eval_at(t_now + dt) for m in movers]
            rk = _movers_only_scan(pos, mk, nb, max_sense, angles)
            clean = np.minimum(clean, rk)
    ranges, _, _ = sensor_noise(clean, cfg, rng)
    a = apf_obstacle_repulsion(ranges, angles, danger,
                               cfg.avoid_falloff_power, eps)
    nv = float(np.linalg.norm(a))
    return a / nv if nv > eps else a


def _vfh_baseline_scan(pos, world, movers, t_now, margin, cfg: VfhConfig, rng):
    """One noisy scan for the deterministic-VFH baseline (no mover horizon:
    the classic single-robot stack has no dynamic-obstacle prediction)."""
    max_sense = cfg.sense_range_base + margin
    mr = cfg.sense.get("max_range", float("inf"))
    if math.isfinite(mr):
        max_sense = min(max_sense, mr)
    nb = cfg.n_beams if cfg.n_beams else 4 * cfg.num_sectors
    amin, amax = _beam_fan(cfg)
    clean, angles = lidar_scan(pos, world, nb, max_sense, amin, amax)
    if movers:
        clean2 = clean.copy()
        mk = [m.eval_at(t_now) for m in movers]
        rk = _movers_only_scan(pos, mk, nb, max_sense, angles)
        clean = np.minimum(clean2, rk)
    ranges, _, _ = sensor_noise(clean, cfg, rng)
    return ranges, angles


def _vfh_from_scan(ranges, angles, pos, goal, margin, cfg: VfhConfig, eps,
                   prev_heading):
    """Deterministic VFH valley search on a provided scan (vfh_from_scan.m)."""
    S = cfg.num_sectors
    max_sense = cfg.sense_range_base + margin
    danger_dist = cfg.danger_dist_base + margin
    sector_angles = -math.pi + (2 * math.pi) * np.arange(S) / S
    dsec = 2 * math.pi / S
    sec_idx = np.clip(((angles + math.pi) / (2 * math.pi) * S).astype(int),
                      0, S - 1)
    rmin = np.full(S, np.inf)
    for si, r in zip(sec_idx, ranges):
        rmin[si] = min(rmin[si], r)

    r_eff = margin if cfg.enlarge else 0.0
    sigma = cfg.sigma
    if cfg.auto_sigma:
        sigma = (r_eff + cfg.react_gap) / math.sqrt(2 * math.log(1 / cfg.threshold))

    hist = np.zeros(S)
    for si in np.where(rmin < max_sense)[0]:
        d = rmin[si]
        val = math.exp(-(d * d) / (2 * sigma * sigma))
        if r_eff > 0:
            gamma = math.asin(min(1.0, r_eff / max(d, r_eff)))
            kb = int(math.ceil(gamma / dsec))
            for off in range(-kb, kb + 1):
                idx = (si + off) % S
                hist[idx] = max(hist[idx], val)
        else:
            hist[si] = max(hist[si], val)

    hist = _smooth_hist(hist, cfg.smooth_window, cfg.circular_smooth)

    candidates = np.where(hist < cfg.threshold)[0]
    if len(candidates) == 0:
        candidates = np.array([int(np.argmin(hist))])
    goal_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
    dev = np.abs([_wrap_pi(sector_angles[i] - goal_heading) for i in candidates])
    feasible = candidates[dev <= cfg.turn_limit]
    if len(feasible) == 0:
        feasible = candidates
    sh = sector_angles[feasible]
    costs = np.abs([_wrap_pi(x - goal_heading) for x in sh]) + \
        cfg.hysteresis_weight * np.abs([_wrap_pi(x - prev_heading) for x in sh])
    chosen = sh[int(np.argmin(costs))]
    vec = np.array([math.cos(chosen), math.sin(chosen)])
    vec = vec + cfg.avoid_gain * apf_obstacle_repulsion(
        ranges, angles, danger_dist, cfg.avoid_falloff_power, eps)
    nv = float(np.linalg.norm(vec))
    if nv < eps:
        vec = np.array([math.cos(prev_heading), math.sin(prev_heading)])
    else:
        vec = vec / nv
    return vec
