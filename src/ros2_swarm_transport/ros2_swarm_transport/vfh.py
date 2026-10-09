"""Steering laws and the shared sensing front end.

Port of the corrected (October 2026 audit) MATLAB reference:

    scan_ranges_dyn.m         -> scan_ranges_dyn   (body-frame ToF fan, dth)
    sensor_noise.m            -> sensor_noise
    apf_obstacle_repulsion.m  -> apf_obstacle_repulsion (beam width aware)
    range_filter.m            -> RangeFilter / range_filter
    pvfh_prob.m               -> pvfh_prob          (P-VFH, the proposal)
    steering_dispatch.m       -> steering_rigid / steering_point and the laws
                                 apf, vfh, fgm, dwa, mpc, pvfh, pvfh_filt

Every law consumes a scan from ONE front end (`_sense`): same beam fan, same
noise model, same seeded stream. The laws differ only in the steering rule and
in whether the scan carries the mover prediction horizon:

    algo         steering law                          mover horizon
    apf          APF obstacle reflex (no histogram)    none (instantaneous)
    vfh          deterministic VFH+ histogram          none (instantaneous)
    fgm          Follow-the-Gap                        T_lead (baseline_horizon)
    dwa          Dynamic-Window arc sampling           T_lead (baseline_horizon)
    mpc          receding-horizon lattice MPC          T_lead (baseline_horizon)
    pvfh         deterministic VFH+ histogram          T_lead
    pvfh_filt    pvfh + recursive range filter         T_lead   (ablation)
    pvfh_prob    range filter + expected kernel        T_lead   (proposal)

pvfh_scan / pvfh_predict (noise-free, static path of Simulation7) are kept for
the static tests; the comparison no longer uses them.
"""

from __future__ import annotations

import math
import time

import numpy as np

from .geometry import lidar_scan

ALGOS = ("apf", "vfh", "fgm", "dwa", "mpc", "pvfh", "pvfh_filt", "pvfh_prob")


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
                 sense=None, p_unknown=0.05, baseline_horizon=True,
                 rfilter=True, rfilter_q=0.08, fgm_alpha=2.0,
                 dwa=None, mpc=None):
        self.sense = {"kind": "lidar", "sigma0": 0.02, "k_rng": 0.03,
                      "fov_deg": 60.0, "max_range": float("inf"),
                      "frame": "body"}
        if sense:
            self.sense.update(sense)
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
        self.baseline_horizon = baseline_horizon
        self.rfilter = rfilter
        self.rfilter_q = rfilter_q
        self.fgm_alpha = fgm_alpha
        self.dwa = dict(dwa) if dwa else {}
        self.mpc = dict(mpc) if mpc else {}
        # per-mission filter state (range_filter.m keeps it in a persistent);
        # a new VfhConfig, or reset_filters(), starts a new mission.
        self.filters = {}

    def reset_filters(self):
        self.filters = {}

    def sigma_hist(self, margin):
        """Kernel width from the auto_sigma contract: env(d_block) = threshold."""
        if not self.auto_sigma:
            return self.sigma
        r_eff = margin if self.enlarge else 0.0
        return (r_eff + self.react_gap) / math.sqrt(
            2 * math.log(1.0 / self.threshold))


# ---------------------------------------------------------------------------
# shared sensing front end + noise model (scan_ranges_dyn.m + sensor_noise.m)
# ---------------------------------------------------------------------------

def _beam_fan(v: VfhConfig, boresight=0.0):
    """Beam-fan limits: full circle for LiDAR; for ToF the FoV arc centred on
    the boresight (the body heading when sense.frame == 'body')."""
    if v.sense.get("kind", "lidar") == "tof":
        fov = math.radians(float(v.sense.get("fov_deg", 60.0)))
        bore = boresight if v.sense.get("frame", "body") == "body" else 0.0
        bore = 0.0 if bore is None else float(bore)
        return bore - fov / 2.0, bore + fov / 2.0
    return -math.pi, math.pi


def _horizon_samples(pred_lead, pred_dt):
    s = {0.0, max(float(pred_lead), 0.0)}
    if pred_lead > 0:
        s.update(float(x) for x in np.arange(0.0, pred_lead + 1e-9, pred_dt))
    return sorted(s)


def scan_ranges_dyn(pos, world, movers, t_now, nb, max_sense, v: VfhConfig,
                    boresight=0.0, pred_lead=None):
    """Clean scan: statics + movers at t_now and over the prediction horizon.

    Returns (ranges, angles, dth). angles are absolute bearings wrapped to
    [-pi, pi); dth is the angular width one beam represents (2*pi/nb for the
    lidar, fov/nb for the ToF fan). pred_lead = 0 is the INSTANTANEOUS scan:
    movers are still seen, where they are now."""
    if pred_lead is None:
        pred_lead = v.pred_lead
    amin, amax = _beam_fan(v, boresight)
    dth = (amax - amin) / nb
    ranges, angles = lidar_scan(pos, world, nb, max_sense, amin, amax)
    if movers:
        for dt in _horizon_samples(pred_lead, v.pred_dt):
            mk = [m.eval_at(t_now + dt) for m in movers]
            ranges = np.minimum(ranges, _movers_only_scan(
                pos, mk, nb, max_sense, angles))
    return ranges, _wrap_pi(angles), dth


def sensor_noise(ranges, v: VfhConfig, rng=None):
    """Apply the range-noise model (sensor_noise.m port).
    lidar: s = sigma0;  tof: s = sigma0 + k_rng * d (linear)."""
    max_r = v.sense.get("max_range", float("inf"))
    rng = np.random.default_rng() if rng is None else rng
    r = np.asarray(ranges, dtype=float)
    s = _per_beam_sigma(r, v)
    r_noisy = r + rng.normal(0.0, 1.0, size=r.shape) * s
    drop = r_noisy >= max_r
    r_noisy = np.where(drop, max_r, r_noisy)
    return r_noisy, np.isfinite(r_noisy), drop


def _per_beam_sigma(ranges, v: VfhConfig):
    s0 = float(v.sense.get("sigma0", 0.02))
    kr = float(v.sense.get("k_rng", 0.03))
    r = np.asarray(ranges, dtype=float)
    if v.sense.get("kind", "lidar") == "tof":
        return s0 + kr * np.maximum(r, 0.0)
    return np.full_like(r, s0)


def _erf(x):
    x = np.asarray(x, dtype=float)
    return np.vectorize(math.erf, otypes=[float])(x)


def expected_kernel(d_meas, s_b, sigma_hist):
    """Posterior-expected deterministic danger kernel, eq. (6) of pvfh_prob.m:

        u_b = E[exp(-d_true^2 / (2 sigma_hist^2))],  d_true ~ N(d_meas, s_b^2)
            = sqrt(sh2/(sh2+s2)) * exp(-d_meas^2 / (2 (sh2+s2)))

    Element-wise over d_meas / s_b."""
    d = np.asarray(d_meas, dtype=float)
    s2 = np.maximum(np.asarray(s_b, dtype=float), 1e-12) ** 2
    sh2 = float(sigma_hist) ** 2
    return np.sqrt(sh2 / (sh2 + s2)) * np.exp(-(d * d) / (2.0 * (sh2 + s2)))


def kernel_reaction_distance(s_b, sigma_hist, threshold=0.3):
    """Range at which a return crosses the threshold, u_b(d) = threshold.
    Grows with s_b up to s* = 1.757 sigma_hist (Proposition 2)."""
    sh2 = float(sigma_hist) ** 2
    tot = sh2 + float(s_b) ** 2
    arg = 2.0 * tot * math.log(math.sqrt(sh2 / tot) / threshold)
    return math.sqrt(arg) if arg > 0 else 0.0


def apf_obstacle_repulsion(ranges, angles, danger_dist, falloff_power, eps,
                           dth=None):
    """APF obstacle reflex on a scan (apf_obstacle_repulsion.m port).
    dth is the beam angular width; default 2*pi/n is right only for a
    full-circle fan."""
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    if ranges.size == 0:
        return np.zeros(2)
    close = (ranges < danger_dist) & np.isfinite(ranges)
    if not close.any():
        return np.zeros(2)
    r, aB = ranges[close], angles[close]
    w = 1.0 / (r ** falloff_power + eps)
    if dth is None:
        dth = 2 * math.pi / len(ranges)
    return np.array([-np.sum(w * np.cos(aB)) * dth,
                     -np.sum(w * np.sin(aB)) * dth])


def range_filter(z, s, t_now, q_std, state, tag="default"):
    """Per-beam recursive Bayesian (Kalman) range filter (range_filter.m).

    x_t = x_{t-1} + w, w ~ N(0, q^2);  z_t = x_t + e, e ~ N(0, s^2).
    Gain K = (P+q^2)/(P+q^2+s^2); an innovation beyond 3 sigma re-initialises
    that beam; a time gap > 0.5 s, a backward step or a change of N
    re-initialises the filter. `state` is a dict owned by the caller.
    Returns (posterior mean, posterior std)."""
    z = np.asarray(z, dtype=float).ravel()
    r = np.maximum(np.asarray(s, dtype=float).ravel(), 1e-9) ** 2
    st = state.get(tag)
    fresh = (st is None or len(st["x"]) != len(z)
             or t_now < st["t"] - 1e-9 or t_now > st["t"] + 0.5)
    if fresh:
        x, P = z.copy(), r.copy()
    else:
        x = st["x"]
        Pp = st["P"] + q_std ** 2
        innov = z - x
        Sg = Pp + r
        K = Pp / Sg
        jump = (np.abs(innov) > 3 * np.sqrt(Sg)) | ~np.isfinite(x)
        x = x + K * innov
        P = (1 - K) * Pp
        x = np.where(jump, z, x)
        P = np.where(jump, r, P)
    state[tag] = {"x": x, "P": P, "t": t_now}
    return x.copy(), np.sqrt(P)


def _sense(pos, world, movers, t_now, cfg: VfhConfig, margin, with_horizon,
           heading, rng):
    """local_sense: the one sensing path of every dispatch-level law."""
    max_sense = cfg.sense_range_base + margin
    mr = cfg.sense.get("max_range", float("inf"))
    if math.isfinite(mr):
        max_sense = min(max_sense, mr)
    nb = cfg.n_beams if cfg.n_beams else 4 * cfg.num_sectors
    clean, angles, dth = scan_ranges_dyn(
        pos, world, movers, t_now, nb, max_sense, cfg, boresight=heading,
        pred_lead=cfg.pred_lead if with_horizon else 0.0)
    ranges, valid, drop = sensor_noise(clean, cfg, rng)
    return {"ranges": ranges, "angles": angles, "dth": dth, "valid": valid,
            "drop": drop, "max_sense": max_sense,
            "danger": cfg.danger_dist_base + margin}


# ---------------------------------------------------------------------------
# histogram machinery shared by vfh / pvfh / pvfh_filt / pvfh_prob
# ---------------------------------------------------------------------------

def _sector_index(angles, S):
    return np.clip(np.floor((angles + math.pi) / (2 * math.pi) * S).astype(int),
                   0, S - 1)


def _select_valley(hist, pos, goal, v: VfhConfig, prev_heading):
    S = v.num_sectors
    sector_angles = -math.pi + (2 * math.pi) * np.arange(S) / S
    candidates = np.where(hist < v.threshold)[0]
    if len(candidates) == 0:
        candidates = np.array([int(np.argmin(hist))])
    goal_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
    dev = np.abs(_wrap_pi(sector_angles[candidates] - goal_heading))
    feasible = candidates[dev <= v.turn_limit]
    if len(feasible) == 0:
        feasible = candidates
    sh = sector_angles[feasible]
    costs = np.abs(_wrap_pi(sh - goal_heading)) + \
        v.hysteresis_weight * np.abs(_wrap_pi(sh - prev_heading))
    return float(sh[int(np.argmin(costs))])


def _finish(chosen, ranges, angles, danger, dth, v: VfhConfig, eps,
            prev_heading):
    vec = np.array([math.cos(chosen), math.sin(chosen)])
    vec = vec + v.avoid_gain * apf_obstacle_repulsion(
        ranges, angles, danger, v.avoid_falloff_power, eps, dth)
    nv = float(np.linalg.norm(vec))
    if nv < eps:
        return np.array([math.cos(prev_heading), math.sin(prev_heading)])
    return vec / nv


def vfh_histogram(ranges, angles, margin, v: VfhConfig, max_sense=None):
    """Deterministic VFH+ histogram (nearest return per sector, enlarged
    obstacles, circular smoothing). Unobserved sectors read free."""
    S = v.num_sectors
    if max_sense is None:
        max_sense = v.sense_range_base + margin
    dsec = 2 * math.pi / S
    sec_idx = _sector_index(np.asarray(angles), S)
    rmin = np.full(S, np.inf)
    np.minimum.at(rmin, sec_idx, np.asarray(ranges, dtype=float))
    r_eff = margin if v.enlarge else 0.0
    sigma = v.sigma_hist(margin)
    hist = np.zeros(S)
    for si in np.where(rmin < max_sense)[0]:
        d = max(float(rmin[si]), 0.0)
        val = math.exp(-(d * d) / (2 * sigma * sigma))
        if r_eff > 0:
            gamma = math.asin(min(1.0, r_eff / max(d, r_eff)))
            kb = int(math.ceil(gamma / dsec))
            idx = np.arange(si - kb, si + kb + 1) % S
            hist[idx] = np.maximum(hist[idx], val)
        else:
            hist[si] = max(hist[si], val)
    return _smooth_hist(hist, v.smooth_window, v.circular_smooth)


def _vfh_from_scan(ranges, angles, pos, goal, margin, cfg: VfhConfig, eps,
                   prev_heading, dth=None, max_sense=None, danger=None):
    """Deterministic VFH+ valley search on a provided scan
    (local_vfh_from_scan). Bearings come from the scan, never rebuilt."""
    if danger is None:
        danger = cfg.danger_dist_base + margin
    hist = vfh_histogram(ranges, angles, margin, cfg, max_sense)
    chosen = _select_valley(hist, pos, goal, cfg, prev_heading)
    return _finish(chosen, ranges, angles, danger, dth, cfg, eps, prev_heading)


# ---------------------------------------------------------------------------
# P-VFH (pvfh_prob.m port)
# ---------------------------------------------------------------------------

def pvfh_prob(pos, world, movers, t_now, goal, margin, v: VfhConfig, eps,
              prev_heading, rng=None, use_filter=False, return_info=False):
    """Probabilistic VFH steering. One noisy scan; optional recursive range
    filter (transport phase, lidar fan); posterior-expected kernel
    u_b = E[exp(-d^2/(2 sigma_hist^2))], d ~ N(d_meas, s_b^2), max-pooled per
    sector and VFH+ propagated; threshold / valley / hysteresis as in the
    deterministic stack; APF reflex on the same ranges. The noisy-OR risk
    layer p is a diagnostic only."""
    S = v.num_sectors
    sc = _sense(pos, world, movers, t_now, v, margin, True, prev_heading, rng)
    ranges, angles, valid, drop = (sc["ranges"], sc["angles"], sc["valid"],
                                   sc["drop"])
    max_sense = sc["max_sense"]
    r_geom = margin if v.enlarge else 0.0
    d_block = r_geom + float(v.react_gap)
    p_unk = float(v.p_unknown)
    sigma_hist = v.sigma_hist(margin)
    sig_b = _per_beam_sigma(ranges, v)

    if use_filter and bool(np.all(valid)):
        ranges, sig_b = range_filter(ranges, sig_b, t_now, v.rfilter_q,
                                     v.filters, "pvfh_prob")

    d_meas = np.where(valid, ranges, max_sense)
    u_beam = expected_kernel(d_meas, sig_b, sigma_hist)
    u_beam = np.where(valid, u_beam, p_unk)
    u_beam = np.where(drop, np.maximum(u_beam, p_unk), u_beam)

    z = (d_block - d_meas) / np.maximum(sig_b, 1e-6)
    p_beam = 0.5 * (1.0 + _erf(z / math.sqrt(2.0)))
    p_beam = np.where(valid, p_beam, 0.0)
    p_beam = np.where(drop, np.maximum(p_beam, p_unk), p_beam)

    dsec = 2 * math.pi / S
    sec_idx = _sector_index(angles, S)
    u_sec = np.zeros(S)
    np.maximum.at(u_sec, sec_idx, u_beam)
    n_in = np.bincount(sec_idx[p_beam > 0], minlength=S)
    u_sec = np.where((u_sec == 0) & (n_in == 0), p_unk, u_sec)

    hist = u_sec.copy()
    if r_geom > 0:
        dmin = np.full(S, np.inf)
        np.minimum.at(dmin, sec_idx[valid], ranges[valid])
        for si in np.where(u_sec > p_unk)[0]:
            d = float(dmin[si])
            if not math.isfinite(d):
                continue
            gamma = math.asin(min(1.0, r_geom / max(d, r_geom)))
            kb = int(math.ceil(gamma / dsec))
            idx = np.arange(si - kb, si + kb + 1) % S
            hist[idx] = np.maximum(hist[idx], u_sec[si])
    hist = _smooth_hist(hist, v.smooth_window, v.circular_smooth)

    chosen = _select_valley(hist, pos, goal, v, prev_heading)
    vec = _finish(chosen, ranges, angles, sc["danger"], sc["dth"], v, eps,
                  prev_heading)
    if return_info:
        q = np.ones(S)
        np.multiply.at(q, sec_idx, 1.0 - p_beam)
        p_sec = np.where(n_in == 0, p_unk, 1.0 - q)
        return vec, {"p_beam": p_beam, "p_sector": p_sec, "u_beam": u_beam,
                     "u_sector": u_sec, "hist": hist, "ranges": ranges,
                     "angles": angles, "sigma_hist": sigma_hist,
                     "sigma_b": sig_b, "chosen_heading": chosen}
    return vec


# ---------------------------------------------------------------------------
# static, noise-free path of Simulation7 (kept for the static tests)
# ---------------------------------------------------------------------------

def pvfh_core(ranges, angles, pos, goal, margin, v: VfhConfig, eps,
              prev_heading):
    hist = vfh_histogram(ranges, angles, margin, v)
    chosen = _select_valley(hist, pos, goal, v, prev_heading)
    vec = _finish(chosen, ranges, angles, v.danger_dist_base + margin, None,
                  v, eps, prev_heading)
    return vec, hist, v.sigma_hist(margin)


def pvfh_scan(pos, world, goal, margin, v: VfhConfig, eps, prev_heading):
    """pvfh_scan.m port: instantaneous noise-free scan of the static world."""
    max_sense = v.sense_range_base + margin
    nb = v.n_beams if v.n_beams else 4 * v.num_sectors
    ranges, angles = lidar_scan(pos, world, nb, max_sense)
    return pvfh_core(ranges, angles, pos, goal, margin, v, eps, prev_heading)[0]


def pvfh_predict(pos, world, movers, v_self, t_now, goal, margin,
                 v: VfhConfig, eps, prev_heading):
    """pvfh_predict.m port: noise-free scan + movers over the horizon."""
    max_sense = v.sense_range_base + margin
    nb = v.n_beams if v.n_beams else 4 * v.num_sectors
    ranges, angles = lidar_scan(pos, world, nb, max_sense)
    if movers and v.pred_lead > 0:
        for dt in _horizon_samples(v.pred_lead, v.pred_dt):
            mk = [m.eval_at(t_now + dt) for m in movers]
            ranges = np.minimum(ranges, _movers_only_scan(
                pos, mk, nb, max_sense, angles))
    return pvfh_core(ranges, angles, pos, goal, margin, v, eps, prev_heading)[0]


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
# external baselines (local_fgm / local_dwa / local_mpc, line for line)
# ---------------------------------------------------------------------------

def _obstacle_points(sc, pos):
    hit = (sc["ranges"] < sc["max_sense"]) & np.isfinite(sc["ranges"])
    r = np.maximum(sc["ranges"][hit], 0.0)
    a = sc["angles"][hit]
    return pos[0] + r * np.cos(a), pos[1] + r * np.sin(a)


def local_fgm(sc, pos, goal, margin, cfg: VfhConfig, prev_heading):
    """Follow-the-Gap (Sezer & Gokasan 2012): widest free run of beams after
    body-radius bubbles; heading = goal bearing pulled toward the gap centre
    with weight alpha/d_min."""
    r = np.maximum(sc["ranges"], 1e-3)
    nb = len(r)
    dth = sc["dth"]
    d_thr = margin + cfg.react_gap
    full_circle = abs(nb * dth - 2 * math.pi) < 1e-6
    blocked = np.zeros(nb, dtype=bool)
    for b in np.where(r < d_thr)[0]:
        gamma = math.asin(min(1.0, margin / max(float(r[b]), margin)))
        k = int(math.ceil(gamma / dth))
        if full_circle:
            blocked[np.arange(b - k, b + k + 1) % nb] = True
        else:
            blocked[max(0, b - k):min(nb, b + k + 1)] = True
    goal_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
    free = ~blocked
    if not free.any():
        return np.array([math.cos(prev_heading), math.sin(prev_heading)])
    if free.all():
        phi_gap = goal_heading
    else:
        if full_circle:
            shift = int(np.argmax(blocked))
            f = np.roll(free, -shift)
        else:
            shift, f = 0, free
        d = np.diff(np.concatenate([[0], f.astype(int), [0]]))
        st = np.where(d == 1)[0]
        en = np.where(d == -1)[0] - 1
        gi = int(np.argmax(en - st))
        mid = (st[gi] + en[gi]) / 2.0 + shift          # 0-based beam index
        phi_gap = _wrap_pi(sc["angles"][0] + mid * dth)
    d_min = max(float(r.min()), 1e-3)
    w_gap = cfg.fgm_alpha / d_min
    phi = goal_heading + _wrap_pi(phi_gap - goal_heading) * w_gap / (w_gap + 1)
    return np.array([math.cos(phi), math.sin(phi)])


def local_dwa(sc, pos, goal, margin, cfg: VfhConfig, prev_heading, V):
    """Dynamic-Window arc sampling (Fox et al. 1997), turn-rate window only:
    the mission layer owns the speed."""
    P = {"w_max": 1.5, "n_w": 31, "T": 2.0, "T_adm": 1.0, "dt": 0.2,
         "a_h": 0.8, "a_c": 0.2, "tau": 1.0, "clr_cap": 1.0}
    P.update(cfg.dwa)
    ox, oy = _obstacle_points(sc, pos)
    ws = np.linspace(-P["w_max"], P["w_max"], int(P["n_w"]))
    ts = P["dt"] * np.arange(1, int(round(P["T"] / P["dt"])) + 1)
    n_adm = int(np.sum(ts <= P["T_adm"] + 1e-9))
    th0 = prev_heading
    th = th0 + ws[:, None] * ts[None, :]
    straight = np.abs(ws) < 1e-6
    wsafe = np.where(straight, 1.0, ws)[:, None]
    px = np.where(straight[:, None], pos[0] + V * ts[None, :] * math.cos(th0),
                  pos[0] + (V / wsafe) * (np.sin(th) - math.sin(th0)))
    py = np.where(straight[:, None], pos[1] + V * ts[None, :] * math.sin(th0),
                  pos[1] - (V / wsafe) * (np.cos(th) - math.cos(th0)))
    if len(ox) == 0:
        clr_all = np.full(px.shape, np.inf)
    else:
        d2 = (px[:, :, None] - ox) ** 2 + (py[:, :, None] - oy) ** 2
        clr_all = np.sqrt(d2.min(axis=2)) - margin
    clr_adm = clr_all[:, :n_adm].min(axis=1)
    clr = clr_all.min(axis=1)
    gb = np.arctan2(goal[1] - py[:, -1], goal[0] - px[:, -1])
    head = 1 - np.abs(_wrap_pi(gb - th[:, -1])) / math.pi
    score = P["a_h"] * head + P["a_c"] * np.minimum(
        np.maximum(clr, 0.0), P["clr_cap"]) / P["clr_cap"]
    score = np.where(clr_adm > 0, score, -np.inf)
    if np.isfinite(score).any():
        best_w = ws[int(np.argmax(score))]
    else:                                   # boxed in: max-clearance arc
        best_w = ws[int(np.argmax(clr_adm))]
    phi = th0 + max(min(best_w * P["tau"], math.pi / 2), -math.pi / 2)
    return np.array([math.cos(phi), math.sin(phi)])


_MPC_LATTICE = {}


def local_mpc(sc, pos, goal, margin, cfg: VfhConfig, prev_heading, V):
    """Receding-horizon lattice MPC: exhaustive search over turn-rate
    sequences (n_seg segments x levels), soft obstacle barrier + hard
    clearance over T_hard, first move applied."""
    P = {"n_seg": 3, "seg_T": 0.8, "levels": (-1.2, -0.6, 0.0, 0.6, 1.2),
         "dt": 0.2, "T_hard": 1.0, "d_safe": 0.5, "w_obs": 4.0,
         "w_smooth": 0.05, "w_jerk": 0.05, "tau": 1.0}
    P.update(cfg.mpc)
    levels = np.asarray(P["levels"], dtype=float)
    key = (int(P["n_seg"]), tuple(levels), P["seg_T"], P["dt"])
    if key not in _MPC_LATTICE:
        L, ns = len(levels), int(P["n_seg"])
        nq = L ** ns
        q = np.arange(nq)
        idx = np.column_stack([(q // L ** s) % L for s in range(ns)])
        Wseg = levels[idx]
        per = max(1, int(round(P["seg_T"] / P["dt"])))
        _MPC_LATTICE[key] = (Wseg, np.repeat(Wseg, per, axis=1))
    Wseg, W = _MPC_LATTICE[key]
    nq, T = W.shape
    th = prev_heading + np.cumsum(W * P["dt"], axis=1)
    px = pos[0] + np.cumsum(V * np.cos(th) * P["dt"], axis=1)
    py = pos[1] + np.cumsum(V * np.sin(th) * P["dt"], axis=1)
    ox, oy = _obstacle_points(sc, pos)
    if len(ox) == 0:
        clr = np.full((nq, T), np.inf)
    else:
        d2 = (px[:, :, None] - ox) ** 2 + (py[:, :, None] - oy) ** 2
        clr = np.sqrt(d2.min(axis=2)) - margin
    n_hard = min(T, max(1, int(round(P["T_hard"] / P["dt"]))))
    hard = clr[:, :n_hard].min(axis=1)
    feasible = hard > 0
    d0 = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
    prog = d0 - np.hypot(goal[0] - px[:, -1], goal[1] - py[:, -1])
    bar = np.mean(np.maximum(0.0, P["d_safe"] - clr) ** 2, axis=1)
    jerk = np.mean(np.abs(np.diff(
        np.column_stack([np.zeros(nq), Wseg]), axis=1)), axis=1)
    J = -prog + P["w_obs"] * bar + P["w_smooth"] * np.mean(np.abs(W), axis=1) \
        + P["w_jerk"] * jerk
    if feasible.any():
        J = np.where(feasible, J, np.inf)
    else:
        J = -hard
    best = int(np.argmin(J))
    phi = prev_heading + max(min(Wseg[best, 0] * P["tau"], math.pi / 2),
                             -math.pi / 2)
    return np.array([math.cos(phi), math.sin(phi)])


# ---------------------------------------------------------------------------
# algorithm dispatch (steering_dispatch.m port)
# ---------------------------------------------------------------------------

def _dispatch(pos, goal, w_att, world, movers, t_now, margin, cfg: VfhConfig,
              algo, eps, prev_heading, rng, forming, V):
    algo = algo.lower()
    pos = np.asarray(pos, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if algo == "apf":
        sc = _sense(pos, world, movers, t_now, cfg, margin, False,
                    prev_heading, rng)
        a = apf_obstacle_repulsion(sc["ranges"], sc["angles"], sc["danger"],
                                   cfg.avoid_falloff_power, eps, sc["dth"])
        na = float(np.linalg.norm(a))
        if na > eps:
            a = a / na
        if not forming:
            return a                    # the caller's vec_goal is the pull
        vec = w_att * (goal - pos) / (float(np.linalg.norm(goal - pos)) + eps) \
            + cfg.avoid_gain * a
        nv = float(np.linalg.norm(vec))
        return vec / nv if nv > eps else vec
    if algo in ("vfh", "pvfh", "pvfh_filt"):
        sc = _sense(pos, world, movers, t_now, cfg, margin, algo != "vfh",
                    prev_heading, rng)
        ranges = sc["ranges"]
        if (algo == "pvfh_filt" and not forming
                and cfg.sense.get("kind", "lidar") == "lidar"):
            s0 = np.full_like(ranges, float(cfg.sense.get("sigma0", 0.02)))
            ranges, _ = range_filter(ranges, s0, t_now, 0.08, cfg.filters,
                                     "pvfh_filt")
        return _vfh_from_scan(ranges, sc["angles"], pos, goal, margin, cfg,
                              eps, prev_heading, sc["dth"], sc["max_sense"],
                              sc["danger"])
    if algo == "pvfh_prob":
        use_filter = (not forming and cfg.rfilter
                      and cfg.sense.get("kind", "lidar") == "lidar")
        return pvfh_prob(pos, world, movers, t_now, goal, margin, cfg, eps,
                         prev_heading, rng, use_filter)
    if algo in ("fgm", "dwa", "mpc"):
        sc = _sense(pos, world, movers, t_now, cfg, margin,
                    cfg.baseline_horizon, prev_heading, rng)
        if algo == "fgm":
            return local_fgm(sc, pos, goal, margin, cfg, prev_heading)
        if algo == "dwa":
            return local_dwa(sc, pos, goal, margin, cfg, prev_heading, V)
        return local_mpc(sc, pos, goal, margin, cfg, prev_heading, V)
    raise ValueError(f"unknown algo {algo!r}")


def steering_rigid(pos, goal, w_att, world, movers, t_now, margin,
                   cfg: VfhConfig, algo, eps, prev_heading, v_self=None,
                   rng=None, V=0.6):
    """Transport-phase dispatch ('rigid'): returns (vec, comp_ms). The caller
    blends w_goal*vec_goal + w_vfh*vec outside. `world` is the STATIC world;
    movers are scanned from the contract."""
    t0 = time.perf_counter()
    vec = _dispatch(pos, goal, w_att, world, movers, t_now, margin, cfg, algo,
                    eps, prev_heading, rng, False, V)
    return vec, 1000.0 * (time.perf_counter() - t0)


def steering_point(pos, goal, w_att, world, movers, t_now, robot_radius,
                   cfg: VfhConfig, algo, eps, prev_heading, rng=None, V=0.5):
    """Forming-phase dispatch ('point'): single robot, the caller passes the
    APF virtual goal. For 'apf' the attraction is added here (cascaded)."""
    t0 = time.perf_counter()
    vec = _dispatch(pos, goal, w_att, world, movers, t_now, robot_radius, cfg,
                    algo, eps, prev_heading, rng, True, V)
    return vec, 1000.0 * (time.perf_counter() - t0)
