"""Sensor-based safety monitor, kept separate from ground-truth screening.

The mission's original safety gate (transport.step_safe) asks the TRUE world
and the TRUE mover contract whether a candidate pose overlaps anything. That
is a sound *evaluation metric* (ground-truth collision screening) but it is
not deployable: a robot only has its own noisy range scan.

This module provides the deployable counterpart:

  SensorSafetyMonitor.observe(pos, heading, t)
      takes one instantaneous scan from the formation centre with the same
      range-noise model as the planners (vfh.sensor_noise), runs the per-beam
      Kalman range filter (vfh.range_filter) and stores the obstacle points
      together with each beam's posterior standard deviation.

  SensorSafetyMonitor.safe(cand_pos, cand_heading)
      accepts the candidate pose only if the payload body (convex hull of
      the slots, rounded by the robot radius) keeps a distance of at least
          robot_radius + k_sigma * sigma_post + v_obs_max * horizon
      from every observed obstacle point. The second term covers range
      uncertainty, the third the motion of anything observed (no access to
      the mover contract: every point may approach at v_obs_max).

The monitor never reads the world geometry or the movers' future; the
raycast that produces the scan is the simulated sensor. The ground-truth
screen keeps running beside it as the METRIC, so every decision can be
scored: a *false accept* is a pose the monitor admitted that the ground truth
flags as a contact, a *false reject* the converse.

Limits (stated in the documentation): 2-D scan from one point, so a body
part occluded from the centre is not observed; points beyond the sensor's
range are treated as free; v_obs_max is an assumed bound on traffic speed.
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import _convex_hull
from .vfh import (VfhConfig, _per_beam_sigma, range_filter, scan_ranges_dyn,
                  sensor_noise)


def _seg_point_dist(a, b, P):
    """Distance from points P (N,2) to segment a-b."""
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-12:
        return np.linalg.norm(P - a, axis=1)
    t = np.clip(((P - a) @ ab) / L2, 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def hull_point_distance(hull, P):
    """Unsigned distance from points P (N,2) to the convex polygon `hull`
    (CCW vertices); 0 for points inside."""
    P = np.atleast_2d(P)
    if len(P) == 0:
        return np.zeros(0)
    H = np.atleast_2d(hull)
    if len(H) == 1:
        return np.linalg.norm(P - H[0], axis=1)
    d = np.min([_seg_point_dist(H[i], H[(i + 1) % len(H)], P)
                for i in range(len(H))], axis=0)
    if len(H) >= 3:
        inside = np.ones(len(P), dtype=bool)
        for i in range(len(H)):
            a, b = H[i], H[(i + 1) % len(H)]
            cross = (b[0] - a[0]) * (P[:, 1] - a[1]) - (b[1] - a[1]) * (P[:, 0] - a[0])
            inside &= cross >= -1e-12
        d = np.where(inside, 0.0, d)
    return d


class SensorSafetyMonitor:
    def __init__(self, offsets, robot_radius, cfg: VfhConfig, rng,
                 n_beams=288, max_range=6.0, k_sigma=3.0, v_obs_max=1.0,
                 horizon=0.2, use_filter=True, world=None, movers=None):
        self.offsets = np.asarray(offsets, dtype=float)
        self.r = float(robot_radius)
        self.cfg = cfg
        self.rng = rng
        self.n_beams = int(n_beams)
        self.max_range = float(max_range)
        self.k_sigma = float(k_sigma)
        self.v_obs_max = float(v_obs_max)
        self.horizon = float(horizon)
        self.use_filter = use_filter
        # the SENSOR's view of the world: used only to raycast the scan
        self._world, self._movers = world, movers or []
        self._state = {}
        self.pts = np.zeros((0, 2))
        self.sig = np.zeros(0)
        self.pts_prev = np.zeros((0, 2))
        self.learned = None          # (model dict, conformal quantile q)
        # decision log for scoring against the ground-truth screen
        self.stats = {"checks": 0, "accept": 0, "reject": 0,
                      "false_accept": 0, "false_reject": 0}

    # -- sensing ----------------------------------------------------------
    def observe(self, pos, heading, t_now):
        """One instantaneous scan (movers where they are NOW, no prediction)."""
        pos = np.asarray(pos, dtype=float)
        v = self.cfg
        clean, ang, _ = scan_ranges_dyn(pos, self._world, self._movers, t_now,
                                        self.n_beams, self.max_range, v,
                                        boresight=heading, pred_lead=0.0)
        z, _, _ = sensor_noise(clean, v, self.rng)
        s = _per_beam_sigma(z, v)
        if self.use_filter:
            z, s = range_filter(z, s, t_now, v.rfilter_q, self._state, "monitor")
        hit = np.isfinite(z) & (z < self.max_range - 1e-6)
        z, a, s = np.maximum(z[hit], 0.0), ang[hit], s[hit]
        self.pts_prev = self.pts if len(self.pts) else None
        self.pts = pos + np.column_stack([z * np.cos(a), z * np.sin(a)])
        if self.pts_prev is None:
            self.pts_prev = self.pts
        self.sig = s

    # -- learned, conformal-calibrated variant -----------------------------
    def use_learned(self, model_name, alpha):
        from .neural import load_model
        m = load_model(model_name)
        key = f"q_{int(round(alpha * 1000)):03d}"
        if key not in m:
            raise KeyError(f"{model_name}: no conformal quantile for alpha={alpha} ({key})")
        self.learned = (m, float(m[key]))

    def features(self, cand_pos, cand_heading):
        from .neural import monitor_features
        return monitor_features(self.pts, self.pts_prev, self.sig,
                                np.asarray(cand_pos, float), cand_heading,
                                self.offsets)

    def learned_safe(self, cand_pos, cand_heading):
        from .neural import monitor_forward
        m, q = self.learned
        pred = monitor_forward(m, self.features(cand_pos, cand_heading))
        return pred - q > self.r

    def required_gap(self):
        return self.r + self.k_sigma * self.sig + self.v_obs_max * self.horizon

    def clearance(self, cand_pos, cand_heading):
        """Monitor's estimate of body clearance minus the required gap (m);
        >= 0 means admit."""
        if len(self.pts) == 0:
            return float("inf")
        c, s = math.cos(cand_heading), math.sin(cand_heading)
        slots = (np.array([[c, -s], [s, c]]) @ self.offsets.T).T + cand_pos
        hull = _convex_hull(slots) if len(slots) > 2 else slots
        d = hull_point_distance(hull, self.pts)
        return float(np.min(d - self.required_gap()))

    def safe(self, cand_pos, cand_heading):
        return self.clearance(np.asarray(cand_pos, float), cand_heading) >= 0.0

    # -- scoring ------------------------------------------------------------
    def score(self, admitted, truth_safe):
        st = self.stats
        st["checks"] += 1
        st["accept" if admitted else "reject"] += 1
        if admitted and not truth_safe:
            st["false_accept"] += 1
        if not admitted and truth_safe:
            st["false_reject"] += 1
