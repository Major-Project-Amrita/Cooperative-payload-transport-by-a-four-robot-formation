"""Real-sensor input for the steering laws and the safety monitors.

In simulation every scan is ray-cast against the world model
(`vfh.scan_ranges_dyn`). On robots, the scans come from the robots' own
range sensors. `PointCloudScan` fuses the robots' `LaserScan`s into one
point cloud in the map frame and answers the same query the ray-caster
answers: "what does a scan of nb beams over [amin, amax) see from `pos`?".
Setting `cfg.scan_provider = PointCloudScan(...)` routes EVERY scan of the
sensor-only laws (PA-VFH, PA-VFH (no NN), the forming phase) and of the
sensor and learned safety monitors to the real data. No synthetic noise is
added to real data (`vfh.sensor_noise` passes it through); the noise model
still supplies the per-beam sigma the range filter and the monitor use.

Laws that read the traffic contract's future (PA-VFH-K, and the ray-cast
prediction in the other baselines) and the ground-truth gate
(`gate_source="truth"`) need the world model by construction; on robots use
`algo:=pavfh` with `gate_source:=sensor` or `learned`.

Self-filtering: returns from the team's own robots, and from the payload
footprint while it is carried, are removed before the query.
"""
import math

import numpy as np


def _wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def laserscan_to_points(pose, ranges, angle_min, angle_increment, range_min=0.0,
                        range_max=float("inf"), mount=(0.0, 0.0, 0.0)):
    """One planar scan -> points in the map frame.

    pose: (x, y, yaw) of the robot base in the map frame; mount: (x, y, yaw)
    of the sensor in the base frame. Invalid returns (NaN, inf, out of
    [range_min, range_max)) are dropped."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    mx, my, myaw = mount
    px = mx + r * np.cos(a + myaw)
    py = my + r * np.sin(a + myaw)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * px - s * py, y + s * px + c * py])


class PointCloudScan:
    """Fused team point cloud -> virtual scan from any query position."""

    def __init__(self, robot_radius=0.18, self_margin=0.10, max_age=0.5):
        self.r = float(robot_radius)
        self.margin = float(self_margin)
        self.max_age = float(max_age)
        self.pts = np.zeros((0, 2))
        self.stamp = -math.inf
        self.n_raw = self.n_self = 0

    def update(self, point_sets, robot_pos, stamp, payload_poly=None):
        """point_sets: list of (N_i, 2) map-frame arrays (one per robot);
        robot_pos: (K, 2) team positions; payload_poly: (M, 2) polygon of the
        carried load (None while not carrying)."""
        P = (np.vstack([p for p in point_sets if len(p)])
             if any(len(p) for p in point_sets) else np.zeros((0, 2)))
        self.n_raw = len(P)
        keep = np.ones(len(P), dtype=bool)
        R = np.asarray(robot_pos, dtype=float).reshape(-1, 2)
        if len(P) and len(R):
            d = np.linalg.norm(P[:, None, :] - R[None, :, :], axis=2).min(axis=1)
            keep &= d > self.r + self.margin
        if len(P) and payload_poly is not None and len(payload_poly) >= 3:
            keep &= ~_inside(P, _inflate(np.asarray(payload_poly, float), self.margin))
        self.n_self = int((~keep).sum())
        self.pts = P[keep]
        self.stamp = float(stamp)

    def fresh(self, t_now):
        return (t_now - self.stamp) <= self.max_age

    def __call__(self, pos, nb, max_sense, amin, amax):
        """Same contract as geometry.lidar_scan: (ranges, beam angles);
        beams with no return read max_sense."""
        angles = amin + (amax - amin) * np.arange(nb) / nb
        ranges = np.full(nb, float(max_sense))
        if not len(self.pts):
            return ranges, angles
        d = self.pts - np.asarray(pos, dtype=float)[None, :]
        rr = np.hypot(d[:, 0], d[:, 1])
        b = np.arctan2(d[:, 1], d[:, 0])
        dth = (amax - amin) / nb
        full = abs((amax - amin) - 2 * math.pi) < 1e-9
        rel = _wrap(b - amin) % (2 * math.pi) if full else _wrap(b - amin - math.pi) + math.pi
        k = np.floor(rel / dth + 0.5).astype(int)
        if full:
            k %= nb
        ok = (rr < max_sense) & (k >= 0) & (k < nb)
        np.minimum.at(ranges, k[ok], rr[ok])
        return ranges, angles


def _inflate(poly, m):
    c = poly.mean(axis=0)
    v = poly - c
    n = np.linalg.norm(v, axis=1, keepdims=True)
    return c + v * (1.0 + m / np.maximum(n, 1e-9))


def _inside(P, poly):
    """Points inside a convex polygon (vertices in either order)."""
    n = len(poly)
    sgn = None
    inside = np.ones(len(P), dtype=bool)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        cr = (b[0] - a[0]) * (P[:, 1] - a[1]) - (b[1] - a[1]) * (P[:, 0] - a[0])
        if sgn is None:
            area = sum((poly[j][0] * poly[(j + 1) % n][1] - poly[(j + 1) % n][0] * poly[j][1])
                       for j in range(n))
            sgn = 1.0 if area > 0 else -1.0
        inside &= sgn * cr >= 0
    return inside
