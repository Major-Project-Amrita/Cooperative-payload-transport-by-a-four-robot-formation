"""Real-sensor input (sensing.py): LaserScan -> map points -> virtual scan."""
import math
import os

import numpy as np
import pytest

from ros2_swarm_transport.geometry import lidar_scan, world_from_sdf
from ros2_swarm_transport.sensing import PointCloudScan, laserscan_to_points
from ros2_swarm_transport.vfh import VfhConfig, pavfh, scan_ranges_dyn, sensor_noise

HERE = os.path.dirname(os.path.abspath(__file__))
W = os.path.join(os.path.dirname(HERE), "worlds")
SDF = os.path.join(W, "swarm_world_dynamic.sdf")


def test_laserscan_to_points_transform():
    # a single return straight ahead of a robot at (1, 2) facing +y, sensor
    # mounted 0.1 m forward
    P = laserscan_to_points((1.0, 2.0, math.pi / 2), [1.0, float("inf"), float("nan")],
                            0.0, 0.1, 0.05, 8.0, mount=(0.1, 0.0, 0.0))
    assert P.shape == (1, 2)
    assert P[0] == pytest.approx([1.0, 3.1])


def test_self_filter_removes_team_and_load():
    c = PointCloudScan(robot_radius=0.18, self_margin=0.1)
    team = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    pts = np.array([[0.05, 0.0],      # on robot 1
                    [0.5, 0.5],       # under the load
                    [3.0, 0.5]])      # a real obstacle
    c.update([pts], team, 0.0, payload_poly=team)
    assert c.pts.tolist() == [[3.0, 0.5]]
    assert c.n_self == 2


def test_virtual_scan_matches_raycast():
    """Dense scans from four robots, fused, queried from the centroid,
    reproduce the direct ray-cast where the obstacle surface was observed."""
    world = world_from_sdf(SDF)
    team = np.array([[-6.6, -0.6], [-5.4, -0.6], [-5.4, 0.6], [-6.6, 0.6]])
    sets = []
    for p in team:
        r, a = lidar_scan(p, world, 2880, 8.0)
        r = np.where(r >= 8.0 - 1e-6, np.inf, r)
        sets.append(laserscan_to_points((p[0], p[1], 0.0), r, a[0], a[1] - a[0]))
    c = PointCloudScan()
    c.update(sets, team, 0.0)
    q = team.mean(axis=0)
    ref, ang = lidar_scan(q, world, 288, 5.0)
    got, ang2 = c(q, 288, 5.0, -math.pi, math.pi)
    assert np.allclose(ang, ang2)
    seen = ref < 5.0 - 1e-6
    err = np.abs(got[seen] - ref[seen])
    assert np.median(err) < 0.03
    assert np.mean(err < 0.10) > 0.9


def test_provider_routes_all_scans_and_skips_synthetic_noise():
    world = world_from_sdf(SDF)
    cfg = VfhConfig()
    cfg.scan_provider = lambda pos, nb, ms, a0, a1: (np.full(nb, 2.5),
                                                    a0 + (a1 - a0) * np.arange(nb) / nb)
    r, a, _ = scan_ranges_dyn(np.zeros(2), world, [], 0.0, 72, 5.0, cfg)
    assert np.all(r == 2.5)
    z, valid, drop = sensor_noise(r, cfg, np.random.default_rng(0))
    assert np.array_equal(z, r) and valid.all() and not drop.any()
    # PA-VFH runs on the provided scan only (world/movers not consulted)
    out = pavfh(np.zeros(2), None, None, 1.0, np.array([5.0, 0.0]), 1.0, cfg, 1e-6, 0.0,
                np.random.default_rng(0), True, False)
    assert np.all(np.isfinite(out))
