"""Unit tests for the sensing + probabilistic layer (sensor_noise.m,
pvfh_prob.m and their Python ports) and the steering dispatch.

Run from the package root:

    python3 -m pytest test/test_pvfh_prob.py -q

The behavioural anchor: with a precise sensor the probabilistic stack must
degenerate to the deterministic one (exact deterministic limit of the
posterior-expected kernel), and with realistic ToF noise it must react
FASTER than the deterministic stack near the reaction envelope without
phantom-blocking free space.
"""

import math

import numpy as np
import pytest

from ros2_swarm_transport.geometry import Box, World
from ros2_swarm_transport.vfh import (VfhConfig, _vfh_from_scan, expected_kernel,
                                      pvfh_prob, scan_ranges_dyn, sensor_noise,
                                      steering_point, steering_rigid)

EPS = 1e-6


def flat_world(box_at=None, half=(0.45, 0.45)):
    """Empty world, optionally one axis-aligned box obstacle."""
    boxes = []
    if box_at is not None:
        boxes.append(Box(np.asarray(box_at, dtype=float), 0.0,
                         np.asarray(half, dtype=float), 'ob'))
    return World(boxes=boxes, circles=[], goal=np.array([2.0, 0.0]),
                 goal_r=1.2)


# ---------------------------------------------------------------------------
# sensor noise model
# ---------------------------------------------------------------------------

def test_sensor_noise_lidar_sigma():
    """lidar mode: homoscedastic noise with std = sigma0."""
    v = VfhConfig(sense={"kind": "lidar", "sigma0": 0.05, "k_rng": 0.03,
                         "max_range": float("inf")})
    truth = np.full(4000, 3.0)
    rng = np.random.default_rng(42)
    noisy, valid, drop = sensor_noise(truth, v, rng)
    assert valid.all() and not drop.any()
    assert abs(noisy.mean() - 3.0) < 0.01
    assert abs(noisy.std() - 0.05) < 0.01


def test_sensor_noise_tof_distance_dependent():
    """tof mode: sigma grows linearly with range (s0 + k_rng*d)."""
    v = VfhConfig(sense={"kind": "tof", "sigma0": 0.02, "k_rng": 0.03,
                         "max_range": float("inf")})
    near = np.full(4000, 1.0)     # sigma = 0.05
    far = np.full(4000, 5.0)      # sigma = 0.17
    sn, _, _ = sensor_noise(near, v, np.random.default_rng(1))
    sf, _, _ = sensor_noise(far, v, np.random.default_rng(2))
    assert sn.std() < sf.std()
    assert abs(sn.std() - 0.05) < 0.01
    assert abs(sf.std() - 0.17) < 0.02


def test_sensor_noise_drop_cut():
    """Returns at/above max_range report max_range and are flagged."""
    v = VfhConfig(sense={"kind": "lidar", "sigma0": 0.02, "k_rng": 0.03,
                         "max_range": 5.0})
    ranges = np.array([1.0, 4.98, 6.0])
    noisy, valid, drop = sensor_noise(ranges, v, np.random.default_rng(3))
    assert valid.all()
    assert drop[2]                       # clean 6.0 -> dropped
    assert noisy[2] == 5.0
    assert not drop[0] and not drop[1]


def test_sensor_noise_seeded_reproducible():
    v = VfhConfig()
    r = np.full(64, 2.0)
    a, _, _ = sensor_noise(r, v, np.random.default_rng(7))
    b, _, _ = sensor_noise(r, v, np.random.default_rng(7))
    assert np.array_equal(a, b)


# ---------------------------------------------------------------------------
# posterior-expected kernel (eq. 6 of pvfh_prob.m)
# ---------------------------------------------------------------------------

def test_expected_kernel_deterministic_limit():
    """s_b -> 0 must reproduce the deterministic kernel env(d) exactly."""
    sigma_h = 0.760
    d = np.array([0.5, 1.2, 2.5, 4.0])
    env = np.exp(-(d * d) / (2 * sigma_h ** 2))
    u = expected_kernel(d, np.full_like(d, 1e-9), sigma_h)
    assert np.allclose(u, env, rtol=1e-6)


def test_expected_kernel_matches_quadrature():
    """Closed form vs direct expectation integral over N(d_meas, s_b)."""
    sigma_h, s_b, d_meas = 0.760, 0.25, 1.3
    from scipy.stats import norm
    xs = np.linspace(d_meas - 8 * s_b, d_meas + 8 * s_b, 200001)
    env = np.exp(-(xs * xs) / (2 * sigma_h ** 2))
    num = float(np.sum(env * norm.pdf(xs, d_meas, s_b)) * (xs[1] - xs[0]))
    u = float(expected_kernel(np.array([d_meas]), np.array([s_b]), sigma_h)[0])
    assert abs(u - num) < 1e-4


def test_expected_kernel_monotone_in_sigma():
    """A noisier measurement inflates expected occupancy at fixed range."""
    d = np.array([2.0])
    us = [float(expected_kernel(d, np.array([s]), 0.760)[0])
          for s in (0.02, 0.1, 0.2, 0.4)]
    assert us == sorted(us)
    assert all(u < 1.0 for u in us)      # bounded: never saturates


# ---------------------------------------------------------------------------
# pvfh_prob behaviour
# ---------------------------------------------------------------------------

@pytest.fixture()
def free_cfg():
    return VfhConfig(n_beams=288, sense={"kind": "lidar", "sigma0": 0.02,
                                         "k_rng": 0.03, "max_range": float("inf")})


def test_pvfh_prob_open_corridor(free_cfg):
    """Free space: P-VFH heads for the goal like the deterministic stack."""
    pos, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])
    vec = pvfh_prob(pos, flat_world(), [], 0.0, goal, 0.18, free_cfg, EPS, 0.0,
                    np.random.default_rng(0))
    assert abs(math.atan2(vec[1], vec[0])) < 0.35


def test_pvfh_prob_avoids_direct_obstacle(free_cfg):
    """Obstacle dead ahead with its face inside the reaction envelope
    (d_block = 0.18 + react_gap = 1.18 m): the probabilistic stack must not
    steer through it."""
    pos, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])
    vec = pvfh_prob(pos, flat_world([-0.5, 0.0]), [], 0.0, goal, 0.18,
                    free_cfg, EPS, 0.0, np.random.default_rng(0))
    assert abs(math.atan2(vec[1], vec[0])) > 0.3


def test_pvfh_prob_matches_deterministic_at_low_noise():
    """Exact deterministic limit: with a near-noiseless lidar and no movers,
    the probabilistic choice equals the deterministic choice on the same
    scan (both stacks consume scan_ranges_dyn output)."""
    w = flat_world([0.0, 0.0])
    pos, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])
    v = VfhConfig(n_beams=288, sense={"kind": "lidar", "sigma0": 1e-4,
                                      "k_rng": 0.0, "max_range": float("inf")})
    vec_p = pvfh_prob(pos, w, [], 0.0, goal, 0.18, v, EPS, 0.0,
                      np.random.default_rng(5))
    clean, angles = scan_ranges_dyn(pos, w, [], 0.0, 288,
                                    v.sense_range_base + 0.18, v)
    vec_d = _vfh_from_scan(clean, angles, pos, goal, 0.18, v, EPS, 0.0)
    assert float(np.linalg.norm(vec_p - vec_d)) < 5e-3


def test_pvfh_prob_reacts_where_deterministic_coasts():
    """Boundary regime: a return measurably inside the reaction envelope with
    realistic ToF noise. The deterministic histogram (graded kernel) still
    calls the goal corridor free; the probabilistic histogram (noise-inflated
    expected kernel) blocks it and deviates."""
    w = flat_world([0.0, 0.0])
    pos, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])   # d_true = 1.55
    # tof sigma at ~1.2-1.5 m with sigma0=0.35 -> s_b ~ 0.39
    v = VfhConfig(n_beams=288, sense={"kind": "tof", "sigma0": 0.35,
                                      "k_rng": 0.03, "max_range": float("inf")})
    vec_d = _vfh_from_scan(*scan_ranges_dyn(pos, w, [], 0.0, 288,
                                            v.sense_range_base + 0.18, v),
                           pos, goal, 0.18, v, EPS, 0.0)
    vec_p = pvfh_prob(pos, w, [], 0.0, goal, 0.18, v, EPS, 0.0,
                      np.random.default_rng(9))
    dev_d = abs(math.atan2(vec_d[1], vec_d[0]))
    dev_p = abs(math.atan2(vec_p[1], vec_p[0]))
    assert dev_d < 0.2
    assert dev_p > 0.3


def test_pvfh_prob_unknown_floor_no_phantom():
    """max_range drop-cut: an obstacle beyond max_range must not become a
    phantom wall -- max-range returns are floored at p_unknown < threshold.
    (max_range must exceed d_block = r_geom + react_gap = 1.18 m; a sensor
    with max_range inside its own reaction envelope cannot sense at all.)"""
    v = VfhConfig(n_beams=288, sense={"kind": "lidar", "sigma0": 0.02,
                                      "k_rng": 0.03, "max_range": 2.5})
    vec = pvfh_prob(np.array([-2.0, 0.0]), flat_world([3.0, 0.0]), [], 0.0,
                    np.array([2.0, 0.0]), 0.18, v, EPS, 0.0,
                    np.random.default_rng(1))
    assert abs(math.atan2(vec[1], vec[0])) < 0.35


def test_pvfh_prob_seed_reproducible(free_cfg):
    pos, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])
    w = flat_world([0.0, 0.0])
    a = pvfh_prob(pos, w, [], 0.0, goal, 0.18, free_cfg, EPS, 0.0,
                  np.random.default_rng(7))
    b = pvfh_prob(pos, w, [], 0.0, goal, 0.18, free_cfg, EPS, 0.0,
                  np.random.default_rng(7))
    assert np.array_equal(a, b)


def test_pvfh_prob_mover_horizon_seen():
    """A mover contracted to arrive on the beam fan within the prediction
    horizon must be treated as blocking by the probabilistic stack."""
    from ros2_swarm_transport.geometry import Mover
    m = Mover('t', 'box', np.array([0.7, 0.7]), 0.5,
              np.array([[4.0, 0.0], [-4.0, 0.0]]), speed=1.0, t0=0.0)
    # at t=5.0 the mover is at (1.0, 0), 1.0 s later (horizon) at (0.0, 0):
    # straight ahead of pos=(-2,0) but currently 3 m away.
    v = VfhConfig(n_beams=288, sense={"kind": "lidar", "sigma0": 0.02,
                                      "k_rng": 0.03, "max_range": float("inf")})
    vec = pvfh_prob(np.array([-2.0, 0.0]), flat_world(), [m], 5.0,
                    np.array([2.0, 0.0]), 0.18, v, EPS, 0.0,
                    np.random.default_rng(2))
    assert abs(math.atan2(vec[1], vec[0])) > 0.3


# ---------------------------------------------------------------------------
# steering dispatch (all four algorithm variants)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("algo", ["apf", "vfh", "pvfh", "pvfh_prob"])
def test_steering_dispatch_all_algos(algo, free_cfg):
    """Every variant returns a finite, bounded steering vector in free space."""
    pos, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])
    w = flat_world()
    vec, ms = steering_rigid(pos, goal, 1.0, w, [], 0.0, 0.18, free_cfg,
                             algo, EPS, 0.0, np.zeros(2),
                             np.random.default_rng(3))
    assert np.all(np.isfinite(vec))
    assert float(np.linalg.norm(vec)) <= 1.0 + 1e-9
    assert ms >= 0.0
    if algo != "apf":                    # apf returns a zero reflex when free
        assert float(np.linalg.norm(vec)) > 0.9


def test_steering_point_apf_attracts(free_cfg):
    """Forming-phase pure-APF: attraction toward the virtual goal dominates
    when no obstacle is close."""
    vec, _ = steering_point(np.array([-2.0, 0.0]), np.array([2.0, 0.0]), 2.0,
                            flat_world(), [], 0.0, 0.18, free_cfg, "apf",
                            EPS, 0.0, np.random.default_rng(4))
    assert abs(math.atan2(vec[1], vec[0])) < 0.1
