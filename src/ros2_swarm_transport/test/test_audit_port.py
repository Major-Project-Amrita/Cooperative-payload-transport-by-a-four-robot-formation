"""Tests of the October 2026 audit port (ROS2_REPLICATION.md section 9).

(a) the safety gate: a full mission of every law has zero static contact
(b) the ToF fan follows the body heading and reports dth = fov / n
(c) vfh and pvfh differ only in the mover horizon
(d) pvfh_prob -> pvfh as sigma0 -> 0
(e) kernel reaction distance grows with s_b up to 1.757 sigma_h (Prop. 2)
(f) fgm / dwa / mpc return unit vectors and steer away from a wall ahead
plus: the range filter, the exact separating-axis gate, the manifest.
"""
import math
import os
import sys

import numpy as np
import pytest

from ros2_swarm_transport.geometry import (Box, Mover, World,
                                           formation_clearance,
                                           formation_offsets)
from ros2_swarm_transport.headless_runner import run_mission
from ros2_swarm_transport.vfh import (ALGOS, VfhConfig, kernel_reaction_distance,
                                      range_filter, scan_ranges_dyn,
                                      steering_rigid)

HERE = os.path.dirname(os.path.abspath(__file__))
W = os.path.join(HERE, "..", "worlds")
SDF = os.path.join(W, "swarm_world_dynamic.sdf")
JSON = os.path.join(W, "dynamic_obstacles.json")
EPS = 1e-6


def wall_world(x=1.5):
    return World(boxes=[Box(np.array([x, 0.0]), 0.0, np.array([0.1, 3.0]), "wall")])


# ---- (a) ------------------------------------------------------------------
@pytest.mark.parametrize("algo", ALGOS)
def test_no_static_contact_full_mission(algo):
    rows, S = run_mission(SDF, JSON, algo=algo, seed=1, verbose=False)
    assert S["coll_static"] == 0
    # a blocked body never changes heading into a colliding pose: every
    # logged pose is clear of the static world
    assert min(r["clearance_static"] for r in rows if r["phase"] == 2) > 0.18


def test_same_seed_is_deterministic():
    a = run_mission(SDF, JSON, algo="pvfh_prob", seed=3, verbose=False)[0]
    b = run_mission(SDF, JSON, algo="pvfh_prob", seed=3, verbose=False)[0]
    assert len(a) == len(b)
    assert all(x["leader_x"] == y["leader_x"] for x, y in zip(a, b))


# ---- (b) ------------------------------------------------------------------
def test_tof_fan_follows_heading():
    v = VfhConfig(n_beams=16, sense={"kind": "tof", "fov_deg": 60.0})
    w = World()
    for bore in (0.0, 1.2, 3.1, -2.5):
        r, ang, dth = scan_ranges_dyn(np.zeros(2), w, [], 0.0, 16, 5.0, v,
                                      boresight=bore)
        assert dth == pytest.approx(math.radians(60.0) / 16)
        dev = np.abs((ang - bore + math.pi) % (2 * math.pi) - math.pi)
        assert dev.max() <= math.radians(30.0) + 1e-9
        assert np.all((ang >= -math.pi) & (ang < math.pi))
    r, ang, dth = scan_ranges_dyn(np.zeros(2), w, [], 0.0, 288, 5.0, VfhConfig())
    assert dth == pytest.approx(2 * math.pi / 288)


def test_instantaneous_scan_still_sees_movers():
    m = Mover("m", "box", np.array([0.7, 0.7]), 0.5,
              np.array([[2.0, 0.0], [2.0, 5.0]]), 0.0)
    r, _, _ = scan_ranges_dyn(np.zeros(2), World(), [m], 0.0, 288, 5.0,
                              VfhConfig(), pred_lead=0.0)
    assert r.min() == pytest.approx(1.65, abs=1e-6)


# ---- (c) ------------------------------------------------------------------
def test_vfh_and_pvfh_differ_only_in_horizon():
    w, cfg = wall_world(), VfhConfig()
    pos, goal = np.array([-1.0, 0.3]), np.array([4.0, 0.0])
    a, _ = steering_rigid(pos, goal, 1.0, w, [], 0.0, 1.0, cfg, "vfh", EPS, 0.0,
                          rng=np.random.default_rng(11))
    b, _ = steering_rigid(pos, goal, 1.0, w, [], 0.0, 1.0, cfg, "pvfh", EPS, 0.0,
                          rng=np.random.default_rng(11))
    assert np.array_equal(a, b)                       # no movers: identical
    m = Mover("m", "box", np.array([0.7, 0.7]), 0.5,
              np.array([[0.5, -3.0], [0.5, 3.0]]), 3.0)
    a, _ = steering_rigid(pos, goal, 1.0, World(), [m], 0.0, 1.0, cfg, "vfh",
                          EPS, 0.0, rng=np.random.default_rng(11))
    b, _ = steering_rigid(pos, goal, 1.0, World(), [m], 0.0, 1.0, cfg, "pvfh",
                          EPS, 0.0, rng=np.random.default_rng(11))
    assert not np.array_equal(a, b)                   # the horizon matters


# ---- (d) ------------------------------------------------------------------
def test_pvfh_prob_equals_pvfh_as_sigma_to_zero():
    w = wall_world()
    cfg = VfhConfig(sense={"kind": "lidar", "sigma0": 1e-9})
    for y in (-0.8, 0.0, 0.3, 1.1):
        pos, goal = np.array([-1.0, y]), np.array([4.0, 0.0])
        a, _ = steering_rigid(pos, goal, 1.0, w, [], 0.0, 1.0, cfg, "pvfh", EPS,
                              0.0, rng=np.random.default_rng(1))
        b, _ = steering_rigid(pos, goal, 1.0, w, [], 0.0, 1.0, cfg, "pvfh_prob",
                              EPS, 0.0, rng=np.random.default_rng(1))
        assert float(np.linalg.norm(a - b)) < 1e-6


# ---- (e) ------------------------------------------------------------------
def test_reaction_distance_grows_up_to_s_star():
    sh = 1.3
    s = np.linspace(0.0, 1.757 * sh, 400)
    d = np.array([kernel_reaction_distance(x, sh) for x in s])
    assert np.all(np.diff(d) > 0)                     # grows up to s*
    assert d[0] == pytest.approx(sh * math.sqrt(2 * math.log(1 / 0.3)))
    after = [kernel_reaction_distance(x, sh) for x in
             np.linspace(1.76 * sh, 3.0 * sh, 50)]
    assert np.all(np.diff(after) < 0)                 # and shrinks beyond it


# ---- (f) ------------------------------------------------------------------
@pytest.mark.parametrize("algo", ["fgm", "dwa", "mpc"])
def test_baselines_unit_and_avoid_wall(algo):
    cfg = VfhConfig()
    pos, goal = np.array([0.0, 0.4]), np.array([6.0, 0.0])
    free, _ = steering_rigid(pos, goal, 1.0, World(), [], 0.0, 1.0, cfg, algo,
                             EPS, 0.0, rng=np.random.default_rng(2))
    assert float(np.linalg.norm(free)) == pytest.approx(1.0)
    assert abs(math.atan2(free[1], free[0])) < 0.2    # free space: at the goal
    wall = World(boxes=[Box(np.array([1.6, 0.0]), 0.0, np.array([0.1, 1.2]), "w")])
    vec, _ = steering_rigid(pos, goal, 1.0, wall, [], 0.0, 1.0, cfg, algo, EPS,
                            0.0, rng=np.random.default_rng(2))
    assert float(np.linalg.norm(vec)) == pytest.approx(1.0)
    assert abs(math.atan2(vec[1], vec[0])) > 0.25     # turned away


# ---- range filter ---------------------------------------------------------
def test_range_filter_reduces_noise_and_resets_on_jump():
    rng = np.random.default_rng(0)
    st, s = {}, np.full(50, 0.2)
    for k in range(40):
        x, sp = range_filter(3.0 + 0.2 * rng.normal(size=50), s, 0.1 * k, 0.08, st)
    assert np.all(sp < 0.2)
    assert np.abs(x - 3.0).std() < 0.2 * 0.75
    z = np.full(50, 3.0)
    z[7] = 1.0                                        # a mover enters beam 7
    x, sp = range_filter(z, s, 4.0, 0.08, st)
    assert x[7] == 1.0 and sp[7] == pytest.approx(0.2)
    x2, _ = range_filter(z, s, 9.0, 0.08, st)         # time gap: re-initialised
    assert np.array_equal(x2, z)


def test_precise_sensor_is_not_filtered():
    st, s = {}, np.full(4, 1e-4)
    range_filter(np.full(4, 2.0), s, 0.0, 0.08, st)
    x, _ = range_filter(np.full(4, 2.001), s, 0.1, 0.08, st)
    assert np.allclose(x, 2.001, atol=1e-6)           # gain ~ 1


# ---- the gate -------------------------------------------------------------
def _true_gap(center, heading, offsets, box, r, n=400):
    """Brute force: distance between the rounded hull and a box."""
    from ros2_swarm_transport.geometry import _convex_hull
    R = np.array([[math.cos(heading), -math.sin(heading)],
                  [math.sin(heading), math.cos(heading)]])
    H = _convex_hull((R @ offsets.T).T + center)
    pts = []
    for i in range(len(H)):
        a, b = H[i], H[(i + 1) % len(H)]
        pts.append(a + np.linspace(0, 1, n)[:, None] * (b - a))
    hull_d = World(boxes=[box]).distance(np.vstack(pts)).min()
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    Rb = np.array([[c, -s], [s, c]])
    cor = (Rb @ (np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * box.h).T).T + box.c
    bp = np.vstack([cor[i] + np.linspace(0, 1, n)[:, None] * (cor[(i + 1) % 4] - cor[i])
                    for i in range(4)])
    # polygon-polygon distance = min over both boundary samplings
    from matplotlib.path import Path
    inside = Path(H).contains_points(bp).any() or \
        Path(cor).contains_points(np.vstack(pts)).any()
    d2 = min(np.hypot(*(bp[:, None, :] - np.vstack(pts)[None, ::8, :]).reshape(-1, 2).T).min(),
             hull_d)
    return (-1.0 if inside else d2) - r


def test_exact_gate_matches_brute_force_and_matlab_gate_is_conservative():
    rng = np.random.default_rng(4)
    offs = formation_offsets("square", 4, 1.18)
    n_false = 0
    for _ in range(300):
        box = Box(rng.uniform(-2.5, 2.5, 2), rng.uniform(-3, 3),
                  rng.uniform(0.2, 1.0, 2), "b")
        th = rng.uniform(-3, 3)
        w = World(boxes=[box])
        gap = _true_gap(np.zeros(2), th, offs, box, 0.18)
        _, hit_e = formation_clearance(np.zeros(2), th, offs, w, 0.18, sat="exact")
        _, hit_m = formation_clearance(np.zeros(2), th, offs, w, 0.18, sat="matlab")
        if abs(gap) > 0.02:                           # away from the boundary
            assert hit_e == (gap < 0)
        assert hit_m or not hit_e                     # never misses an overlap
        n_false += hit_m and not hit_e
    assert n_false > 0                                # ...but over-reports


# ---- the manifest ---------------------------------------------------------
def test_grid_manifest_matches_matlab_table():
    ref = os.path.join(HERE, "..", "..", "..", "bundle", "reference", "results_all.csv")
    if not os.path.exists(ref):
        pytest.skip("MATLAB results table not present")
    sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "tools"))
    from run_headless_grid import build_jobs, run_name
    import csv
    with open(ref) as fh:
        rows = list(csv.DictReader(fh))
    J = build_jobs()
    assert {j["exp"] + "|" + j["key"] for j in J} == {r["exp"] + "|" + r["key"] for r in rows}
    assert {run_name(j) for j in J} == {r["run_name"] for r in rows}
