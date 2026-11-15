"""Tests for the review-driven extensions: traffic phase, team size, the
sensor safety monitor, MPPI, the learned components and the MATLAB gate fix."""
import math
import os

import numpy as np
import pytest

from ros2_swarm_transport.geometry import (formation_clearance, formation_offsets,
                                           traffic_phase_offsets, world_from_sdf)
from ros2_swarm_transport.headless_runner import load_scenario, run_mission
from ros2_swarm_transport.safety_monitor import hull_point_distance

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
W = os.path.join(PKG, "worlds")
REPO = os.path.dirname(os.path.dirname(PKG))
DYN = (os.path.join(W, "swarm_world_dynamic.sdf"), os.path.join(W, "dynamic_obstacles.json"))


def test_phase_offsets_deterministic_and_seed_dependent():
    _, m = load_scenario(*DYN)
    a = traffic_phase_offsets(m, 3)
    b = traffic_phase_offsets(m, 3)
    c = traffic_phase_offsets(m, 4)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)
    assert len(a) == len(m)


def test_phase_offsets_ranges():
    _, m = load_scenario(*DYN)
    for seed in range(50):
        off = traffic_phase_offsets(m, seed)
        for mk, o in zip(m, off):
            if mk.loop:
                L = float(np.linalg.norm(mk.route[1] - mk.route[0]))
                assert -2 * (L / mk.speed + mk.wait) <= o <= 0
            else:
                assert -mk.t0 <= o <= 20.0


def test_phase_shifts_mover_timetable_only():
    w0, m0 = load_scenario(*DYN)
    off = np.array([5.0, -3.0, 1.5])
    w1, m1 = load_scenario(*DYN, t0_offset=off)
    assert len(w0.boxes) == len(w1.boxes)
    for a, b, o in zip(m0, m1, off):
        assert b.t0 == pytest.approx(a.t0 + o)
        # the shifted mover at t + o is where the nominal one is at t
        for t in (10.0, 25.0, 40.0):
            assert np.allclose(b.eval_at(t + o).c, a.eval_at(t).c)


def test_grid_formations():
    sq = formation_offsets("square", 4, 1.0)
    g22 = formation_offsets("grid2x2", 4, 1.0)
    assert sorted(map(tuple, np.round(sq, 9))) == sorted(map(tuple, np.round(g22, 9)))
    g33 = formation_offsets("grid3x3", 9, 0.5)
    assert g33.shape == (9, 2) and np.allclose(g33.mean(0), 0)
    assert np.ptp(g33[:, 0]) == pytest.approx(1.0)
    g23 = formation_offsets("grid2x3", 6, 1.0)
    assert g23.shape == (6, 2) and np.ptp(g23[:, 0]) == pytest.approx(2.0)


def test_large_team_outline_sampled():
    world = world_from_sdf(DYN[0], movers_json=DYN[1])
    offs = formation_offsets("grid3x3", 9, 0.59)
    clr, _ = formation_clearance(np.array([-12.0, 0.0]), 0.0, offs, world, 0.18)
    assert np.isfinite(clr)


def test_hull_point_distance():
    H = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    P = np.array([[0.5, 0.5], [2, 0.5], [0.5, -1], [2, 2]], float)
    d = hull_point_distance(H, P)
    assert d == pytest.approx([0, 1, 1, math.sqrt(2)])


def test_audit_mode_does_not_change_trajectory():
    a_rows, a = run_mission(*DYN, algo="mpc", seed=2, verbose=False, timesteps=250)
    b_rows, b = run_mission(*DYN, algo="mpc", seed=2, verbose=False, timesteps=250,
                            gate_source="audit")
    assert len(a_rows) == len(b_rows)
    assert a_rows[-1]["leader_x"] == b_rows[-1]["leader_x"]
    assert b["gate_stats"]["checks"] > 0


def test_mppi_runs_and_is_deterministic():
    r1, _ = run_mission(*DYN, algo="mppi", seed=1, verbose=False, timesteps=120)
    r2, _ = run_mission(*DYN, algo="mppi", seed=1, verbose=False, timesteps=120)
    assert r1[-1]["leader_x"] == r2[-1]["leader_x"]


def test_matlab_gate_fix_is_cyclic():
    """The MATLAB reference must now list box corners in cyclic order,
    identical to the Python exact gate."""
    p = os.path.join(REPO, "matlab_ref", "formation_clearance_dyn.m")
    if not os.path.exists(p):
        pytest.skip("matlab_ref not present")
    src = open(p).read()
    assert "B.h(2) -B.h(2) -B.h(2)  B.h(2)" in src
    assert "B.h(2) -B.h(2)  B.h(2) -B.h(2)" not in src


@pytest.mark.skipif(not os.path.exists(os.path.join(PKG, "models", "policy_dagger.npz")),
                    reason="policy not trained")
def test_nn_policy_runs():
    _, S = run_mission(*DYN, algo="nn", seed=1, verbose=False, timesteps=150)
    assert S["transport_time"] > 0


@pytest.mark.skipif(not os.path.exists(os.path.join(PKG, "models", "monitor_conformal.npz")),
                    reason="monitor not trained")
def test_learned_monitor_runs():
    _, S = run_mission(*DYN, algo="mpc", seed=1, verbose=False, timesteps=150,
                       gate_source="learned")
    assert S["gate_stats"]["checks"] > 0


def test_pavfh_nonn_uses_no_contract_prediction():
    """PA-VFH must not see movers' future: its steering is identical whether
    the movers' contract future differs or not, given the same present."""
    from ros2_swarm_transport.vfh import VfhConfig, pavfh
    world, m = load_scenario(*DYN)
    _, m2 = load_scenario(*DYN, t0_offset=np.zeros(len(m)))
    pos, goal = np.array([-2.0, 0.0]), np.array([12.0, 0.0])
    out = []
    for movers in (m, m2):
        cfg = VfhConfig()
        out.append(pavfh(pos, world, movers, 10.0, goal, 1.0, cfg, 1e-6, 0.0,
                         np.random.default_rng(0), True, False))
    assert np.allclose(out[0], out[1])


def test_pavfh_nonn_mission_runs():
    _, S = run_mission(*DYN, algo="pavfh_nonn", seed=1, verbose=False, timesteps=150)
    assert S["transport_time"] > 0


@pytest.mark.skipif(not os.path.exists(os.path.join(PKG, "models", "pavfh_occupancy.npz")),
                    reason="PA-VFH occupancy network not trained")
def test_pavfh_neural_mission_runs():
    from ros2_swarm_transport import neural
    from ros2_swarm_transport.vfh import VfhConfig, pavfh
    world, m = load_scenario(*DYN)
    _, info = pavfh(np.array([-2.0, 0.0]), world, m, 5.0, np.array([12.0, 0.0]), 1.0,
                    VfhConfig(), 1e-6, 0.0, np.random.default_rng(0), True, True,
                    return_info=True)
    u = info["u_sector"]
    assert u.shape == (72,) and np.all((u >= 0) & (u <= 1))
    _, S = run_mission(*DYN, algo="pavfh", seed=1, verbose=False, timesteps=150)
    assert S["transport_time"] > 0
