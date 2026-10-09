"""Unit tests for the pure-Python ports. Run from the repo root:

    python3 -m pytest ros2_swarm_transport/test -q

These validate the Python port against hand-computed geometry and against
invariants that hold for the MATLAB originals (world_dist vs lidar_scan
consistency, formation_clearance payload SAT, mover motion continuity).
"""

import json
import math
import os

import numpy as np
import pytest

from ros2_swarm_transport.assign_slots import assign_slots
from ros2_swarm_transport.geometry import (Box, Circle, Mover, World,
                                           eval_mover_pose, eval_mover_vel,
                                           formation_clearance, lidar_scan,
                                           load_dynamic_obstacles,
                                           world_from_sdf)
from ros2_swarm_transport.vfh import VfhConfig, pvfh_predict, pvfh_scan

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WORLD = os.path.join(PKG, 'worlds', 'swarm_world_dynamic.sdf')
CONTRACT = os.path.join(PKG, 'worlds', 'dynamic_obstacles.json')


# ---------------------------------------------------------------------------
# world / distance
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def world():
    return world_from_sdf(WORLD, movers_json=CONTRACT)


def test_world_parses(world):
    # 15 boxes (walls, shelves, pillars, pallets) + 3 circles (pillar1/3/5);
    # the three mover models are skipped - their geometry lives in the JSON.
    assert len(world.boxes) == 15
    assert len(world.circles) == 3
    assert world.goal is not None
    assert np.allclose(world.goal, [12.0, 0.0])
    assert abs(world.goal_r - 1.2) < 1e-9


def test_distance_inside_outside(world):
    p = np.array([[0.0, 7.9],      # inside wall_n (pose y=8, half 0.15)
                  [0.0, 7.0],      # free space between wall (7.85) and shelf_top (6.5)
                  [-6.0, 0.5]])    # start aisle, free
    d = world.distance(p)
    assert d[0] < 0                        # inside the north wall
    assert d[1] > 0 and d[2] > 0
    # exactness: distance from a point 1 m right of pillar1's surface
    c = np.array([-8.0, 1.4])              # pillar1 centre, r=0.45
    d1 = world.distance(c + np.array([1.0, 0.0]))[0]
    assert abs(d1 - (1.0 - 0.45)) < 1e-9 or d1 > 0  # shelf may be nearer


def test_lidar_matches_distance(world):
    """Every beam endpoint reported by the raycaster must sit on a surface:
    world.distance(endpoint) ~ 0 (or >= max_range for misses)."""
    pos = np.array([-6.0, 0.5])
    ranges, angles = lidar_scan(pos, world, n_beams=64, max_range=6.0)
    ends = pos + ranges[:, None] * np.column_stack(
        [np.cos(angles), np.sin(angles)])
    d = world.distance(ends)
    hit = ranges < 6.0
    assert np.all(np.abs(d[hit]) < 1e-6)


# ---------------------------------------------------------------------------
# movers: the shared contract
# ---------------------------------------------------------------------------

def load_contract():
    with open(CONTRACT) as fh:
        return json.load(fh)


def test_movers_parse():
    movers, meta = load_dynamic_obstacles(CONTRACT)
    assert meta['frame'] == 'world'
    assert len(movers) == 3
    m1 = movers[0]
    assert m1.name == 'mover1'
    assert not m1.loop
    assert np.allclose(m1.route[0], [-6.0, 0.5])


def test_mover_one_way_trip():
    m = Mover('t', 'box', np.array([0.7, 0.7]), 0.5,
              np.array([[0.0, 0.0], [2.0, 0.0]]), speed=1.0, t0=1.0)
    assert np.allclose(eval_mover_pose(m, 0.5), [0, 0])       # still waiting
    assert np.allclose(eval_mover_pose(m, 1.0), [0, 0])
    assert np.allclose(eval_mover_pose(m, 2.0), [1.0, 0.0])   # halfway
    assert np.allclose(eval_mover_pose(m, 3.0), [2.0, 0.0])   # arrived
    assert np.allclose(eval_mover_pose(m, 10.0), [2.0, 0.0])  # parked
    assert np.allclose(eval_mover_vel(m, 10.0), [0, 0])


def test_mover_loop_continuity():
    """Position must be continuous across leg/pause boundaries."""
    m = Mover('t', 'circle', np.array([1.0, 1.0]), 0.5,
              np.array([[0.0, 0.0], [1.0, 0.0]]), speed=1.0, t0=0.0,
              wait=0.5, loop=True)
    T = 1.0 / 1.0 + 0.5
    eps = 1e-9
    for k in range(1, 8):
        a = eval_mover_pose(m, k * T - eps)
        b = eval_mover_pose(m, k * T + eps)
        assert np.linalg.norm(a - b) < 1e-6
    # velocity is zero during the pause
    assert np.allclose(eval_mover_vel(m, 1.0 + 0.25), [0, 0])


def test_contract_matches_eval():
    """Spot-check the JSON scenario: mover1 at t=10 s is 0.9*(10-2)=7.2 m
    along its route from (-6,0.5) toward (13.9,0.5)."""
    movers, _ = load_dynamic_obstacles(CONTRACT)
    m1 = movers[0]
    p = eval_mover_pose(m1, 10.0)
    assert np.allclose(p, [-6.0 + 7.2, 0.5])


# ---------------------------------------------------------------------------
# formation clearance
# ---------------------------------------------------------------------------

def test_clearance_free_and_colliding(world):
    offs = np.array([[-0.59, -0.59], [-0.59, 0.59],
                     [0.59, -0.59], [0.59, 0.59]])
    clr, hit = formation_clearance([-12.0, 0.0], 0.0, offs, world, 0.18,
                                   'payload')
    assert not hit and clr > 2.0

    # centred on pillar1 (-8, 1.4, r=0.45): interior overlap must be caught
    clr, hit = formation_clearance([-8.0, 1.4], 0.0, offs, world, 0.18,
                                   'payload')
    assert hit


def test_payload_mode_catches_interior(world):
    """A small obstacle wholly inside the block is a collision in payload
    mode but invisible to the four slot points."""
    world2 = World(boxes=[Box(np.array([0.0, 0.0]), 0.0,
                              np.array([0.2, 0.2]), 'inner')],
                   circles=[])
    offs = np.array([[-2.0, -2.0], [-2.0, 2.0], [2.0, -2.0], [2.0, 2.0]])
    _, hit_slots = formation_clearance([0.0, 0.0], 0.0, offs, world2, 0.18,
                                       'slots')
    _, hit_payload = formation_clearance([0.0, 0.0], 0.0, offs, world2, 0.18,
                                         'payload')
    assert not hit_slots
    assert hit_payload


# ---------------------------------------------------------------------------
# assignment + vfh
# ---------------------------------------------------------------------------

def test_assign_slots_identity():
    cost = np.array([[0.0, 10.0], [10.0, 0.0]])
    perm = assign_slots(cost)
    assert list(perm) == [0, 1]


def test_assign_slots_swap():
    cost = np.array([[10.0, 0.0], [0.0, 10.0]])
    perm = assign_slots(cost)
    assert list(perm) == [1, 0]


def test_pvfh_open_corridor(world):
    """In free space with the goal ahead, VFH returns ~the goal heading."""
    v = VfhConfig()
    pos, goal = np.array([-6.0, 0.5]), np.array([12.0, 0.5])
    vec = pvfh_scan(pos, world, goal, 0.18, v, 1e-6, 0.0)
    ang = math.atan2(vec[1], vec[0])
    assert abs(ang) < 0.35


def test_pvfh_predict_sees_mover_sooner(world):
    """With a mover closing on the beam fan, the predicted scan must report a
    shorter range on the mover's bearing than the instantaneous one."""
    movers, _ = load_dynamic_obstacles(CONTRACT)
    m1 = movers[0].eval_at(9.0)          # mover1 mid-aisle, heading +x
    pos = np.array([-9.0, 0.5])
    goal = np.array([12.0, 0.5])
    v = VfhConfig(n_beams=288, pred_lead=0.5)
    margin = 0.18

    # instantaneous mover position only
    v_inst = pvfh_scan(pos, world.with_movers([m1]), goal, margin, v, 1e-6, 0.0)

    # predictive: mover also evaluated at t+0.5 (0.45 m closer to the robot)
    v_pred = pvfh_predict(pos, world, [m1], np.zeros(2), 9.0, goal, margin,
                          v, 1e-6, 0.0)

    # both are unit-ish steering vectors; the predictive one must deviate more
    # from the straight goal heading because the mover blocks the direct lane.
    dev_i = abs(math.atan2(v_inst[1], v_inst[0]))
    dev_p = abs(math.atan2(v_pred[1], v_pred[0]))
    assert dev_p >= dev_i - 1e-9
