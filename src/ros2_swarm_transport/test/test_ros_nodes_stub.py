"""The real ROS 2 node classes on an in-process stand-in for rclpy (no ROS 2
needed): lockstep plant <-> controller equals the in-process run, and the
controller's real-sensor mode completes a mission from LaserScans alone.
On a machine with ROS 2, tools/ros_lockstep_test.sh runs the same check over
DDS."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "tools")
try:
    import rclpy  # noqa: F401
    HAVE_RCLPY = True
except ImportError:
    HAVE_RCLPY = False
USABLE = os.path.exists(os.path.join(TOOLS, "ros_lockstep_stub.py")) and not HAVE_RCLPY
# with real rclpy, tools/ros_humble_full.sh runs these checks over DDS instead
pytestmark = pytest.mark.skipif(not USABLE, reason="real rclpy installed or tools/ absent")
if USABLE:
    sys.path.insert(0, TOOLS)
    import ros_lockstep_stub as stub  # noqa: E402
    stub.install_stub()


@pytest.mark.skipif(not os.path.exists(os.path.join(os.path.dirname(HERE), "models",
                                                    "pavfh_occupancy.npz")),
                    reason="PA-VFH weights absent")
def test_lockstep_nodes_equal_inprocess_pavfh(tmp_path):
    name, rows, cnt = stub.lockstep("pavfh", "virtual", 2, True, "sensor", str(tmp_path))
    assert cnt["/swarm/state"] == cnt["/swarm/command"] > 100
    assert all(same for _, _, _, same in rows), rows


def test_real_sensor_mode_refuses_ground_truth_configs():
    from ros2_swarm_transport.formation_controller import FormationController
    stub.PARAMS.clear()
    stub.PARAMS.update(lockstep=False, use_scans=True, algo="mpc", gate_source="truth",
                       payload="virtual")
    with pytest.raises(ValueError):
        FormationController()


def test_watchdog_stops_on_stale_data_and_clamps():
    from ros2_swarm_transport.formation_controller import FormationController
    stub.BUS.clear(), stub.QUEUE.clear(), stub.PARAMS.clear()
    stub.PARAMS.update(lockstep=False, algo="pavfh", payload="virtual", kin=False,
                       hold_err=0.0, max_v=0.3)
    n = FormationController()
    clock = [100.0]
    n.now = lambda: clock[0]
    cmds = {}
    n.cmd_pub = {r: type("P", (), {"publish": lambda self, m, r=r: cmds.__setitem__(
        r, (m.linear.x, m.angular.z))})() for r in n.robots}
    pos = n.F["starts"]
    for i, r in enumerate(n.robots):
        n.pos[r], n.yaw[r] = pos[i].copy(), 0.0
        n.last_odom[r] = clock[0]
    n.step()
    assert n.safety_state == 0
    assert all(abs(v) <= 0.3 + 1e-12 for v, _ in cmds.values())
    t_before = n.t
    clock[0] += 1.0                                  # odometry now 1 s old
    n.step()
    assert n.safety_state == 1 and n.n_watchdog == 1
    assert all(v == 0.0 and w == 0.0 for v, w in cmds.values())
    assert n.t == t_before                           # mission clock did not advance
    for r in n.robots:
        n.last_odom[r] = clock[0]
    n.step()
    assert n.safety_state == 0
