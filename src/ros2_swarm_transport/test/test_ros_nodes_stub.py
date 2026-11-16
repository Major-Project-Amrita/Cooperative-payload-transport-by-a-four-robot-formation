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
if not os.path.exists(os.path.join(TOOLS, "ros_lockstep_stub.py")):
    pytest.skip("tools/ not present", allow_module_level=True)
try:
    import rclpy  # noqa: F401
    pytest.skip("real rclpy installed: use tools/ros_lockstep_test.sh", allow_module_level=True)
except ImportError:
    pass
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
