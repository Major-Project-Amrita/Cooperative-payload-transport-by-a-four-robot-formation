"""Closed-loop self-test of the ROS 2 controller node without a simulator:
an ideal unicycle integrates the node's cmd_vel and feeds the poses back.

    source /opt/ros/humble/setup.bash && python3 tools/ros_node_selftest.py
"""
import os, sys
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, PKG)
import rclpy
from ros2_swarm_transport.formation_controller import FormationController

rclpy.init(args=["--ros-args", "-p", f"world_sdf:={PKG}/worlds/swarm_world_dynamic.sdf",
                 "-p", f"contract:={PKG}/worlds/dynamic_obstacles.json", "-p", "kin:=false",
                 "-p", "lockstep:=false", "-p", "payload:=virtual", "-p", "algo:=pvfh_prob",
                 "-p", "hold_err:=0.0"])
n = FormationController()
pos = n.F["starts"].copy()
d = n.F["assigned_slots"] - pos
yaw = np.arctan2(d[:, 1], d[:, 0])
cmds = {}
n.cmd = lambda name, v, w: cmds.__setitem__(name, (v, w))
for k in range(4000):
    for i, r in enumerate(n.robots):
        n.pos[r], n.yaw[r] = pos[i].copy(), float(yaw[i])
    n.step()
    for i, r in enumerate(n.robots):
        v, w = cmds[r]
        yaw[i] += w * n.dt
        pos[i] += v * n.dt * np.array([np.cos(yaw[i]), np.sin(yaw[i])])
    if n.phase == 3:
        break
err = float(np.linalg.norm(pos - n.leader.slots(), axis=1).max())
print(f"phase {n.phase} t={n.t:.1f} centroid {np.round(pos.mean(axis=0), 2)} slot err max {err:.2f}")
n.destroy_node()
rclpy.shutdown()
sys.exit(0 if n.phase == 3 and err < 0.3 else 1)
