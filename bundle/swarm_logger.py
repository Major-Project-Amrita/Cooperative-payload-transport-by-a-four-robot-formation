#!/usr/bin/env python3
"""
swarm_logger.py - log a ROS 2 swarm run in the SAME CSV schema Simulation7_sdf.m
writes, so compare_runs.m can read both without any conversion step.

Schema (one header row, then one row per sample):
    t, phase, leader_x, leader_y, leader_yaw, dist_to_goal, clearance,
    r1_x .. rK_x, r1_y .. rK_y

Design notes
------------
* "leader" is the formation CENTROID, matching what the MATLAB script logs.
  Do not log a designated lead robot's pose here or the trajectories will be
  offset by half the formation width and every deviation metric will be wrong.
* leader_yaw is a CIRCULAR mean. Averaging raw yaw values breaks whenever the
  formation crosses +/-pi, which in this warehouse happens on the return leg.
* clearance is written as NaN on purpose. Let compare_runs.m recompute it from
  the SDF for both runs, so the two stacks are scored by identical geometry
  rather than by whatever a costmap happened to report.
* Samples are only written once every robot has been heard from at least once,
  so the log never starts with half-initialised poses.

Usage
-----
    ros2 run <your_pkg> swarm_logger.py --ros-args \
        -p robots:="[robot1,robot2,robot3,robot4]" \
        -p goal:="[12.0, 0.0]" \
        -p output:=ros2_simple.csv \
        -p rate:=10.0

Phase: publish std_msgs/Int32 on /swarm/phase (1 = forming, 2 = transport) if
your controller knows. Otherwise everything is logged as phase 2 and you should
compare with compare_runs(..., 'Phase', []) or set phase_default.
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


def yaw_from_quaternion(q):
    """Extract yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SwarmLogger(Node):
    def __init__(self):
        super().__init__("swarm_logger")

        self.declare_parameter("robots", ["robot1", "robot2", "robot3", "robot4"])
        self.declare_parameter("odom_topic", "odom")   # per-robot: /<robot>/<odom_topic>
        self.declare_parameter("goal", [12.0, 0.0])
        self.declare_parameter("output", "ros2_run.csv")
        self.declare_parameter("rate", 10.0)           # match 1/cfg.dt in MATLAB
        self.declare_parameter("phase_default", 2)
        self.declare_parameter("stop_within", 0.0)     # >0: auto-stop near goal

        self.robots = list(self.get_parameter("robots").value)
        self.goal = list(self.get_parameter("goal").value)
        self.output = self.get_parameter("output").value
        self.rate = float(self.get_parameter("rate").value)
        self.phase = int(self.get_parameter("phase_default").value)
        self.stop_within = float(self.get_parameter("stop_within").value)

        self.K = len(self.robots)
        if self.K == 0:
            raise RuntimeError("No robots given; set the 'robots' parameter.")

        self.pose = {r: None for r in self.robots}
        self.t0 = None
        self.rows = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        odom_topic = self.get_parameter("odom_topic").value
        self.subs = []
        for r in self.robots:
            topic = f"/{r}/{odom_topic}"
            self.subs.append(
                self.create_subscription(
                    Odometry, topic, self._make_cb(r), qos
                )
            )
            self.get_logger().info(f"subscribing to {topic}")

        self.create_subscription(Int32, "/swarm/phase", self._phase_cb, 10)

        # Open the file and write the header immediately, so a crashed run
        # still leaves a readable partial log.
        self.fh = open(self.output, "w", buffering=1)
        hdr = ["t", "phase", "leader_x", "leader_y", "leader_yaw",
               "dist_to_goal", "clearance"]
        hdr += [f"r{i+1}_x" for i in range(self.K)]
        hdr += [f"r{i+1}_y" for i in range(self.K)]
        self.fh.write(",".join(hdr) + "\n")

        self.timer = self.create_timer(1.0 / self.rate, self._sample)
        self.get_logger().info(
            f"logging {self.K} robots at {self.rate} Hz -> {os.path.abspath(self.output)}"
        )

    def _make_cb(self, name):
        def cb(msg):
            p = msg.pose.pose
            self.pose[name] = (p.position.x, p.position.y,
                               yaw_from_quaternion(p.orientation))
        return cb

    def _phase_cb(self, msg):
        self.phase = int(msg.data)

    def _sample(self):
        if any(self.pose[r] is None for r in self.robots):
            return  # wait until every robot has reported at least once

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        xs = [self.pose[r][0] for r in self.robots]
        ys = [self.pose[r][1] for r in self.robots]
        yaws = [self.pose[r][2] for r in self.robots]

        cx = sum(xs) / self.K
        cy = sum(ys) / self.K
        # Circular mean: plain averaging is discontinuous across +/-pi.
        cyaw = math.atan2(
            sum(math.sin(a) for a in yaws) / self.K,
            sum(math.cos(a) for a in yaws) / self.K,
        )

        dist = math.hypot(self.goal[0] - cx, self.goal[1] - cy)

        vals = [t, float(self.phase), cx, cy, cyaw, dist, float("nan")] + xs + ys
        self.fh.write(",".join(f"{v:.6f}" for v in vals) + "\n")
        self.rows += 1

        if self.stop_within > 0.0 and dist < self.stop_within:
            self.get_logger().info(
                f"goal reached (dist {dist:.2f} m); wrote {self.rows} rows"
            )
            self.timer.cancel()
            self.fh.close()
            rclpy.shutdown()

    def destroy_node(self):
        try:
            if not self.fh.closed:
                self.fh.close()
                self.get_logger().info(f"closed log with {self.rows} rows")
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = SwarmLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
