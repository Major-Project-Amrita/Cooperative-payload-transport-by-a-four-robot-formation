#!/usr/bin/env python3
"""swarm_logger.py - log a ROS 2 swarm run in the SAME CSV schema the MATLAB
scripts write, so compare_runs.m reads both without conversion.

Same contract as the standalone script in the MATLAB repo, plus the optional
dynamic-obstacle columns (mover_vx, mover_vy, speed_scale, mover_gap,
mover_gap_pred) which are written as 99 sentinels unless a
/dynamic_obstacles topic is alive.
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SwarmLogger(Node):

    def __init__(self):
        super().__init__('swarm_logger')

        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4'])
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('goal', [12.0, 0.0])
        self.declare_parameter('output', 'ros2_run.csv')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('phase_default', 2)
        self.declare_parameter('stop_within', 0.0)

        self.robots = list(self.get_parameter('robots').value)
        self.goal = list(self.get_parameter('goal').value)
        self.output = self.get_parameter('output').value
        self.rate = float(self.get_parameter('rate').value)
        self.phase = int(self.get_parameter('phase_default').value)
        self.stop_within = float(self.get_parameter('stop_within').value)

        self.K = len(self.robots)
        if self.K == 0:
            raise RuntimeError("No robots given; set the 'robots' parameter.")

        self.pose = {r: None for r in self.robots}
        self.t0 = None
        self.rows = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        odom_topic = self.get_parameter('odom_topic').value
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/{odom_topic}',
                                     self._make_cb(r), qos)
        self.create_subscription(Int32, '/swarm/phase', self._phase_cb, 10)

        self.fh = open(self.output, 'w', buffering=1)
        hdr = ['t', 'phase', 'leader_x', 'leader_y', 'leader_yaw',
               'dist_to_goal', 'clearance']
        hdr += [f'r{i+1}_x' for i in range(self.K)]
        hdr += [f'r{i+1}_y' for i in range(self.K)]
        # dynamic columns (sentinels; MATLAB writes real values)
        hdr += ['mover_vx', 'mover_vy', 'speed_scale', 'mover_gap',
                'mover_gap_pred']
        self.fh.write(','.join(hdr) + '\n')

        self.timer = self.create_timer(1.0 / self.rate, self._sample)
        self.get_logger().info(
            f'logging {self.K} robots at {self.rate} Hz -> '
            f'{os.path.abspath(self.output)}')

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
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        xs = [self.pose[r][0] for r in self.robots]
        ys = [self.pose[r][1] for r in self.robots]
        yaws = [self.pose[r][2] for r in self.robots]

        cx, cy = sum(xs) / self.K, sum(ys) / self.K
        cyaw = math.atan2(sum(math.sin(a) for a in yaws) / self.K,
                          sum(math.cos(a) for a in yaws) / self.K)
        dist = math.hypot(self.goal[0] - cx, self.goal[1] - cy)

        vals = ([t, float(self.phase), cx, cy, cyaw, dist, 99.0] + xs + ys +
                [99.0] * 5)
        self.fh.write(','.join(f'{v:.6f}' for v in vals) + '\n')
        self.rows += 1

        if self.stop_within > 0.0 and dist < self.stop_within:
            self.get_logger().info(
                f'goal reached (dist {dist:.2f} m); wrote {self.rows} rows')
            self.timer.cancel()
            self.fh.close()
            rclpy.shutdown()

    def destroy_node(self):
        try:
            if not self.fh.closed:
                self.fh.close()
                self.get_logger().info(f'closed log with {self.rows} rows')
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


if __name__ == '__main__':
    main()
