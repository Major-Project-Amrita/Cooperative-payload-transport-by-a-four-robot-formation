#!/usr/bin/env python3
"""A team of simulated robots as a REAL ROS 2 node, for testing the
deployment interface of the formation controller (launch/robots.launch.py)
over DDS without hardware.

Per robot <r>: subscribes /<r>/cmd_vel, publishes /<r>/odom (map frame) and
/<r>/scan (sensor_msgs/LaserScan, 360 beams, 8 m, Gaussian range noise
`sigma`) ray-cast from that robot's own pose against the static world, the
movers where they are now and its team-mates. Unicycle kinematics at 10 Hz
on a wall-clock timer. It scores contacts against the ground truth (which
the controller never sees) and, when /swarm/phase reads 3 or `timeout_s`
passes, writes <out>.json and exits.

    python3 tools/ros_fake_robots.py --ros-args -p seed:=1 -p sigma:=0.02 \\
        -p traffic_phase:=true -p out:=results/ros_humble/real_sensor_s1
    (with: ros2 launch ros2_swarm_transport robots.launch.py ... in parallel)

Option `drop_after` (s): stop publishing scans of robot1 at that time, to
test the controller's watchdog (it must stop every robot).
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "ros2_swarm_transport"))
from ros2_swarm_transport.ros_common import declare_mission_params, mission_setup  # noqa: E402
from ros2_swarm_transport.geometry import lidar_scan, movers_distance  # noqa: E402
from ros2_swarm_transport.transport import eval_movers  # noqa: E402
from ros2_swarm_transport.vfh import _movers_only_scan  # noqa: E402


class FakeRobots(Node):
    def __init__(self):
        super().__init__('fake_robots')
        declare_mission_params(self)
        for n, d in (('sigma', 0.02), ('out', ''), ('timeout_s', 400.0), ('drop_after', -1.0),
                     ('robots', ['robot1', 'robot2', 'robot3', 'robot4'])):
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.S = mission_setup(self)
        F = self.S['F']
        self.world, self.movers = self.S['world'], self.S['movers']
        self.r = self.S['P']['robot_radius']
        self.dt = self.S['P']['dt']
        self.robots = list(p('robots'))[:len(F['starts'])]
        self.pos = F['starts'].copy()
        d = F['assigned_slots'] - self.pos
        self.yaw = np.arctan2(d[:, 1], d[:, 0])
        self.v = np.zeros(len(self.robots))
        self.w = np.zeros(len(self.robots))
        self.sigma, self.out = float(p('sigma')), str(p('out'))
        self.timeout, self.drop_after = float(p('timeout_s')), float(p('drop_after'))
        self.rng = np.random.default_rng([int(p('seed')), 4242])
        self.t, self.phase, self.done = 0.0, 0, False
        self.contact = self.contact_s = 0
        self.min_gap = math.inf
        self.stop_seen, self.t_drop_stop = False, None
        self.pub_odom = {n: self.create_publisher(Odometry, f'/{n}/odom', 10) for n in self.robots}
        self.pub_scan = {n: self.create_publisher(LaserScan, f'/{n}/scan', 10) for n in self.robots}
        for i, n in enumerate(self.robots):
            self.create_subscription(Twist, f'/{n}/cmd_vel', self._cmd(i), 10)
        self.create_subscription(Int32, '/swarm/phase', self._phase, 10)
        self.safety = 0
        self.create_subscription(Int32, '/swarm/safety', self._safety, 10)
        self.t0 = time.monotonic()
        self.cmd_log = []             # (wall s, sim s, v, w) of robot1's cmd_vel
        self.sig_log = []             # (sim s, min gap, safety flag) per tick
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'{len(self.robots)} fake robots, sigma={self.sigma}')

    def _cmd(self, i):
        def cb(m):
            self.v[i], self.w[i] = m.linear.x, m.angular.z
            if i == 0:
                self.cmd_log.append((time.monotonic() - self.t0, self.t,
                                     float(m.linear.x), float(m.angular.z)))
        return cb

    def _phase(self, m):
        self.phase = int(m.data)

    def _safety(self, m):
        self.safety = int(m.data)

    def _scan(self, i, mk):
        n = 360
        rr, ang = lidar_scan(self.pos[i], self.world, n, 8.0)
        if mk:
            rr = np.minimum(rr, _movers_only_scan(self.pos[i], mk, n, 8.0, ang))
        for j in range(len(self.pos)):
            if j == i:
                continue
            dd = self.pos[j] - self.pos[i]
            b = dd @ np.vstack([np.cos(ang), np.sin(ang)])
            disc = b * b - (dd @ dd - self.r ** 2)
            hit = (disc >= 0) & (b > 0)
            rr = np.minimum(rr, np.where(hit, b - np.sqrt(np.maximum(disc, 0)), np.inf))
        rr = rr + self.rng.normal(0, self.sigma, rr.shape)
        rr = np.where(rr >= 8.0 - 1e-6, np.inf, rr)
        return rr, ang

    def tick(self):
        if self.done:
            return
        # integrate the last commands
        for i in range(len(self.robots)):
            self.yaw[i] += self.w[i] * self.dt
            self.pos[i] += self.v[i] * self.dt * np.array([math.cos(self.yaw[i]),
                                                           math.sin(self.yaw[i])])
        self.t += self.dt
        mk = eval_movers(self.movers, self.t)
        d_s = float(self.world.distance(self.pos).min())
        dd = min(d_s, float(movers_distance(self.pos, mk).min())) if mk else d_s
        self.min_gap = min(self.min_gap, dd - self.r)
        self.contact += int(dd <= self.r)
        self.contact_s += int(d_s <= self.r)
        self.sig_log.append((round(self.t, 2), round(dd - self.r, 4), self.safety))
        stamp = self.get_clock().now().to_msg()
        for i, n in enumerate(self.robots):
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'map', f'{n}/base_link'
            o.pose.pose.position.x, o.pose.pose.position.y = float(self.pos[i][0]), float(self.pos[i][1])
            o.pose.pose.orientation.z = math.sin(self.yaw[i] / 2)
            o.pose.pose.orientation.w = math.cos(self.yaw[i] / 2)
            self.pub_odom[n].publish(o)
            if self.drop_after >= 0 and self.t >= self.drop_after and i == 0:
                continue                              # robot1's lidar has failed
            rr, ang = self._scan(i, mk)
            s = LaserScan()
            s.header.stamp, s.header.frame_id = stamp, f'{n}/laser'
            s.angle_min = float(ang[0] - self.yaw[i])
            s.angle_increment = float(ang[1] - ang[0])
            s.angle_max = float(s.angle_min + s.angle_increment * (len(rr) - 1))
            s.range_min, s.range_max = 0.05, 8.0
            s.ranges = [float(x) for x in rr]
            self.pub_scan[n].publish(s)
        if self.drop_after >= 0 and self.t >= self.drop_after + 1.0:
            moving = float(np.abs(self.v).max())
            if self.safety == 1 and moving == 0.0 and self.t_drop_stop is None:
                self.t_drop_stop = self.t
            if self.t >= self.drop_after + 3.0:
                self.finish(watchdog=dict(scan_dropped_at=self.drop_after,
                                          all_stopped_at=self.t_drop_stop,
                                          safety_flag=self.safety,
                                          max_cmd_speed_after=moving))
                return
        if self.phase == 3 or self.t >= self.timeout:
            self.finish()

    def finish(self, **extra):
        self.done = True
        res = dict(sigma=self.sigma, seed=int(self.get_parameter('seed').value),
                   traffic_phase=bool(self.get_parameter('traffic_phase').value),
                   controller="robots.launch.py defaults: algo pavfh, gate_source sensor",
                   reached=self.phase == 3, sim_time=round(self.t, 1),
                   wall_s=round(time.monotonic() - self.t0, 1),
                   contact_steps=self.contact, contact_steps_static=self.contact_s,
                   min_gap=round(self.min_gap, 3), **extra)
        self.get_logger().info(json.dumps(res))
        if self.out:
            os.makedirs(os.path.dirname(self.out) or '.', exist_ok=True)
            json.dump(res, open(self.out + '.json', 'w'), indent=1)
            json.dump(dict(cmd=self.cmd_log, sig=self.sig_log),
                      open(self.out + '_signals.json', 'w'))
        raise SystemExit


def main():
    rclpy.init()
    node = FakeRobots()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
