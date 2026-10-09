#!/usr/bin/env python3
"""formation_controller - the mission controller as a ROS 2 node.

A thin wrapper around transport.MissionController, the same object the
headless tools and the direct MuJoCo run use: forming from a scattered
start, docking, load pick-up, rigid transport with the safety gate, escapes,
anti-deadlock layer, motion limits and formation feedback.

Two ways to feed it:

  lockstep (default with the MuJoCo plant node)
      subscribes  /swarm/state    std_msgs/Float64MultiArray  one message per
                                  control tick from mujoco_plant
      publishes   /swarm/command  std_msgs/Float64MultiArray  the reply the
                                  plant waits for before it advances physics
      The controller and the physics advance tick by tick, so a run is
      reproducible and identical to the in-process run.

  free-running (lockstep:=false; real robots, or any simulator with odometry)
      subscribes  /<robot>/odom   nav_msgs/Odometry
      steps on a wall-clock timer of period dt.

In both modes it publishes /<robot>/cmd_vel (geometry_msgs/Twist) and
/swarm/phase (std_msgs/Int32: 1 forming/docking, 2 transport, 3 done).
"""

import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Int32

from .ros_common import declare_mission_params, mission_setup, unpack_state


class FormationController(Node):

    def __init__(self):
        super().__init__('formation_controller')
        declare_mission_params(self)
        self.declare_parameter('lockstep', True)
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4'])
        self.declare_parameter('output_dir', '')
        self.declare_parameter('run_name', '')
        p = lambda n: self.get_parameter(n).value
        self.S = mission_setup(self)
        self.ctrl = self.S['make_controller']()
        self.F, self.leader = self.S['F'], self.ctrl.leader
        self.dt = self.S['P']['dt']
        self.robots = list(p('robots'))[:self.ctrl.K]
        self.t = 0.0
        self.log = None
        self.t_wall = time.perf_counter()
        self.pos = {n: None for n in self.robots}
        self.yaw = {n: 0.0 for n in self.robots}
        self.cmd_pub = {n: self.create_publisher(Twist, f'/{n}/cmd_vel', 10)
                        for n in self.robots}
        self.pub_phase = self.create_publisher(Int32, '/swarm/phase', 10)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        if bool(p('lockstep')):
            from .mujoco_validation import MissionLog
            self.log = MissionLog(self.S)
            self.pub_cmd = self.create_publisher(Float64MultiArray, '/swarm/command', rel)
            self.create_subscription(Float64MultiArray, '/swarm/state', self._on_state, rel)
            self.prev_comp = 0.0
            self.get_logger().info('formation controller up (lockstep, waiting for /swarm/state)')
        else:
            be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
            for n in self.robots:
                self.create_subscription(Odometry, f'/{n}/odom', self._odom_cb(n), be)
            self.timer = self.create_timer(self.dt, self.step)
            self.get_logger().info('formation controller up (free-running on odometry)')

    @property
    def phase(self):
        return 3 if self.ctrl.mode in ('finishing', 'done') else self.ctrl.phase

    # ---- shared ----
    def cmd(self, name, v, w):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.cmd_pub[name].publish(tw)

    def _publish(self, out):
        for n, (v, w) in zip(self.robots, out['cmds']):
            self.cmd(n, v, w)
        m = Int32()
        m.data = self.phase
        self.pub_phase.publish(m)

    # ---- free-running ----
    def _odom_cb(self, name):
        def cb(msg):
            q = msg.pose.pose.orientation
            self.pos[name] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.yaw[name] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return cb

    def step(self):
        if any(self.pos[n] is None for n in self.robots):
            return                                   # wait for all odometry
        pos = np.array([self.pos[n] for n in self.robots])
        yaw = np.array([self.yaw[n] for n in self.robots])
        was = self.ctrl.mode
        out = self.ctrl.step(self.t, pos, yaw, None)
        self.t += self.dt
        self._publish(out)
        if self.ctrl.mode == 'finishing' and was != 'finishing':
            self.get_logger().info('mission complete' if self.ctrl.reached_all
                                   else 'mission timeout')

    # ---- lockstep ----
    def _on_state(self, msg):
        st = unpack_state(msg.data)
        if st['tick'] > 0:                           # result of the previous command
            self.log.add(st['t'], self.ctrl, st['proxy'], st['hs'], st['hm'], self.prev_comp)
        if self.ctrl.mode == 'finishing':
            self._finish(st)
            return
        was = self.ctrl.phase
        out = self.ctrl.step(st['t'], st['pos'], st['yaw'], st['plate'])
        if was == 1 and self.ctrl.phase == 2:
            self.get_logger().info(f'formed and docked at t={st["t"]:.1f} s -> transport')
        self.prev_comp = out['comp_ms']
        m = Float64MultiArray()
        m.data = [float(st['tick']), float(out['attach']), 0.0, float(self.ctrl.phase)] + \
            [float(x) for c in out['cmds'] for x in c]
        self.pub_cmd.publish(m)
        self._publish(out)

    def _finish(self, st):
        m = Float64MultiArray()
        m.data = [float(st['tick']), 0.0, 1.0, 3.0] + [0.0] * (2 * self.ctrl.K)
        self.pub_cmd.publish(m)
        m2 = Int32()
        m2.data = 3
        self.pub_phase.publish(m2)
        p = lambda n: self.get_parameter(n).value
        out_dir = str(p('output_dir')) or os.path.join(os.getcwd(), 'results', 'mujoco_ros')
        S = self.S
        name = str(p('run_name')) or f"ros_{S['algo']}_{S['formation']}_{S['payload']}_s{S['seed']}"
        self.log.finish(self.ctrl, st['proxy'], name, out_dir,
                        time.perf_counter() - self.t_wall)
        self.get_logger().info(f'mission {"complete" if self.ctrl.reached_all else "timeout"}; '
                               f'wrote {out_dir}/{name}.csv/.json')
        self.ctrl.mode = 'done'
        raise SystemExit


def main():
    rclpy.init()
    node = FormationController()
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
