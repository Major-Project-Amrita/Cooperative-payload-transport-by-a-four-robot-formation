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
                  /<robot>/scan   sensor_msgs/LaserScan   (only if use_scans:=true)
      steps on a timer of period dt: wall clock by default; with
      use_sim_time:=true the timer and the watchdog follow /clock, so a
      simulator slower than real time (e.g. while rendering) stays in step.

      use_scans:=true makes the robots' own range sensors the ONLY source of
      obstacle information for steering and for the safety monitor: their
      scans are fused into one map-frame point cloud (sensing.PointCloudScan,
      self-filtered for the team and the carried load) and every scan query
      of the controller is answered from it. Use it with algo:=pavfh (or
      pavfh_nonn) and gate_source:=sensor or learned; the node refuses
      combinations that need the simulator's ground truth or the traffic
      contract's future. The static map (world_sdf) is still used for
      formation planning and route waypoints.

In both modes it publishes /<robot>/cmd_vel (geometry_msgs/Twist) and
/swarm/phase (std_msgs/Int32: 1 forming/docking, 2 transport, 3 done).

Safety watchdog (free-running mode): if any robot's odometry, or with
use_scans any robot's scan, is older than watchdog_timeout seconds, every
robot is commanded to stop and /swarm/safety (std_msgs/Int32) reads 1 until
fresh data arrive (0 = nominal). Every command is clamped to |v| <= max_v,
|w| <= max_w, and zero velocity is sent to every robot on shutdown. The
watchdog complements, and does not replace, a hardware emergency stop.
"""

import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
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
        self.declare_parameter('use_scans', False)
        self.declare_parameter('scan_topic', 'scan')            # /<robot>/<scan_topic>
        self.declare_parameter('scan_mount', [0.0, 0.0, 0.0])   # sensor x, y, yaw in base frame
        self.declare_parameter('scan_max_age', 0.5)             # s; older -> stop
        self.declare_parameter('watchdog_timeout', 0.5)         # s; stale odom/scan -> stop
        self.declare_parameter('max_v', 1.0)                    # m/s, hard clamp
        self.declare_parameter('max_w', 2.0)                    # rad/s, hard clamp
        p = lambda n: self.get_parameter(n).value
        self.S = mission_setup(self)
        self.ctrl = self.S['make_controller']()
        self.F, self.leader = self.S['F'], self.ctrl.leader
        self.dt = self.S['P']['dt']
        self.robots = list(p('robots'))[:self.ctrl.K]
        self.t = 0.0
        self.log = None
        self.now = time.monotonic                    # replaceable clock (tests)
        try:                                         # simulation time from /clock
            if bool(self.get_parameter('use_sim_time').value):
                self.now = lambda: self.get_clock().now().nanoseconds * 1e-9
        except Exception:                            # noqa: BLE001  (no such parameter)
            pass
        self.wd_timeout = float(p('watchdog_timeout'))
        self.max_v, self.max_w = float(p('max_v')), float(p('max_w'))
        self.last_odom = {n: -math.inf for n in self.robots}
        self.last_scan = {n: -math.inf for n in self.robots}
        self.n_watchdog = 0
        self.safety_state = 0
        self.pub_safety = self.create_publisher(Int32, '/swarm/safety', 10)
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
            self.plate = None                        # carried load pose, if published
            self.create_subscription(PoseStamped, '/payload/pose', self._plate_cb, be)
            self.scans = None
            if bool(p('use_scans')):
                self._setup_scans(p, be)
            self.timer = self.create_timer(self.dt, self.step)
            self.get_logger().info('formation controller up (free-running on odometry)')

    @property
    def phase(self):
        return 3 if self.ctrl.mode in ('finishing', 'done') else self.ctrl.phase

    # ---- shared ----
    def cmd(self, name, v, w):
        tw = Twist()
        v = max(-self.max_v, min(self.max_v, float(v)))
        w = max(-self.max_w, min(self.max_w, float(w)))
        tw.linear.x, tw.angular.z = v, w
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
            self.last_odom[name] = self.now()
        return cb

    def _plate_cb(self, msg):
        q = msg.pose.orientation
        self.plate = (np.array([msg.pose.position.x, msg.pose.position.y]),
                      math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def stop_all(self):
        for n in self.robots:
            self.cmd(n, 0.0, 0.0)

    def _set_safety(self, state):
        if state != self.safety_state:
            self.get_logger().info('watchdog: stale sensor data -> all robots stopped'
                                   if state else 'watchdog: data fresh again -> resuming')
        self.safety_state = state
        m = Int32()
        m.data = int(state)
        self.pub_safety.publish(m)

    def _stale(self):
        now = self.now()
        if any(now - self.last_odom[n] > self.wd_timeout for n in self.robots):
            return True
        return self.scans is not None and any(
            now - self.last_scan[n] > self.wd_timeout for n in self.robots)

    def _setup_scans(self, p, qos):
        from sensor_msgs.msg import LaserScan
        from .sensing import PointCloudScan
        algo, gsrc = self.S['algo'], self.S.get('gate_source', 'truth')
        if algo not in ('pavfh', 'pavfh_nonn') or gsrc not in ('sensor', 'learned'):
            raise ValueError(
                f"use_scans needs a sensor-only configuration (algo pavfh|pavfh_nonn, "
                f"gate_source sensor|learned); got algo={algo}, gate_source={gsrc}")
        # self-filter widens with range noise: team-mate returns scatter by
        # about 3 sigma beyond the robot's radius
        sig0 = float(self.S.get('sigma0', 0.02))
        self.cloud = PointCloudScan(robot_radius=self.S['P']['robot_radius'],
                                    self_margin=0.10 + 3.0 * sig0,
                                    max_age=float(p('scan_max_age')))
        self.mount = tuple(float(x) for x in p('scan_mount'))
        self.scans = {n: None for n in self.robots}
        topic = str(p('scan_topic'))
        for n in self.robots:
            self.create_subscription(LaserScan, f'/{n}/{topic}', self._scan_cb(n), qos)
        cfg = self.ctrl.leader.cfg
        cfg.scan_provider = self.cloud
        self.get_logger().info(f'real-sensor mode: obstacles only from /<robot>/{topic}')

    def _scan_cb(self, name):
        def cb(msg):
            if self.pos[name] is None:
                return
            from .sensing import laserscan_to_points
            pose = (self.pos[name][0], self.pos[name][1], self.yaw[name])
            self.scans[name] = laserscan_to_points(
                pose, msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, self.mount)
            self.last_scan[name] = self.now()
        return cb

    def _update_cloud(self, pos):
        if any(v is None for v in self.scans.values()):
            return False
        poly = None
        if self.ctrl.phase == 2:                     # carrying: filter the load too
            c = pos.mean(axis=0)
            poly = pos[np.argsort(np.arctan2(pos[:, 1] - c[1], pos[:, 0] - c[0]))]
            if len(poly) < 3:
                poly = None
        self.cloud.update(list(self.scans.values()), pos, self.t, poly)
        return True

    def step(self):
        if any(self.pos[n] is None for n in self.robots):
            self.stop_all()
            return                                   # wait for all odometry
        if self._stale():                            # watchdog: hold still
            self.n_watchdog += 1
            self.stop_all()
            self._set_safety(1)
            return
        self._set_safety(0)
        pos = np.array([self.pos[n] for n in self.robots])
        yaw = np.array([self.yaw[n] for n in self.robots])
        if self.scans is not None and not self._update_cloud(pos):
            for n in self.robots:                    # no scans yet: hold still
                self.cmd(n, 0.0, 0.0)
            return
        was = self.ctrl.mode
        out = self.ctrl.step(self.t, pos, yaw, getattr(self, 'plate', None))
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
        try:
            node.stop_all()                          # never leave robots moving
        except Exception:                            # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
