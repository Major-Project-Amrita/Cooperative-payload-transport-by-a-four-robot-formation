#!/usr/bin/env python3
"""mujoco_plant - the MuJoCo physics world as a ROS 2 node.

Wraps mujoco_validation.MujocoPlant (the same plant the in-process run
uses). It contains no mission logic: it publishes where the robots and the
load are, and applies the wheel commands it is sent.

  publishes   /swarm/state        std_msgs/Float64MultiArray  one per control tick
              /<robot>/odom       nav_msgs/Odometry           (map frame)
              /payload/pose       geometry_msgs/PoseStamped
              /dynamic_obstacles  visualization_msgs/MarkerArray
              /clock              rosgraph_msgs/Clock          simulation time
  subscribes  /swarm/command      std_msgs/Float64MultiArray

Lockstep: after publishing the state of tick k the physics waits for the
command of tick k, then advances one control period (0.1 s, 50 MuJoCo
steps). Runs are therefore reproducible and independent of wall-clock load.

Free-running (lockstep:=false): the plant behaves like a robot simulator.
It advances one control period per wall-clock tick, applies the latest
/<robot>/cmd_vel of each robot, publishes /<robot>/odom and, with
publish_scans:=true, /<robot>/scan (sensor_msgs/LaserScan, 360 beams, 8 m,
range noise sense_sigma0) ray-cast from each PHYSICAL robot's pose against
the statics, the movers and the team-mates. The load is attached when
/swarm/phase switches to 2. When /swarm/phase reads 3 (or after timeout_s)
it stops the robots, settles 1 s, writes <output_dir>/<run_name>.json with
the physics-engine contact counts, and exits.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from .mujoco_validation import DT_CTRL
from .ros_common import declare_mission_params, mission_setup, pack_state


class _FreeMixin:
    """Free-running simulator behaviour (lockstep:=false)."""

    def _setup_free(self):
        import numpy as np
        from sensor_msgs.msg import LaserScan
        self._np = np
        self.LaserScan = LaserScan
        K = self.plant.K
        self.cmds = [(0.0, 0.0)] * K
        self.ctrl_phase = 0
        self.attached = False
        self.pins, self.tilts = [], []
        self.scans_on = bool(self.get_parameter('publish_scans').value)
        self.sigma = float(self.S['sigma0'])
        self.rng = np.random.default_rng([int(self.S['seed']), 4242])
        self.pub_scan = {n: self.create_publisher(LaserScan, f'/{n}/scan', 10)
                         for n in self.robots}
        for i, n in enumerate(self.robots):
            self.create_subscription(Twist, f'/{n}/cmd_vel', self._cmd_cb(i), 10)
        self.create_subscription(Int32, '/swarm/phase', self._phase_cb, 10)
        self.safety, self.t_flag, self.v_after = 0, None, 0.0
        self.create_subscription(Int32, '/swarm/safety', self._safety_cb, 10)
        self.drop_after = float(self.get_parameter('scan_drop_after').value)
        self.t_end = float(self.get_parameter('timeout_s').value)
        self.create_timer(DT_CTRL, self._tick_free)
        self.get_logger().info('free-running plant: cmd_vel in, odom' +
                               (' + scan' if self.scans_on else '') + ' out')

    def _cmd_cb(self, i):
        def cb(m):
            c = list(self.cmds)
            c[i] = (float(m.linear.x), float(m.angular.z))
            self.cmds = c
        return cb

    def _phase_cb(self, m):
        self.ctrl_phase = int(m.data)

    def _safety_cb(self, m):
        self.safety = int(m.data)
        if self.safety and self.t_flag is None:
            self.t_flag = self.t

    def _publish_scans(self):
        from .sensing import simulate_laserscan
        from .transport import eval_movers
        pos, yaw = self.plant.state()
        mk = eval_movers(self.S['movers'], self.t)
        r = self.S['P']['robot_radius']
        sec, nsec = self._stamp()
        for i, n in enumerate(self.robots):
            if i == 0 and 0 <= self.drop_after <= self.t:
                continue                             # robot1's lidar has failed
            rr, ang = simulate_laserscan(i, pos, self.S['world'], mk, r, sigma=self.sigma,
                                         rng=self.rng)
            s = self.LaserScan()
            s.header.stamp.sec, s.header.stamp.nanosec = sec, nsec
            s.header.frame_id = f'{n}/laser'
            s.angle_min = float(ang[0] - yaw[i])
            s.angle_increment = float(ang[1] - ang[0])
            s.angle_max = float(s.angle_min + s.angle_increment * (len(rr) - 1))
            s.range_min, s.range_max = 0.05, 8.0
            s.ranges = [float(x) for x in rr]
            self.pub_scan[n].publish(s)

    def _tick_free(self):
        if self.finished:
            return
        if self.ctrl_phase >= 2 and not self.attached:
            if self.S['payload'] != 'virtual':
                self.plant.attach()
            self.attached = True
        if self.t_flag is not None and self.t > self.t_flag + DT_CTRL + 1e-9:   # commands after the flag
            self.v_after = max(self.v_after, max(abs(v) for v, _ in self.cmds))
        self.plant.set_cmds(self.cmds)
        self.hs, self.hm = self.plant.advance(self.t, self.ctrl_phase >= 2)
        self.t += DT_CTRL
        self.tick += 1
        self.plant.frame()
        if self.attached:
            self.pins.append(float(self.plant.pin_gap()))
            self.tilts.append(float(self.plant.tilt_deg()))
        self._publish()
        if self.scans_on:
            self._publish_scans()
        if self.ctrl_phase == 3 or self.t >= self.t_end or (
                self.drop_after >= 0 and self.t >= self.drop_after + 5.0):
            self._finish_free()

    def _finish_free(self):
        import json
        import os
        np = self._np
        self.finished = True
        self.plant.set_cmds([(0.0, 0.0)] * self.plant.K)
        for _ in range(10):
            self.plant.advance(self.t, False)
            self.t += DT_CTRL
        pos, _ = self.plant.state()
        goal = np.asarray(self.S['waypoints'][-1], float)
        err = float(np.linalg.norm(pos.mean(axis=0) - goal))
        res = dict(name=str(self.get_parameter('run_name').value), mode='free-running',
                   algo=self.S['algo'], payload=self.S['payload'], seed=int(self.S['seed']),
                   sigma0=self.sigma, scans=self.scans_on, phase=bool(self.S.get('phase')),
                   gate_margin=float(self.S.get('gate_margin', 0.0)),
                   reached=bool(self.ctrl_phase == 3 and err <= 1.0),
                   controller_finished=bool(self.ctrl_phase == 3), final_centroid_err=err,
                   sim_time=round(self.t, 1),
                   contact_steps_static=int(self.plant.contact['static']),
                   contact_steps_mover=int(self.plant.contact['mover']),
                   contact_force_max=float(self.plant.contact['fmax']),
                   pin_gap_max=float(max(self.pins, default=0.0)),
                   payload_tilt_max_deg=float(max(self.tilts, default=0.0)))
        if self.drop_after >= 0:
            res['watchdog'] = dict(scan_dropped_at=self.drop_after, safety_flag_at=self.t_flag,
                                   max_cmd_speed_after_flag=self.v_after)
        out = str(self.get_parameter('output_dir').value)
        if out:
            os.makedirs(out, exist_ok=True)
            json.dump(res, open(os.path.join(out, res['name'] + '.json'), 'w'), indent=1)
        self.get_logger().info(json.dumps(res))
        self.plant.close()
        raise SystemExit



class MujocoPlantNode(Node, _FreeMixin):

    def __init__(self):
        super().__init__('mujoco_plant')
        declare_mission_params(self)
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4'])
        self.declare_parameter('video', '')
        self.declare_parameter('gui', False)         # interactive MuJoCo window
        self.declare_parameter('lockstep', True)
        self.declare_parameter('publish_scans', False)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('run_name', '')
        self.declare_parameter('timeout_s', 400.0)
        self.declare_parameter('scan_drop_after', -1.0)   # s; robot1's lidar stops (watchdog test)
        vid = str(self.get_parameter('video').value) or None
        self.gui = bool(self.get_parameter('gui').value)
        self.S = mission_setup(self, video=vid, gui=self.gui)
        self.plant = self.S['make_plant']()
        self.robots = list(self.get_parameter('robots').value)[:self.plant.K]
        self.tick, self.t = 0, 0.0
        self.hs = self.hm = False
        self.started = self.finished = False
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_state = self.create_publisher(Float64MultiArray, '/swarm/state', rel)
        self.pub_odom = {n: self.create_publisher(Odometry, f'/{n}/odom', 10) for n in self.robots}
        self.pub_load = self.create_publisher(PoseStamped, '/payload/pose', 10)
        self.pub_mov = self.create_publisher(MarkerArray, '/dynamic_obstacles', 10)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.free = not bool(self.get_parameter('lockstep').value)
        if self.free:
            self._setup_free()
        else:
            self.create_subscription(Float64MultiArray, '/swarm/command', self._on_cmd, rel)
            self.kick = self.create_timer(0.5, self._kick)  # until the controller answers
        self.get_logger().info(f'MuJoCo plant up: {self.plant.K} robots, payload '
                               f'{self.S["payload"]}, world {self.S["world_sdf"]}')

    def _stamp(self):
        s = int(self.t)
        return s, int(round((self.t - s) * 1e9))

    def _publish(self):
        pos, yaw = self.plant.state()
        plate = self.plant.plate()
        sec, nsec = self._stamp()
        c = Clock()
        c.clock.sec, c.clock.nanosec = sec, nsec
        self.pub_clock.publish(c)
        for i, n in enumerate(self.robots):
            o = Odometry()
            o.header.frame_id, o.child_frame_id = 'map', f'{n}/base_link'
            o.header.stamp.sec, o.header.stamp.nanosec = sec, nsec
            o.pose.pose.position.x, o.pose.pose.position.y = float(pos[i][0]), float(pos[i][1])
            o.pose.pose.orientation.z = math.sin(yaw[i] / 2)
            o.pose.pose.orientation.w = math.cos(yaw[i] / 2)
            self.pub_odom[n].publish(o)
        if plate is not None:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp.sec, ps.header.stamp.nanosec = sec, nsec
            ps.pose.position.x, ps.pose.position.y = float(plate[0][0]), float(plate[0][1])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(plate[1] / 2), math.cos(plate[1] / 2)
            self.pub_load.publish(ps)
        ma = MarkerArray()
        for k, m in enumerate(self.S['movers']):
            mk = Marker()
            mk.header.frame_id, mk.ns, mk.id = 'map', 'movers', k
            mk.type = Marker.CYLINDER if m.shape == 'circle' else Marker.CUBE
            c_ = m.eval_at(self.t).c
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(c_[0]), float(c_[1]), 0.4
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = float(m.size[0]), float(m.size[1]), 0.8
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.95, 0.55, 0.1, 1.0
            ma.markers.append(mk)
        self.pub_mov.publish(ma)
        msg = Float64MultiArray()
        msg.data = pack_state(self.tick, self.t, pos, yaw, plate, self.hs, self.hm,
                              self.plant.pin_gap(), self.plant.tilt_deg(), self.plant.contact)
        self.pub_state.publish(msg)

    def _kick(self):
        if self.started:
            self.kick.cancel()
            return
        if self.pub_state.get_subscription_count() > 0:
            self._publish()                           # tick 0, repeated until answered

    def _on_cmd(self, msg):
        d = msg.data
        if int(d[0]) != self.tick or self.finished:
            return                                    # stale or duplicate reply
        self.started = True
        K = self.plant.K
        if d[2] > 0:                                  # finish: stop and settle 1 s
            self.finished = True
            self.plant.set_cmds([(0.0, 0.0)] * K)
            for _ in range(10):
                self.plant.advance(self.t, False)
                self.t += DT_CTRL
                self.plant.frame()
            self.plant.close()
            self.get_logger().info(f'mission finished at t={self.t:.1f} s; plant stopped')
            raise SystemExit
        if d[1] > 0:
            self.plant.attach()
        self.plant.set_cmds([(d[4 + 2 * i], d[5 + 2 * i]) for i in range(K)])
        self.hs, self.hm = self.plant.advance(self.t, d[3] >= 2)   # contacts in transport
        self.t += DT_CTRL
        self.tick += 1
        self.plant.frame()
        self._publish()




def main():
    rclpy.init()
    node = MujocoPlantNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        gui = getattr(node, 'gui', False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if gui:
            # The passive viewer's render thread can still be tearing down its
            # GL context when the interpreter finalises (seen as a segfault at
            # exit, after all results were written). Leave without finalising.
            import os
            import sys
            sys.stdout.flush()
            os._exit(0)


if __name__ == '__main__':
    main()
