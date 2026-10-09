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
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from .mujoco_validation import DT_CTRL
from .ros_common import declare_mission_params, mission_setup, pack_state


class MujocoPlantNode(Node):

    def __init__(self):
        super().__init__('mujoco_plant')
        declare_mission_params(self)
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4'])
        self.declare_parameter('video', '')
        self.declare_parameter('gui', False)         # interactive MuJoCo window
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
        self.create_subscription(Float64MultiArray, '/swarm/command', self._on_cmd, rel)
        self.kick = self.create_timer(0.5, self._kick)      # until the controller answers
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
