#!/usr/bin/env python3
"""mover_node.py - drive the dynamic obstacles from the shared JSON contract.

Loads dynamic_obstacles.json (the SAME file the MATLAB simulation reads),
publishes each mover's pose on /dynamic_obstacles (Marker array) and
teleports the matching Gazebo models every tick via the gz set_pose service.

eval_mover_pose lives in geometry.py - keep it in sync with eval_mover_pose.m
(the single source of truth for the motion model).
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import eval_mover_pose, eval_mover_vel, load_dynamic_obstacles


class MoverNode(Node):

    def __init__(self):
        super().__init__('mover_node')

        self.declare_parameter('contract', 'dynamic_obstacles.json')
        self.declare_parameter('rate', 10.0)          # match cfg.dt = 0.1
        self.declare_parameter('gz_world', '')
        self.declare_parameter('use_gz', False)

        contract = self.get_parameter('contract').value
        rate = float(self.get_parameter('rate').value)
        self.gz_world = self.get_parameter('gz_world').value
        self.use_gz = bool(self.get_parameter('use_gz').value)

        self.movers, meta = load_dynamic_obstacles(contract)
        if meta['frame'] != 'world':
            raise RuntimeError(f"unsupported frame {meta['frame']!r}")

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(MarkerArray, '/dynamic_obstacles', 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"driving {len(self.movers)} movers from {contract} at {rate} Hz")

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        arr = MarkerArray()
        for k, m in enumerate(self.movers):
            p = eval_mover_pose(m, t)
            v = eval_mover_vel(m, t)
            m.c, m.v, m.t = p, v, t

            mk = Marker()
            mk.header.frame_id = 'world'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns = 'dynamic_obstacles'
            mk.id = k
            mk.type = Marker.CUBE if m.shape == 'box' else Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose = Pose()
            z = float(m.size[1]) / 2 if m.shape == 'box' else float(m.r)
            mk.pose.position = Point(x=float(p[0]), y=float(p[1]), z=z)
            sx = float(m.size[0])
            sy = float(m.size[1] if m.shape == 'box' else m.r * 2)
            sz = float(m.size[1] if m.shape == 'box' else m.r * 2)
            mk.scale = Vector3(x=sx, y=sy, z=sz)
            mk.color = ColorRGBA(r=0.95, g=0.55, b=0.10, a=0.95)
            arr.markers.append(mk)

            if self.use_gz:
                self._gz_set_pose(m.name, p)

        self.pub.publish(arr)

    def _gz_set_pose(self, model, p):
        """Teleport the model in Gazebo (best effort; needs gz sim running)."""
        import subprocess
        try:
            subprocess.run(
                ['gz', 'service', '-s',
                 f'/world/{self.gz_world}/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '100',
                 '--req', f'name: "{model}", position {{x: {p[0]}, y: {p[1]}}}'],
                check=False, timeout=1.0,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:                              # pragma: no cover
            pass


def main():
    rclpy.init()
    node = MoverNode()
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
