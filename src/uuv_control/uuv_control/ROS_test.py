#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose


class UUVControl(Node):

    def __init__(self):
        super().__init__('uuv_control_node')
        self.pub = self.create_publisher(
            Twist,
            '/uuv/cmd_vel',
            10
        )
        self.pose_pub = self.create_publisher(
            Pose,
            '/uuv/pose',
            10
        )
        self.pose_sub = self.create_subscription(
            Pose,
            '/uuv/pose',
            self.pose_callback,
            10
        )
        self.timer = self.create_timer(0.5, self.publish_cmd)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info("UUV Control Node Started")

    def pose_callback(self, msg):
        self.get_logger().info(
            f"Pose received: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}"
        )

    def publish_cmd(self):
        cmd = Twist()

        cmd.linear.x = 1.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.2
        self.pub.publish(cmd)

        dt = 0.5

        self.theta += cmd.angular.z * dt
        self.x += cmd.linear.x * dt
        pose = Pose()

        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        self.pose_pub.publish(pose)

        self.get_logger().info("Publishing velocity and updated pose")


def main(args=None):
    rclpy.init(args=args)

    node = UUVControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()