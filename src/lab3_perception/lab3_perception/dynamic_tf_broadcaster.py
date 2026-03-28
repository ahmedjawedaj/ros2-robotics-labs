#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
import tf2_ros


class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

        # Circle params
        self.declare_parameter('radius', 2.0)   # meters (task requires 2m)
        self.declare_parameter('omega', 0.5)    # rad/s (tune if you want)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_frame', 'virtual_robot')

        self.radius = float(self.get_parameter('radius').value)
        self.omega = float(self.get_parameter('omega').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        self.br = tf2_ros.TransformBroadcaster(self)
        self.t0 = self.get_clock().now()

        # 20 Hz
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(
            f"Broadcasting dynamic TF: {self.world_frame} -> {self.robot_frame} | "
            f"R={self.radius} m, omega={self.omega} rad/s"
        )

    def timer_cb(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9  # seconds

        # Circle in XY plane: x=R cos(wt), y=R sin(wt)
        x = self.radius * math.cos(self.omega * t)
        y = self.radius * math.sin(self.omega * t)
        z = 0.0

        # Optional: face tangent to motion (yaw = wt + pi/2)
        yaw = self.omega * t + math.pi / 2.0

        # Yaw-only quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.robot_frame

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.br.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
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
