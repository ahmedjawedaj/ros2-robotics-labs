#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class MoveCircle(Node):
    def __init__(self):
        super().__init__('move_circle')

        # Task requirement: circle radius fixed at 5 meters (default)
        self.declare_parameter('radius', 5.0)

        # Choose a linear velocity v (m/s). You can tune this.
        self.declare_parameter('linear_speed', 0.5)

        self.radius = float(self.get_parameter('radius').value)
        self.v = float(self.get_parameter('linear_speed').value)

        # Compute angular velocity using v = w * r  =>  w = v / r
        # Sign decides clockwise vs anticlockwise (positive = CCW)
        self.w = self.v / self.radius

        # Task requirement: publish to /cmd_vel
        self.topic = '/cmd_vel'
        self.pub = self.create_publisher(TwistStamped, self.topic, 10)

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(
            f"Publishing TwistStamped to {self.topic} | radius={self.radius} m | v={self.v} m/s | w={self.w} rad/s"
        )

    def tick(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = self.v
        cmd.twist.angular.z = self.w

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
