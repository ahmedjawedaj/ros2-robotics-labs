#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lab1_interfaces.msg import ComplexNumber


class ComplexPublisher(Node):
    def __init__(self):
        super().__init__('complex_publisher')

        # Declare parameters with defaults (required by assignment style)
        self.declare_parameter('real', 0.0)
        self.declare_parameter('imaginary', 0.0)

        # Fallback / initial snapshot (useful for logging + safety)
        self.real = float(self.get_parameter('real').value)
        self.imag = float(self.get_parameter('imaginary').value)

        self.pub = self.create_publisher(ComplexNumber, '/complex_numbers', 10)
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info(
            f"Publishing ComplexNumber on /complex_numbers. "
            f"Initial params: real={self.real}, imaginary={self.imag}"
        )

    def tick(self):
        # Read live params every tick so runtime updates work (ros2 param set ...)
        real_val = self.get_parameter('real').value
        imag_val = self.get_parameter('imaginary').value

        # Fallback if something weird happens
        real = float(real_val) if real_val is not None else self.real
        imag = float(imag_val) if imag_val is not None else self.imag

        # Update fallback cache (optional, but keeps state consistent)
        self.real, self.imag = real, imag

        msg = ComplexNumber()
        msg.real = real
        msg.imaginary = imag
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ComplexPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
