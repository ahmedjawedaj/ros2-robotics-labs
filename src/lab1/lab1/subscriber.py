#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lab1_interfaces.msg import ComplexNumber

class ComplexSubscriber(Node):
    def __init__(self):
        super().__init__('complex_subscriber')
        self.create_subscription(ComplexNumber, '/complex_numbers', self.cb, 10)
        self.get_logger().info("Subscribed to /complex_numbers (ComplexNumber)")

    def cb(self, msg: ComplexNumber):
        self.get_logger().info(f"Received: real={msg.real}, imaginary={msg.imaginary}")

def main(args=None):
    rclpy.init(args=args)
    node = ComplexSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
