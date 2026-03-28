#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveCircle(Node):
    def __init__(self):
        super().__init__('move_circle')

        self.declare_parameter('turtle', 'turtle2')
        self.declare_parameter('radius', 3.0)
        self.declare_parameter('ang', 0.8)

        turtle = str(self.get_parameter('turtle').value)
        self.radius = float(self.get_parameter('radius').value)
        self.ang = float(self.get_parameter('ang').value)

        topic = f'/{turtle}/cmd_vel'
        self.pub = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        cmd = Twist()
        cmd.angular.z = self.ang
        cmd.linear.x = abs(self.ang) * self.radius
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
