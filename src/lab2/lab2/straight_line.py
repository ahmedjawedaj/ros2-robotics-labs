#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class StraightLine(Node):
    def __init__(self):
        super().__init__('straight_line')

        self.declare_parameter('speed', 1.0)
        self.declare_parameter('distance', 2.0)

        self.speed = float(self.get_parameter('speed').value)
        self.distance = float(self.get_parameter('distance').value)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.start_x = None
        self.start_y = None
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)

    def pose_cb(self, msg: Pose):
        if self.start_x is None:
            self.start_x = msg.x
            self.start_y = msg.y
            return

        dx = msg.x - self.start_x
        dy = msg.y - self.start_y
        traveled = math.sqrt(dx*dx + dy*dy)
        if traveled >= self.distance:
            self.done = True

    def tick(self):
        cmd = Twist()
        if not self.done:
            cmd.linear.x = self.speed
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StraightLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
