#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')

        self.declare_parameter('side', 2.5)
        self.declare_parameter('lin_speed', 1.2)
        self.declare_parameter('k_heading', 6.0)
        self.declare_parameter('k_turn', 6.0)
        self.declare_parameter('max_ang', 2.2)

        self.side = float(self.get_parameter('side').value)
        self.lin = float(self.get_parameter('lin_speed').value)
        self.k_heading = float(self.get_parameter('k_heading').value)
        self.k_turn = float(self.get_parameter('k_turn').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.pose = None
        self.phase = 'move'
        self.edge_start_x = None
        self.edge_start_y = None
        self.edge_heading = None
        self.stop_ticks = 0
        self.timer = self.create_timer(0.05, self.tick)

    def pose_cb(self, msg: Pose):
        self.pose = msg
        if self.edge_start_x is None:
            self.edge_start_x = msg.x
            self.edge_start_y = msg.y
            self.edge_heading = msg.theta

    def tick(self):
        if self.pose is None:
            return

        cmd = Twist()

        if self.stop_ticks > 0:
            self.stop_ticks -= 1
            self.pub.publish(cmd)
            return

        if self.phase == 'move':
            dx = self.pose.x - self.edge_start_x
            dy = self.pose.y - self.edge_start_y
            traveled = math.sqrt(dx*dx + dy*dy)
            remaining = self.side - traveled

            if remaining <= 0.02:
                self.phase = 'turn'
                self.edge_heading = wrap(self.edge_heading + math.pi / 2.0)
                self.stop_ticks = 4
                self.pub.publish(cmd)
                return

            heading_err = wrap(self.edge_heading - self.pose.theta)

            v = self.lin
            if remaining < 0.5:
                v = max(0.35, self.lin * (remaining / 0.5))

            w = self.k_heading * heading_err
            w = max(-self.max_ang, min(self.max_ang, w))

            cmd.linear.x = v
            cmd.angular.z = w

        else:
            err = wrap(self.edge_heading - self.pose.theta)

            if abs(err) < 0.015:
                self.phase = 'move'
                self.edge_start_x = self.pose.x
                self.edge_start_y = self.pose.y
                self.stop_ticks = 4
                self.pub.publish(cmd)
                return

            w = self.k_turn * err
            w = max(-self.max_ang, min(self.max_ang, w))
            cmd.angular.z = w

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MoveSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
