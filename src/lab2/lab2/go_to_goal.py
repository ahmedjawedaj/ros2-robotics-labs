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

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('k_lin', 1.2)
        self.declare_parameter('k_ang', 6.0)
        self.declare_parameter('tolerance', 0.2)
        self.declare_parameter('max_lin', 2.0)
        self.declare_parameter('max_ang', 4.0)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.pose = None
        self.timer = self.create_timer(0.05, self.tick)

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def tick(self):
        if self.pose is None:
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        dist = math.sqrt(dx*dx + dy*dy)

        cmd = Twist()

        if dist < self.tol:
            self.pub.publish(cmd)
            return

        desired = math.atan2(dy, dx)
        err = wrap(desired - self.pose.theta)

        ang = self.k_ang * err
        ang = max(-self.max_ang, min(self.max_ang, ang))

        lin = self.k_lin * dist
        lin = max(0.0, min(self.max_lin, lin))

        if abs(err) > 0.6:
            lin = 0.0

        cmd.linear.x = lin
        cmd.angular.z = ang
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
