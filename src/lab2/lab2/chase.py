import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class Chase(Node):
    def __init__(self):
        super().__init__('chase')

        self.declare_parameter('target', 'turtle1')
        self.declare_parameter('chaser', 'turtle2')

        self.declare_parameter('k_lin', 1.2)
        self.declare_parameter('k_ang', 6.0)
        self.declare_parameter('max_lin', 2.0)
        self.declare_parameter('max_ang', 4.0)
        self.declare_parameter('catch_dist', 0.5)

        self.declare_parameter('manual_override', True)
        self.declare_parameter('manual_timeout', 0.35)

        self.target_pose = None
        self.chaser_pose = None
        self.last_manual = None
        self.last_manual_time = None

        tgt = self.get_parameter('target').value
        chs = self.get_parameter('chaser').value

        self.create_subscription(Pose, f'/{tgt}/pose', self.cb_target, 10)
        self.create_subscription(Pose, f'/{chs}/pose', self.cb_chaser, 10)
        self.create_subscription(Twist, f'/{chs}/cmd_vel_manual', self.cb_manual, 10)

        self.pub = self.create_publisher(Twist, f'/{chs}/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)

    def cb_target(self, msg):
        self.target_pose = msg

    def cb_chaser(self, msg):
        self.chaser_pose = msg

    def cb_manual(self, msg):
        self.last_manual = msg
        self.last_manual_time = self.get_clock().now()

    def tick(self):
        if self.target_pose is None or self.chaser_pose is None:
            return

        manual_ok = bool(self.get_parameter('manual_override').value)
        manual_timeout = float(self.get_parameter('manual_timeout').value)

        if manual_ok and self.last_manual is not None and self.last_manual_time is not None:
            age = (self.get_clock().now() - self.last_manual_time).nanoseconds / 1e9
            if age <= manual_timeout:
                self.pub.publish(self.last_manual)
                return

        dx = self.target_pose.x - self.chaser_pose.x
        dy = self.target_pose.y - self.chaser_pose.y
        dist = math.hypot(dx, dy)

        catch_dist = float(self.get_parameter('catch_dist').value)
        if dist < catch_dist:
            self.pub.publish(Twist())
            return

        desired = math.atan2(dy, dx)
        err = wrap(desired - self.chaser_pose.theta)

        k_lin = float(self.get_parameter('k_lin').value)
        k_ang = float(self.get_parameter('k_ang').value)
        max_lin = float(self.get_parameter('max_lin').value)
        max_ang = float(self.get_parameter('max_ang').value)

        v = k_lin * dist
        if v > max_lin:
            v = max_lin

        w = k_ang * err
        if w > max_ang:
            w = max_ang
        if w < -max_ang:
            w = -max_ang

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Chase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
