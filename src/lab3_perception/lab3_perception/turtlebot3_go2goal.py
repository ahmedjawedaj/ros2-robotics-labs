#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(q) -> float:
    """Quaternion -> yaw (Z) using the standard formula."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoToGoalTB3(Node):
    def __init__(self):
        super().__init__('turtlebot3_go2goal')

        # Params (goal + controller tuning)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('k_lin', 0.6)
        self.declare_parameter('k_ang', 2.5)
        self.declare_parameter('tolerance', 0.15)
        self.declare_parameter('max_lin', 0.22)
        self.declare_parameter('max_ang', 2.0)
        self.declare_parameter('turn_only_thresh', 0.6)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.x = None
        self.y = None
        self.yaw = None

        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            f"Go2Goal active: goal=({self.goal_x:.2f}, {self.goal_y:.2f}), tol={self.tol}"
        )

    def _load_params(self) -> None:
        """Load current parameter values into fields."""
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.turn_only_thresh = float(self.get_parameter('turn_only_thresh').value)

    def _on_param_change(self, params):
        """
        Realtime param updates (goal + tuning).
        Validate values and apply immediately.
        """
        updates = {}
        for p in params:
            if p.name in (
                'goal_x', 'goal_y',
                'k_lin', 'k_ang',
                'tolerance',
                'max_lin', 'max_ang',
                'turn_only_thresh'
            ):
                updates[p.name] = p.value

        # Validate (keep it simple + safe)
        if 'tolerance' in updates and float(updates['tolerance']) <= 0.0:
            return SetParametersResult(successful=False, reason='tolerance must be > 0')
        if 'max_lin' in updates and float(updates['max_lin']) < 0.0:
            return SetParametersResult(successful=False, reason='max_lin must be >= 0')
        if 'max_ang' in updates and float(updates['max_ang']) < 0.0:
            return SetParametersResult(successful=False, reason='max_ang must be >= 0')
        if 'turn_only_thresh' in updates and float(updates['turn_only_thresh']) < 0.0:
            return SetParametersResult(successful=False, reason='turn_only_thresh must be >= 0')

        # Apply
        for name, val in updates.items():
            if name == 'goal_x':
                self.goal_x = float(val)
            elif name == 'goal_y':
                self.goal_y = float(val)
            elif name == 'k_lin':
                self.k_lin = float(val)
            elif name == 'k_ang':
                self.k_ang = float(val)
            elif name == 'tolerance':
                self.tol = float(val)
            elif name == 'max_lin':
                self.max_lin = float(val)
            elif name == 'max_ang':
                self.max_ang = float(val)
            elif name == 'turn_only_thresh':
                self.turn_only_thresh = float(val)

        if 'goal_x' in updates or 'goal_y' in updates:
            self.get_logger().info(f"Updated goal -> ({self.goal_x:.2f}, {self.goal_y:.2f})")

        return SetParametersResult(successful=True)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = quat_to_yaw(q)

    def tick(self):
        if self.x is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if dist < self.tol:
            self.pub.publish(cmd)
            return

        desired = math.atan2(dy, dx)
        err = wrap(desired - self.yaw)

        ang = max(-self.max_ang, min(self.max_ang, self.k_ang * err))
        lin = max(0.0, min(self.max_lin, self.k_lin * dist))
        if abs(err) > self.turn_only_thresh:
            lin = 0.0

        cmd.twist.linear.x = lin
        cmd.twist.angular.z = ang
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalTB3()
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
