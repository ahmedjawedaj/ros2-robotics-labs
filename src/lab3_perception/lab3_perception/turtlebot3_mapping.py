#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import tf2_ros
from tf2_ros import TransformException
import matplotlib.pyplot as plt


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Turtlebot3Mapping(Node):
    def __init__(self):
        super().__init__('turtlebot3_mapping')
        self.approach_dist = 1.20
        self.desired_dist  = 0.80
        self.v_approach    = 0.18
        self.v_orbit       = 0.14
        self.k_angle       = 1.5
        self.k_dist        = 1.0
        self.max_w         = 1.0
        self.max_runtime   = 200.0
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.scan = None
        self.state = 'APPROACH'
        self.points = []
        self._start_stamp = None
        self.last_yaw = None
        self.yaw_accum = 0.0
        self._log_n = 0
        self.timer = self.create_timer(0.05, self.tick)

    def scan_cb(self, msg):
        self.scan = msg

    def cmd(self, v, w):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.twist.linear.x = float(v)
        t.twist.angular.z = float(w)
        self.pub.publish(t)

    def get_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            return None
        return (tr.transform.translation.x, tr.transform.translation.y,
                quat_to_yaw(tr.transform.rotation))

    def closest_hit(self):
        s = self.scan
        best_d, best_i = min(
            ((d, i) for i, d in enumerate(s.ranges) if math.isfinite(d) and d > 0.05),
            default=(s.range_max, 0))
        return best_d, s.angle_min + best_i * s.angle_increment

    def tick(self):
        if self.scan is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._start_stamp is None:
            self._start_stamp = now
        if now - self._start_stamp > self.max_runtime:
            self.done()
            return
        pose = self.get_pose()
        if pose is None:
            return
        x, y, yaw = pose
        d_min, alpha = self.closest_hit()
        rmax = self.scan.range_max
        self._log_n += 1
        if self._log_n % 20 == 0:
            self.get_logger().info(
                f'[{self.state}] pos=({x:.2f},{y:.2f}) d_min={d_min:.2f} '
                f'alpha={math.degrees(alpha):.0f}deg yaw_acc={math.degrees(self.yaw_accum):.0f}deg')
        if d_min >= rmax * 0.99:
            self.cmd(0.0, 0.4)
            return
        if self.state == 'APPROACH':
            if d_min <= self.approach_dist:
                self.state = 'ORBIT'
                self.last_yaw = yaw
                self.yaw_accum = 0.0
                self.cmd(0.0, 0.0)
                self.get_logger().info(f'ORBIT started at pos=({x:.2f},{y:.2f})')
            else:
                ang = alpha if alpha <= math.pi else alpha - 2 * math.pi
                self.cmd(self.v_approach, max(-0.6, min(0.6, 1.5 * ang)))
            return
        if self.last_yaw is not None:
            delta = ((yaw - self.last_yaw) + math.pi) % (2 * math.pi) - math.pi
            self.yaw_accum += delta
        self.last_yaw = yaw
        xobs, yobs = d_min * math.cos(alpha), d_min * math.sin(alpha)
        self.points.append((
            math.cos(yaw) * xobs - math.sin(yaw) * yobs + x,
            math.sin(yaw) * xobs + math.cos(yaw) * yobs + y))
        if abs(self.yaw_accum) >= 2.0 * math.pi * 0.95:
            self.get_logger().info('Full circle detected.')
            self.done()
            return
        target = 3.0 * math.pi / 2.0
        angle_err = ((alpha - target) + math.pi) % (2 * math.pi) - math.pi
        w = (-self.v_orbit / max(0.3, self.desired_dist)
             + self.k_angle * angle_err + self.k_dist * (self.desired_dist - d_min))
        self.cmd(self.v_orbit, max(-self.max_w, min(self.max_w, w)))

    def done(self):
        self.timer.cancel()
        self.cmd(0.0, 0.0)
        self.get_logger().info(f'Points recorded: {len(self.points)}')
        if self.points:
            xs, ys = zip(*self.points)
            self.get_logger().info(f'X: {min(xs):.2f}..{max(xs):.2f} Y: {min(ys):.2f}..{max(ys):.2f}')
            plt.figure()
            plt.plot(xs, ys, 'b.')
            plt.gca().set_aspect('equal')
            plt.title('Map of Object')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.savefig('/tmp/mapping_plot.png', dpi=200)
            plt.show()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3Mapping()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()