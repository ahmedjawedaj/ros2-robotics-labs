#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


class Turtlebot3LaserScan(Node):
    def __init__(self):
        super().__init__('turtlebot3_laserscan')

        # Gazebo Harmonic publishes /scan with BEST_EFFORT
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos)

        self.declare_parameter('print_hz', 5.0)  # limit spam
        self.print_hz = float(self.get_parameter('print_hz').value)
        self._last_print_ns = 0

        self.get_logger().info("Listening to /scan (BEST_EFFORT). Drive robot with teleop to see updates.")

    def cb(self, msg: LaserScan):
        # Throttle prints
        now_ns = self.get_clock().now().nanoseconds
        if self.print_hz > 0:
            period_ns = int(1e9 / self.print_hz)
            if now_ns - self._last_print_ns < period_ns:
                return
            self._last_print_ns = now_ns

        ranges = msg.ranges

        # "Valid" for this task = finite hits (inf means no obstacle hit)
        finite = [(i, r) for i, r in enumerate(ranges) if math.isfinite(r)]
        inf_count = len(ranges) - len(finite)

        if not finite:
            self.get_logger().warn(
                f"No finite hits yet. inf(no-hit)={inf_count}/{len(ranges)}. "
                "Insert an object in Gazebo and drive closer."
            )
            return

        # closest and farthest among finite readings
        i_min, r_min = min(finite, key=lambda t: t[1])
        i_max, r_max = max(finite, key=lambda t: t[1])

        ang_min = msg.angle_min + i_min * msg.angle_increment
        ang_max = msg.angle_min + i_max * msg.angle_increment

        print("\n================ LaserScan Stats ================")
        print(f"Finite hits: {len(finite)}/{len(ranges)}   inf(no-hit): {inf_count}")
        print(f"Closest:  range={r_min:.3f} m  angle={ang_min:.3f} rad ({rad2deg(ang_min):.1f} deg)  index={i_min}")
        print(f"Farthest: range={r_max:.3f} m  angle={ang_max:.3f} rad ({rad2deg(ang_max):.1f} deg)  index={i_max}")
        print("=================================================")


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3LaserScan()
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
