import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.twist = Twist()

        # Gains
        self.k_lateral = 0.015
        self.k_heading = 0.009
        self.max_angular = 2.5

        self.get_logger().info('Line follower node started.')

        cv2.namedWindow('ROI', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Gray', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)

        cv2.resizeWindow('ROI', 1100, 350)
        cv2.resizeWindow('Gray', 1100, 350)
        cv2.resizeWindow('Mask', 1100, 350)

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def get_track_center(self, mask, row_y):
        row = mask[row_y, :]
        white_indices = np.where(row > 0)[0]

        if len(white_indices) < 20:
            return None, None, None

        left_edge = int(white_indices[0])
        right_edge = int(white_indices[-1])
        center_x = (left_edge + right_edge) // 2

        return left_edge, right_edge, center_x

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        height, width, _ = frame.shape

        # Wider ROI for turn anticipation
        roi = frame[int(height * 0.58):int(height * 0.95), :].copy()

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        _, mask = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        image_center_x = mask.shape[1] // 2

        # Two rows:
        near_y = int(mask.shape[0] * 0.82)
        far_y = int(mask.shape[0] * 0.52)

        near_left, near_right, near_center = self.get_track_center(mask, near_y)
        far_left, far_right, far_center = self.get_track_center(mask, far_y)

        if near_center is None or far_center is None:
            self.stop_robot()
            self.get_logger().warn('Track rows not detected. Robot stopped.')
            cv2.imshow('ROI', roi)
            cv2.imshow('Gray', gray)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)
            return

        lateral_error = near_center - image_center_x
        heading_error = far_center - near_center

        # Reduce tiny oscillations
        if abs(lateral_error) < 3:
            lateral_error = 0
        if abs(heading_error) < 3:
            heading_error = 0

        angular_z = -(
            self.k_lateral * lateral_error +
            self.k_heading * heading_error
        )

        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

        # Slow down on stronger steering
        abs_total = abs(lateral_error) + abs(heading_error)

        if abs_total < 12:
            linear_x = 0.11
        elif abs_total < 25:
            linear_x = 0.08
        else:
            linear_x = 0.055

        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.cmd_pub.publish(self.twist)

        # Debug drawing
        cv2.line(roi, (0, near_y), (roi.shape[1], near_y), (255, 255, 0), 2)
        cv2.line(roi, (0, far_y), (roi.shape[1], far_y), (0, 255, 255), 2)

        cv2.circle(roi, (near_left, near_y), 5, (0, 255, 255), -1)
        cv2.circle(roi, (near_right, near_y), 5, (0, 255, 255), -1)
        cv2.circle(roi, (near_center, near_y), 8, (0, 0, 255), -1)

        cv2.circle(roi, (far_left, far_y), 5, (255, 0, 255), -1)
        cv2.circle(roi, (far_right, far_y), 5, (255, 0, 255), -1)
        cv2.circle(roi, (far_center, far_y), 8, (0, 165, 255), -1)

        cv2.line(
            roi,
            (image_center_x, 0),
            (image_center_x, roi.shape[0]),
            (255, 0, 0),
            2
        )

        self.get_logger().info(
            f'near={near_center}, far={far_center}, center={image_center_x}, '
            f'lateral_error={lateral_error}, heading_error={heading_error}, '
            f'linear_x={linear_x:.3f}, angular_z={angular_z:.3f}'
        )

        cv2.imshow('ROI', roi)
        cv2.imshow('Gray', gray)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.twist.linear.x = 0.0
    node.twist.angular.z = 0.0
    node.cmd_pub.publish(node.twist)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
