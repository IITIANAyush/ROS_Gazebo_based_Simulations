#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class PathFollowerGVD(Node):

    def __init__(self):
        super().__init__('path_follower_gvd')

        self.path = None
        self.pose = None
        self.current_idx = 0

        self.path_topic = '/planned_path_gvd'

        qos_policy = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_policy
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("GVD Path Follower started.")

    # ================= CALLBACKS =================

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_idx = 0
        self.get_logger().info(f"GVD path received ({len(self.path)} points)")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    # ================= CONTROL LOOP =================

   # ================= CONTROL LOOP =================

    def control_loop(self):
        if self.path is None or self.pose is None:
            return

        rx = self.pose.position.x
        ry = self.pose.position.y

        goal_x = self.path[-1].pose.position.x
        goal_y = self.path[-1].pose.position.y
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)

        if dist_to_goal < 0.2:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("GVD Goal reached.")
            return

    # -------- INDEX-BASED LOOKAHEAD (GVD STYLE) --------
        min_dist = float('inf')
        closest_idx = self.current_idx

        for i in range(self.current_idx, len(self.path)):
            px = self.path[i].pose.position.x
            py = self.path[i].pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        self.current_idx = closest_idx

    # 🔴 KEY DIFFERENCE: LARGER LOOKAHEAD
        LOOKAHEAD_DIST = 1.0
        lookahead_idx = self.current_idx

        while lookahead_idx < len(self.path):
            px = self.path[lookahead_idx].pose.position.x
            py = self.path[lookahead_idx].pose.position.y
            if math.hypot(px - rx, py - ry) > LOOKAHEAD_DIST:
                break
            lookahead_idx += 1

        if lookahead_idx >= len(self.path):
            lookahead_idx = len(self.path) - 1

        target = self.path[lookahead_idx]

        tx = target.pose.position.x
        ty = target.pose.position.y

    # -------- HEADING --------
        angle_to_target = math.atan2(ty - ry, tx - rx)

        q = self.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        error = angle_to_target - yaw
        error = math.atan2(math.sin(error), math.cos(error))

    # -------- GVD-SPECIFIC CONTROL --------
        cmd = Twist()

        MAX_LIN = 0.08       # slower = smoother
        MAX_ANG = 0.4
        ANGLE_THRESH = 0.6   # pivot only if REALLY misaligned

        if abs(error) > ANGLE_THRESH:
        # Soft pivot (no aggressive snapping)
            cmd.linear.x = 0.02
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, 0.9 * error))
            mode = "SOFT_PIVOT"
        else:
        # Continuous curved motion (THIS makes GVD visible)
            cmd.linear.x = min(MAX_LIN, dist_to_goal)
            cmd.angular.z = max(-0.4, min(0.4, 0.6 * error))
            mode = "CURVE_FOLLOW"

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"GVD {mode} | Dist: {dist_to_goal:.2f} | Err: {error:.2f}",
            throttle_duration_sec=1.0
        )


def main():
    rclpy.init()
    node = PathFollowerGVD()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

