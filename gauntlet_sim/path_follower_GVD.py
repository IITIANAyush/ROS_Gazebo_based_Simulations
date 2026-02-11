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
        self.idx = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Path, '/planned_path_gvd', self.path_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("✅ GVD FOLLOWER (BRUTE FORCE MODE) STARTED")

    # ================= CALLBACKS =================

    def path_cb(self, msg):
        if self.path is not None:
            return  # Ignore re-published path

        self.path = msg.poses
        self.idx = 0
        self.get_logger().info(f"GVD path locked with {len(self.path)} points")
    
    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    # ================= CONTROL =================

    def control_loop(self):
        if self.path is None or self.pose is None:
            return

        if self.idx >= len(self.path):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("GVD PATH COMPLETE")
            return

        rx = self.pose.position.x
        ry = self.pose.position.y

        target = self.path[self.idx].pose.position
        dx = target.x - rx
        dy = target.y - ry
        dist = math.hypot(dx, dy)

        # --- Advance waypoint ---
        if dist < 0.2:
            self.get_logger().info(f"Reached waypoint {self.idx}")
            self.idx += 1
            self.cmd_pub.publish(Twist())
            return

        # --- Heading ---
        angle_to_target = math.atan2(dy, dx)

        q = self.pose.orientation
        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        error = angle_to_target - yaw
        error = math.atan2(math.sin(error), math.cos(error))

        cmd = Twist()

        # --- ROTATE FIRST ---
        if abs(error) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 * error
            mode = "ROTATE"
        else:
            cmd.linear.x = 0.08
            cmd.angular.z = 0.0
            mode = "DRIVE"

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"{mode} | wp={self.idx} | dist={dist:.2f} | err={error:.2f}",
            throttle_duration_sec=1.0
        )


def main():
    rclpy.init()
    node = PathFollowerGVD()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

