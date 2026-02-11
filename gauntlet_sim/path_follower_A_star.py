#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')
        self.current_idx =0

        self.path = None
        self.pose = None

        # 1. Topic Names
        self.path_topic = '/planned_path_astar'
        
        # 2. QoS Profile (The "Secret Sauce" for ROS 2)
        # Many Gazebo plugins publish Odom as "Best Effort". 
        # If we listen with "Reliable" (default), we hear nothing.
        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

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
            qos_policy  # <--- FIXED QoS here
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path Follower Initialized. Waiting for data...")

    # ================= CALLBACKS =================

    def path_callback(self, msg):
        self.path = msg.poses
        # Log only once or when path changes significantly
        self.get_logger().info(f"Path received! {len(self.path)} points.", once=True)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.get_logger().info("Odom received!", once=True)

    # ================= CONTROL LOOP =================

    def control_loop(self):
        if self.path is None or self.pose is None:
            return

        rx = self.pose.position.x
        ry = self.pose.position.y

        # 1. Get Final Goal
        goal_x = self.path[-1].pose.position.x
        goal_y = self.path[-1].pose.position.y
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)

        if dist_to_goal < 0.15:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("GOAL REACHED!", once=True)
            return

        min_dist = float('inf')
        closest_idx = self.current_idx

        for i in range(self.current_idx, len(self.path)):
            px = self.path[i].pose.position.x
            py = self.path[i].pose.position.y
            d = math.hypot(px - rx, py - ry)
  
            if d < min_dist:
                min_dist = d
                closest_idx = i

# Update progress (NEVER go backwards)
        self.current_idx = closest_idx

# 2. Find lookahead point AHEAD of current_idx
        LOOKAHEAD_DIST = 0.6
        lookahead_idx = self.current_idx

        while lookahead_idx < len(self.path):
            px = self.path[lookahead_idx].pose.position.x
            py = self.path[lookahead_idx].pose.position.y
            if math.hypot(px - rx, py - ry) > LOOKAHEAD_DIST:
                break
            lookahead_idx += 1

# Clamp index
        if lookahead_idx >= len(self.path):
            lookahead_idx = len(self.path) - 1

        lookahead = self.path[lookahead_idx]

        tx = lookahead.pose.position.x
        ty = lookahead.pose.position.y
        # 3. Heading Calculation
        angle_to_target = math.atan2(ty - ry, tx - rx)
        
        q = self.pose.orientation
        # Standard Yaw conversion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        error = angle_to_target - yaw
        error = math.atan2(math.sin(error), math.cos(error)) # Normalize

        # 4. Control Logic with Damping
        cmd = Twist()
        MAX_LIN = 0.1     # Keep it steady
        MAX_ANG = 0.6      # Reduced to stop the spinning-top effect
        ANGLE_THRESH = 0.35 

        if abs(error) > ANGLE_THRESH:
            # PIVOT: Turn slowly toward the path
            cmd.linear.x = 0.0
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, 1 * error))
            mode = "PIVOT"
        else:
            # DRIVE: Move forward while adjusting heading
            cmd.linear.x = min(MAX_LIN, dist_to_goal)
            cmd.angular.z = max(-0.5, min(0.5, 0.8 * error)) # Gentler correction
            mode = "DRIVE"

        self.cmd_pub.publish(cmd)
        
        self.get_logger().info(f"{mode} | Dist: {dist_to_goal:.2f} | Err: {error:.2f}", throttle_duration_sec=1.0)
def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
