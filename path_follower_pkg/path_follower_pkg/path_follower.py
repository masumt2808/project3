#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pathlib import Path
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber to real odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Load waypoints
        self.waypoints = self.load_waypoints('/home/masum/project3_ws/src/path_follower_pkg/path_coords.txt')
        self.current_index = 0

        # Robot parameters
        self.wheel_radius = 0.033
        self.wheel_base = 0.287

        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.angle_threshold = math.radians(5)  # 5 degrees
        self.distance_threshold = 0.05  # 5 cm

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # radians

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def load_waypoints(self, filename):
        path = Path(filename)
        if not path.exists():
            self.get_logger().error(f"Waypoint file {filename} not found.")
            return []
        with open(filename, 'r') as f:
            lines = f.readlines()
            waypoints = [tuple(map(float, line.strip().split(','))) for line in lines if line.strip()]
        return waypoints

    def odom_callback(self, msg):
        # Update robot pose from /odom
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Orientation (quaternion to yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        # Finished all waypoints
        if self.current_index >= len(self.waypoints):
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('All waypoints reached!')
            self.timer.cancel()
            return

        goal_x, goal_y = self.waypoints[self.current_index]

        # Calculate error
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_error = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_heading - self.current_theta)

        twist = Twist()

        if distance_error < self.distance_threshold:
            # Waypoint reached
            self.get_logger().info(f'Waypoint {self.current_index} reached at ({self.current_x:.2f},{self.current_y:.2f})!')
            self.current_index += 1
            return

        if abs(angle_error) > self.angle_threshold:
            # Rotate in place
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
        else:
            # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

