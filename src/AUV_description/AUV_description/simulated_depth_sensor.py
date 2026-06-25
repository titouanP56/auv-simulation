#!/usr/bin/env python3
"""
simulated_depth_sensor.py — A node to simulate a depth sensor.

This node subscribes to the perfect Gazebo odometry (`/odom`), extracts 
the exact Z (depth) position, and adds Gaussian noise to simulate a real
pressure/depth sensor reading. It then publishes this as a PoseWithCovarianceStamped
message, which can be safely fused by the EKF (robot_localization).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import random

class SimulatedDepthSensor(Node):
    """ROS 2 Node that generates noisy depth measurements from exact simulation data."""
    def __init__(self):
        super().__init__('simulated_depth_sensor')
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # We publish the depth as a Pose measurement (Z-axis only)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/depth/pose', 10)
        
        self.get_logger().info('Simulated Depth Sensor Started - Publishing on /depth/pose')

    def odom_callback(self, msg):
        """Processes the exact Gazebo odometry to extract and publish a noisy Z position."""
        # We get the exact Z position from Gazebo
        exact_z = msg.pose.pose.position.z
        
        # Add realistic sensor noise (e.g. +/- 2cm standard deviation)
        noise = random.gauss(0.0, 0.02)
        noisy_z = exact_z + noise
            
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        # Usually sensors are in their own frame, but for absolute Z we can often use odom
        pose_msg.header.frame_id = "odom" 
        
        pose_msg.pose.pose.position.z = noisy_z
        
        # We must tell robot_localization that ONLY the Z measurement is valid.
        # We do this by setting a very high variance (uncertainty) for X, Y, Roll, Pitch, Yaw
        # and a low variance for Z.
        
        # In ROS, covariance is a 36-element array (6x6 matrix for x, y, z, roll, pitch, yaw)
        # Indexes along the diagonal are: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw.
        # 1e9 means "ignore this axis", 0.0004 means "trust this axis with 0.02m stddev".
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0]  = 1e9 # X
        pose_msg.pose.covariance[7]  = 1e9 # Y
        pose_msg.pose.covariance[14] = 0.0004 # 0.02^2 (variance in Z)
        pose_msg.pose.covariance[21] = 1e9 # Roll
        pose_msg.pose.covariance[28] = 1e9 # Pitch
        pose_msg.pose.covariance[35] = 1e9 # Yaw
        
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDepthSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
