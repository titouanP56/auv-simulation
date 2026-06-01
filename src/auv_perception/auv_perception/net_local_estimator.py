#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
import numpy as np
import math
from collections import deque
import tf2_ros
import tf2_geometry_msgs

def quaternion_from_euler(ai, aj, ak):
    """
    Convert Euler angles (Roll, Pitch, Yaw) to quaternion.
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs # x
    q[1] = cj*ss + sj*cc # y
    q[2] = cj*cs - sj*sc # z
    q[3] = cj*cc + sj*ss # w
    return q

class NetLocalEstimator(Node):
    """
    ROS 2 Node to estimate the local pose of the net relative to the robot.
    
    It subscribes to the filtered Sonoptix point cloud, fits a line to the detected
    points using Principal Component Analysis (PCA), calculates the normal vector to 
    determine the net's orientation and distance, and publishes this as a PoseStamped
    in the odom frame.
    """
    def __init__(self):
        super().__init__('net_local_estimator')
        
        # Publisher for the target pose on the net
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/net_local_frame', 10)
        
        # Subscriber to filtered sonar points
        self.points_sub = self.create_subscription(
            PointCloud2,
            '/sonoptix/points_filtered',
            self.points_callback,
            10
        )
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Filtering parameters
        self.distance_history = deque(maxlen=5) # Moving average window of 5
        self.angle_history = deque(maxlen=5)    # To smooth orientation too
        
        self.get_logger().info("Net local estimator initialized.")

    def points_callback(self, msg):
        # Convert PointCloud2 to numpy array
        # We only care about x, y for the 2D projection
        points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if len(points) < 5:
            # Not enough points to fit a line
            return
            
        # Filter points: keep those in a horizontal cone of +/- 45 degrees
        angles = np.arctan2(points[:, 1], points[:, 0])
        mask = np.abs(angles) <= (np.pi / 4.0)
        filtered_points = points[mask]
        
        if len(filtered_points) < 5:
            return
            
        # Extract X and Y for regression
        X = filtered_points[:, 0]
        Y = filtered_points[:, 1]
        
        # Use PCA (Total Least Squares) for robust line fitting
        mean_x = np.mean(X)
        mean_y = np.mean(Y)
        
        # Center the data
        centered_x = X - mean_x
        centered_y = Y - mean_y
        
        # Covariance matrix
        cov_matrix = np.cov(centered_x, centered_y)
        
        # Eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # The eigenvector with the smallest eigenvalue is the normal to the line
        normal = eigenvectors[:, 0]
        
        # Ensure normal vector points towards the net (away from sensor/origin)
        # Vector from sensor (0,0) to mean point is (mean_x, mean_y)
        mean_vector = np.array([mean_x, mean_y])
        if np.dot(normal, mean_vector) < 0:
            normal = -normal
            
        # The line equation is normal[0]*(x - mean_x) + normal[1]*(y - mean_y) = 0
        # The closest point on the line to the origin (0,0)
        # distance d = |normal . mean_vector| (since normal is unit length)
        # The closest point is d * normal
        distance = np.dot(normal, mean_vector)
        
        # Apply moving average filter
        self.distance_history.append(distance)
        avg_distance = np.mean(self.distance_history)
        
        # Calculate yaw of the normal vector for smoothing
        yaw = math.atan2(normal[1], normal[0])
        self.angle_history.append(yaw)
        # Handle wraparound in circular mean safely (simplified here assuming mostly consistent directions)
        avg_yaw = np.mean(self.angle_history) 
        
        avg_normal = np.array([math.cos(avg_yaw), math.sin(avg_yaw)])
        
        closest_point_x = avg_distance * avg_normal[0]
        closest_point_y = avg_distance * avg_normal[1]
        
        # Create PoseStamped in sensor frame
        pose_sensor = PoseStamped()
        pose_sensor.header.stamp = msg.header.stamp
        pose_sensor.header.frame_id = msg.header.frame_id
        
        pose_sensor.pose.position.x = closest_point_x
        pose_sensor.pose.position.y = closest_point_y
        pose_sensor.pose.position.z = np.mean(filtered_points[:, 2]) # Keep average height of detected points
        
        # Normal vector as orientation
        q = quaternion_from_euler(0, 0, avg_yaw)
        pose_sensor.pose.orientation.x = q[0]
        pose_sensor.pose.orientation.y = q[1]
        pose_sensor.pose.orientation.z = q[2]
        pose_sensor.pose.orientation.w = q[3]
        
        # Transform to odom frame
        try:
            # Look up transform from sensor frame to odom at the time of the message
            transform = self.tf_buffer.lookup_transform(
                'odom',
                pose_sensor.header.frame_id,
                rclpy.time.Time(), # Get the latest available transform
                rclpy.duration.Duration(seconds=0.1)
            )
            pose_odom = tf2_geometry_msgs.do_transform_pose(pose_sensor.pose, transform)
            
            # Construct final message
            final_msg = PoseStamped()
            final_msg.header.stamp = self.get_clock().now().to_msg()
            final_msg.header.frame_id = 'odom'
            final_msg.pose = pose_odom
            
            self.pose_pub.publish(final_msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform map to odom: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NetLocalEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
