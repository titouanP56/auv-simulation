import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class SonarFilterNode(Node):
    """
    ROS 2 Node to filter Sonoptix PointCloud2 data by distance.
    
    This node removes all points further than a specified distance threshold (4.0m)
    to reduce noise and focus processing on the nearby net structure.
    """
    def __init__(self):
        super().__init__('sonar_filter_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sonoptix/points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/sonoptix/points_filtered', 10)
        self.get_logger().info('SonarFilterNode started, filtering range <= 4.0m')

    def listener_callback(self, msg):
        # 1. Clean extraction of X, Y, Z coordinates into a list
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points_gen)
        
        # Safety: if the cloud is empty (e.g., sensor saw nothing)
        if not points_list:
            return
            
        # 2. Conversion to Numpy array. 
        # Note: point_cloud2 returns a 1D structured array with 'x', 'y', 'z' fields!
        points_array = np.array(points_list)
        
        # 3. Calculate distance D = sqrt(X^2 + Y^2 + Z^2) for each point
        # Extract fields by name instead of indices to avoid errors
        x = points_array['x']
        y = points_array['y']
        z = points_array['z']
        distances = np.sqrt(x**2 + y**2 + z**2)
        
        # 4. Apply strict filter at 4.0 meters
        filtered_points = points_array[distances <= 4.0]
        
        if len(filtered_points) == 0:
            return
            
        # 5. Reconstruction and publication of the message
        # create_cloud_xyz32 expects a list of tuples (x,y,z), tolist() on our structured array does this!
        filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points.tolist())
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
