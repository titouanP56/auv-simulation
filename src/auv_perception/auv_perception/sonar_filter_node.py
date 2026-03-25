import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class SonarFilterNode(Node):
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
        # 1. Extraction propre des coordonnées X, Y, Z dans une liste
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points_gen)
        
        # Sécurité : si le nuage est vide (ex: le capteur n'a rien vu)
        if not points_list:
            return
            
        # 2. Conversion en tableau Numpy. 
        # Attention, point_cloud2 renvoie un tableau structuré 1D avec les champs 'x', 'y', 'z' !
        points_array = np.array(points_list)
        
        # 3. Calcul de la distance D = sqrt(X^2 + Y^2 + Z^2) pour chaque point
        # On extrait les champs par leur nom au lieu d'utiliser des indices qui causent des erreurs
        x = points_array['x']
        y = points_array['y']
        z = points_array['z']
        distances = np.sqrt(x**2 + y**2 + z**2)
        
        # 4. Application du filtre strict à 4.0 mètres
        filtered_points = points_array[distances <= 4.0]
        
        if len(filtered_points) == 0:
            return
            
        # 5. Reconstruction et publication du message
        # create_cloud_xyz32 attend une liste de tuples (x,y,z), tolist() sur notre tableau structuré fait ça !
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
