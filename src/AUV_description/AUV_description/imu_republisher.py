import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Realistic covariance values derived from URDF noise parameters:
#   orientation stddev ~ 0.01 rad  → variance = 1e-4
#   angular velocity stddev = 2e-4 rad/s → variance = 4e-8
#   linear acceleration stddev = 1.7e-2 m/s² → variance = 2.89e-4

ORIENT_VAR = 1e-4    # rad²
ANGVEL_VAR = 4e-8    # (rad/s)²
LINACC_VAR = 2.89e-4 # (m/s²)²

class ImuRepublisher(Node):
    """
    Republishes /imu with non-zero covariances so that robot_localization
    (EKF) can properly weight and reject IMU measurements.
    Gazebo Harmonic sensor_msgs/Imu messages have all-zero covariances which
    cause undefined behaviour in the EKF (infinite trust).
    """
    def __init__(self):
        super().__init__('imu_republisher')
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/imu/fixed', 10)
        self.get_logger().info('IMU Republisher started: /imu → /imu/fixed with realistic covariances')

    def cb(self, msg: Imu):
        # Inject realistic covariances (diagonal 3x3 stored as flat 9-element list)
        msg.orientation_covariance = [
            ORIENT_VAR, 0.0, 0.0,
            0.0, ORIENT_VAR, 0.0,
            0.0, 0.0, ORIENT_VAR
        ]
        msg.angular_velocity_covariance = [
            ANGVEL_VAR, 0.0, 0.0,
            0.0, ANGVEL_VAR, 0.0,
            0.0, 0.0, ANGVEL_VAR
        ]
        msg.linear_acceleration_covariance = [
            LINACC_VAR, 0.0, 0.0,
            0.0, LINACC_VAR, 0.0,
            0.0, 0.0, LINACC_VAR
        ]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
