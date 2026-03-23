import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import math
import struct
import numpy as np

class LawnmowerTrajectoryNode(Node):
    def __init__(self):
        super().__init__('lawnmower_trajectory_node')

        # State
        self.phase2_done = False
        self.state = "WAITING"
        
        # Local Trajectory Variables
        self.target_x = 1.5
        self.target_y = 0.0
        self.current_z = 0.0
        self.target_yaw = math.pi
        self.sweep_width = 2.0
        self.vy = 0.2
        self.vz = 0.5
        
        self.current_wall_dist = 1.5
        self.kp_wall = 0.5

        self.last_time = self.get_clock().now()

        # Subscriptions
        self.sub_phase2 = self.create_subscription(Bool, '/mission/phase2_done', self.phase2_done_callback, 10)
        self.sub_origin = self.create_subscription(PoseStamped, '/mission/local_origin', self.origin_callback, 10)
        self.sub_sonoptix = self.create_subscription(PointCloud2, '/sonoptix/points', self.sonoptix_callback, 10)

        # Publisher
        self.pub_setpoint = self.create_publisher(PoseStamped, '/cmd_setpoint', 10)

        # Timer for trajectory generation (10Hz)
        self.timer = self.create_timer(0.1, self.generate_trajectory)

    def _extract_sonoptix_data(self, msg: PointCloud2):
        field_map = {f.name: f for f in msg.fields}
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            return []

        x_off, y_off, z_off = field_map['x'].offset, field_map['y'].offset, field_map['z'].offset
        point_step, data = msg.point_step, msg.data
        
        boresight_half_angle = math.radians(20.0)
        valid_ranges = []
        
        for i in range(msg.width * msg.height):
            base = i * point_step
            try:
                px = struct.unpack_from('f', data, base + x_off)[0]
                py = struct.unpack_from('f', data, base + y_off)[0]
                pz = struct.unpack_from('f', data, base + z_off)[0]
            except struct.error:
                continue

            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue

            horiz_angle = abs(math.atan2(py, px))
            if horiz_angle > boresight_half_angle:
                continue

            r = math.sqrt(px * px + py * py + pz * pz)
            if r > 0.01:
                valid_ranges.append(r)

        return valid_ranges

    def phase2_done_callback(self, msg):
        if msg.data and not self.phase2_done:
            self.phase2_done = True
            self.state = "SWEEP_RIGHT"
            self.last_time = self.get_clock().now()
            self.get_logger().info("Phase 2 done signal received. Starting Lawnmower (SWEEP_RIGHT).")

    def origin_callback(self, msg):
        pass

    def sonoptix_callback(self, msg):
        if not self.phase2_done:
            return
        
        valid_ranges = self._extract_sonoptix_data(msg)
        if valid_ranges:
            self.current_wall_dist = np.median(valid_ranges)

    def generate_trajectory(self):
        if self.state == "WAITING" or not self.phase2_done:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Wall Following PID
        error = self.current_wall_dist - 1.5
        self.target_x = self.target_x - (self.kp_wall * error * dt)

        # Cartesian Lawnmower Logic
        if self.state == "SWEEP_RIGHT":
            self.target_y += self.vy * dt
            if self.target_y >= self.sweep_width / 2.0:
                self.target_y = self.sweep_width / 2.0
                self.state = "DROP_DOWN"
                self.next_state_after_drop = "SWEEP_LEFT"
                self.target_drop_z = self.current_z - 1.0
                self.get_logger().info(f"🔄 [Lawnmower] State changed to: {self.state}")

        elif self.state == "SWEEP_LEFT":
            self.target_y -= self.vy * dt
            if self.target_y <= -self.sweep_width / 2.0:
                self.target_y = -self.sweep_width / 2.0
                self.state = "DROP_DOWN"
                self.next_state_after_drop = "SWEEP_RIGHT"
                self.target_drop_z = self.current_z - 1.0
                self.get_logger().info(f"🔄 [Lawnmower] State changed to: {self.state}")

        elif self.state == "DROP_DOWN":
            self.current_z -= self.vz * dt
            if self.current_z <= self.target_drop_z:
                self.current_z = self.target_drop_z
                self.state = self.next_state_after_drop
                self.get_logger().info(f"🔄 [Lawnmower] State changed to: {self.state}")

        # Publish Setpoint
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "local_origin"
        
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.current_z
        
        # Yaw
        msg.pose.orientation.z = math.sin(self.target_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.target_yaw / 2.0)
        
        self.pub_setpoint.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LawnmowerTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
