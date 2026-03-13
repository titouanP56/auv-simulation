#!/usr/bin/env python3
"""
move_down.py — Simple open-loop testing script.

Applies a constant downward thrust (Heave) by commanding the 4 vertical thrusters 
(t5 to t8). Useful to verify vertical thruster mapping and signs.

Usage:
    ros2 run AUV_controller move_down
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

# Thruster force to command conversion coefficients (from URDF)
THRUST_COEFFS = [-0.02, 0.02, -0.02, 0.02, -0.02, 0.02, 0.02, -0.02]
RHO = 1025.0

class MoveDownNode(Node):
    """ROS 2 Node to command the AUV to dive (move down) in open loop."""
    def __init__(self):
        super().__init__('test_move_down')
        
        # Publishers for all 8 thrusters
        self.pubs = [self.create_publisher(Float64, f'/cmd_vel_{i}', 10) for i in range(1, 9)]
    
        # Hardcoded thruster commands for pure downward movement
        # Adjusting t5-t8 to produce a net downward force without inducing pitch/roll
        t5 = 40
        t6 = -40
        t7 = -40
        t8 = 40
        
        cmd = [0.0, 0.0, 0.0, 0.0, t5, t6, t7, t8]
        
        self.get_logger().info(f"Plonger publiera les forces : {cmd[4:]}")
        self.timer = self.create_timer(0.1, lambda: self.publish_thrusts(cmd))

    def publish_thrusts(self, cmd):
        """Calculates and publishes the thruster commands."""
        for i in range(8):
            desired_force = float(cmd[i])
            c = THRUST_COEFFS[i]
            msg = Float64()
            # Apply coefficient sign to compensate for Gazebo propeller rotation setup
            msg.data = desired_force * math.copysign(1.0, c)
            self.pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveDownNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
