#!/usr/bin/env python3
"""
move_forward.py — Simple open-loop testing script.

Applies a constant forward thrust (Surge) by commanding the 4 horizontal thrusters 
(t1 to t4). It includes a ramp-up phase to avoid sudden physics jumps in Gazebo.
Useful to verify thruster mapping and signs.

Usage:
    ros2 run AUV_controller move_forward
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

# Thruster force to command conversion coefficients (from URDF)
THRUST_COEFFS = [-0.02, 0.02, -0.02, 0.02, -0.02, 0.02, 0.02, -0.02]
RHO = 997.0 

class MoveForwardNode(Node):
    """ROS 2 Node to command the AUV to move forward in open loop."""
    def __init__(self):
        super().__init__('test_move_forward')
        
        # Synchronize with Gazebo simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # Publishers for all 8 thrusters
        self.pubs = [self.create_publisher(Float64, f'/cmd_vel_{i}', 10) for i in range(1, 9)]
        
        # Target global force (N)
        self.target_F_surge = 141.0
        self.F_sway = 0.0
        
        # Ramp duration (seconds) to gradually apply force
        self.ramp_duration = 5.0
        
        # Timers for control loop and display
        self.command_timer = self.create_timer(0.1, self.publish_thrusts)
        self.display_timer = self.create_timer(0.2, self.display_time)
        self.start_time = None

    def display_time(self):
        """Logs the simulation time."""
        current_time = self.get_clock().now()
        current_sec = current_time.nanoseconds * 1e-9
        
        if self.start_time is None:
            if current_sec > 0:
                self.start_time = current_sec
            return
            
        elapsed = current_sec - self.start_time
        self.get_logger().info(f"[Gazebo Time] Simulation has been running for: {elapsed:.1f} seconds")

    def publish_thrusts(self):
        """Calculates and publishes the thruster commands."""
        if self.start_time is None:
            return
            
        current_time = self.get_clock().now()
        current_sec = current_time.nanoseconds * 1e-9
        elapsed = current_sec - self.start_time
        
        # Ramp-up the force to avoid sudden physics impulses
        if elapsed < self.ramp_duration:
            current_F_surge = self.target_F_surge * (elapsed / self.ramp_duration)
        else:
            current_F_surge = self.target_F_surge
            
        sin45 = 0.7071
        
        # Theoretical calculation for debugging (not actually used here since cmd is hardcoded below)
        t1 = (current_F_surge + self.F_sway) / (4 * sin45)
        t2 = (current_F_surge - self.F_sway) / (4 * sin45)
        t3 = (-current_F_surge + self.F_sway) / (4 * sin45)
        t4 = (-current_F_surge - self.F_sway) / (4 * sin45)
        
        # Hardcoded thruster commands for pure forward movement
        # 5N on t1, t2 and -5N on t3, t4 pushes the robot forward
        cmd = [-5, 5, -5, 5, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info(f"Global Thrust: {current_F_surge:.1f} N | T1-T4: {t1:.1f} N")

        for i in range(8):
            desired_force = float(cmd[i])
            c = THRUST_COEFFS[i]
            msg = Float64()
            # Apply coefficient sign to compensate for Gazebo propeller rotation setup
            msg.data = desired_force * math.copysign(1.0, c)
            self.pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
