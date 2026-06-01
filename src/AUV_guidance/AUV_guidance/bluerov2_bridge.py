#!/usr/bin/env python3
"""
bluerov2_bridge.py
===================
Bridge node converting geometry_msgs/msg/Wrench commands into MAVROS OverrideRCIn
for the BlueROV2 Pixhawk running ArduSub.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Wrench
from mavros_msgs.msg import OverrideRCIn

# Mapping Channels for ArduSub (1-indexed mapping to 0-indexed array)
CH_PITCH    = 0
CH_ROLL     = 1
CH_THROTTLE = 2
CH_YAW      = 3
CH_FORWARD  = 4
CH_LATERAL  = 5

class BlueROV2Bridge(Node):
    def __init__(self):
        super().__init__('bluerov2_bridge')

        self.declare_parameter('max_force', 40.0)
        self.declare_parameter('max_torque', 20.0)

        self.max_force = self.get_parameter('max_force').value
        self.max_torque = self.get_parameter('max_torque').value

        self.sub = self.create_subscription(
            Wrench,
            '/auv/command_wrench',
            self.wrench_cb,
            10
        )
        self.pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        self.get_logger().info(f"BlueROV2Bridge started (max_force={self.max_force}, max_torque={self.max_torque})")

    def wrench_cb(self, msg: Wrench):
        rc_msg = OverrideRCIn()
        # Initialize all 18 channels to UINT16_MAX (65535) which means "ignore" in MAVROS
        rc_msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18

        # Normalize an effort value and convert it to PWM [1100, 1900]
        def to_pwm(value, max_val):
            norm = np.clip(value / max_val, -1.0, 1.0)
            return int(1500 + norm * 400)

        # ArduSub Channel Mapping
        rc_msg.channels[CH_PITCH]    = to_pwm(msg.torque.y, self.max_torque)
        rc_msg.channels[CH_ROLL]     = to_pwm(msg.torque.x, self.max_torque)
        rc_msg.channels[CH_THROTTLE] = to_pwm(msg.force.z, self.max_force)
        rc_msg.channels[CH_YAW]      = to_pwm(msg.torque.z, self.max_torque)
        rc_msg.channels[CH_FORWARD]  = to_pwm(msg.force.x, self.max_force)
        
        # Inversion of lateral force (ROS Y=Left, ArduSub Lateral=Right)
        rc_msg.channels[CH_LATERAL]  = to_pwm(-msg.force.y, self.max_force)

        self.pub.publish(rc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
