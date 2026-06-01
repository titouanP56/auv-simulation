#!/usr/bin/env python3
"""
sim_thruster_bridge.py
======================
Bridge node converting geometry_msgs/msg/Wrench commands into 8 individual
Float64 /cmd_vel_X topics for the Gazebo simulation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64

# ── Thruster allocation ───────────────────────────────────────────────────────

THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]
SIN45 = 0.7071
LEVER = 0.1697

TAM = np.array([
    [ SIN45,  SIN45, -SIN45, -SIN45,  0.0,   0.0,   0.0,   0.0 ],
    [ SIN45, -SIN45,  SIN45, -SIN45,  0.0,   0.0,   0.0,   0.0 ],
    [ 0.0,    0.0,    0.0,    0.0,   -1.0,   1.0,   1.0,  -1.0 ],
    [ 0.0,    0.0,    0.0,    0.0,    0.218, 0.218, 0.218, 0.218],
    [ 0.0,    0.0,    0.0,    0.0,    0.12, -0.12,  0.12, -0.12 ],
    [ LEVER, -LEVER, -LEVER,  LEVER,  0.0,   0.0,   0.0,   0.0 ],
])
TAM_PINV = np.linalg.pinv(TAM)
MAX_INDIVIDUAL_THRUST = 40.0


class SimThrusterBridge(Node):
    def __init__(self):
        super().__init__('sim_thruster_bridge')

        self.sub = self.create_subscription(
            Wrench,
            '/auv/command_wrench',
            self.wrench_cb,
            10
        )
        
        self._thrust_pubs = [
            self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            for i in range(1, 9)
        ]

        self.get_logger().info("SimThrusterBridge started: translating Wrench to /cmd_vel_1..8")

    def wrench_cb(self, msg: Wrench):
        tau = np.array([
            msg.force.x,
            msg.force.y,
            msg.force.z,
            msg.torque.x,
            msg.torque.y,
            msg.torque.z
        ])
        
        raw_thrusts = TAM_PINV @ tau
        thrusts = np.clip(raw_thrusts, -MAX_INDIVIDUAL_THRUST, MAX_INDIVIDUAL_THRUST)

        for i, (thrust, coeff) in enumerate(zip(thrusts, THRUST_COEFFS)):
            out_msg = Float64()
            out_msg.data = float(thrust) * math.copysign(1.0, coeff)
            self._thrust_pubs[i].publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimThrusterBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
