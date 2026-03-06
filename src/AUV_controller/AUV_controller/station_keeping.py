"""
station_keeping.py — Simple PD station-keeping controller for the BlueROV2.

This node implements a Proportional-Derivative (PD) controller to keep the AUV 
at a fixed position (x, y, z) and orientation (yaw). It serves as a robust baseline 
controller or a fail-safe mechanism against drifts and perturbations.

It subscribes to the robust EKF output (`/odometry/filtered`) for localization
and publishes thruster commands (`/cmd_vel_1` to `/cmd_vel_8`) using a pseudo-inverse 
allocation matrix.

Usage:
    ros2 run AUV_controller station_keeping
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import math

# ── 1. Thruster Configuration ─────────────────────────────────────────────────
# Thruster force to command conversion coefficients (from URDF)
THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]
RHO = 997.0 # Water density (kg/m^3)

# ── 2. Thruster Allocation Matrix (TAM) ───────────────────────────────────────
# Describes the geometric placement of thrusters to convert individual 
# forces into global body wrenches (forces/torques).
SIN45  = 0.7071
LEVER  = 0.1697
Z_ARM  = 0.1

TAM = np.array([
    [ SIN45,  SIN45, -SIN45, -SIN45,  0.0,  0.0,  0.0,  0.0],  # Fx (surge)
    [ SIN45, -SIN45,  SIN45, -SIN45,  0.0,  0.0,  0.0,  0.0],  # Fy (sway)
    [ 0.0,    0.0,    0.0,    0.0,   -1.0,  1.0,  1.0, -1.0],  # Fz (heave)
    [ 0.0,    0.0,    0.0,    0.0,    0.218,0.218,0.218,0.218],  # Mx (roll)  ignored by pseudo-inverse
    [ 0.0,    0.0,    0.0,    0.0,    0.12,-0.12, 0.12,-0.12],  # My (pitch) ignored by pseudo-inverse
    [ LEVER, -LEVER, -LEVER,  LEVER,  0.0,  0.0,  0.0,  0.0],  # Mz (yaw)
])

# Moore-Penrose Pseudo-inverse: 
# Used to calculate the optimal individual thruster forces [t1..t8] 
# required to achieve a desired body wrench [Fx, Fy, Fz, Mx, My, Mz]
TAM_PINV = np.linalg.pinv(TAM)

# ── 3. Controller Tuning (PD Gains) ───────────────────────────────────────────
# Proportional (KP): reacts to the distance to the target.
# Derivative (KD): dampens the velocity to prevent overshoot/oscillation.
KP_XY   = 8.0    # Position XY (Surge/Sway)
KD_XY   = 4.0    # Velocity XY damping
KP_Z    = 15.0   # Depth (Heave) - Needs to be strong to counteract buoyancy changes
KI_Z    = 3.0    # Depth integral gain for steady-state error (buoyancy mismatch)
KD_Z    = 6.0    # Depth velocity damping
KP_YAW  = 5.0    # Yaw angle
KD_YAW  = 2.0    # Yaw rate damping

# Actuator Limits (to prevent violent reactions)
MAX_FORCE   = 30.0   # Maximum commanded global force per axis (N)
MAX_TORQUE  = 10.0   # Maximum commanded global torque (N·m)
MAX_THRUST  =  5.0   # Maximum individual thruster force (N)

# Gravity vs Buoyancy compensation
BUOYANCY_NET = 2.0   # Net upward force that the robot naturally experiences (N)

# ── 4. Target Setpoint ────────────────────────────────────────────────────────
TARGET_X   = 0.0
TARGET_Y   = 0.0
TARGET_Z   = -1.0
TARGET_YAW = 0.0


class StationKeepingNode(Node):
    """ROS 2 Node that executes the PD control loop for station keeping."""
    
    def __init__(self):
        super().__init__('station_keeping')

        # Publishers for 8 thrusters
        self.pubs = [
            self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            for i in range(1, 9)
        ]

        # Subscribe to fused localization data
        self.sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, 10
        )

        self.state = None
        self.ez_int = 0.0
        
        # Run control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)  
        self.get_logger().info(
            f'Station-Keeping started — target: ({TARGET_X}, {TARGET_Y}, {TARGET_Z}), yaw={TARGET_YAW}°'
        )

    def odom_cb(self, msg: Odometry):
        """
        Callback to update the robot's current pose and twist.
        Pose is in the world frame ('odom'), Twist is in the body frame ('base_link').
        """
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.twist.linear   # Body frame velocities
        w = msg.twist.twist.angular  # Body frame angular velocities

        # Convert orientation quaternion to yaw angle
        siny_cosp = 2 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1 - 2 * (o.y**2 + o.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.state = {
            'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw,
            'vx': v.x, 'vy': v.y, 'vz': v.z, 'vyaw': w.z,
        }

    def control_loop(self):
        """
        Calculates position errors, applies the PD control law to find desired body
        wrenches, allocates them to thrusters, and publishes the commands.
        """
        if self.state is None:
            return  # Wait for first odometry message

        s = self.state
        yaw = s['yaw']

        # ── 1. Position errors (World frame) ──────────────────────────────────
        ex_world = TARGET_X - s['x']
        ey_world = TARGET_Y - s['y']
        ez       = TARGET_Z - s['z']

        # Integral term for Z to counteract mismatch in exact buoyancy
        self.ez_int += ez * 0.05
        # Anti-windup
        self.ez_int = np.clip(self.ez_int, -10.0, 10.0)

        # ── 2. Frame Transformation (World → Body) ────────────────────────────
        # To command the robot, the error vector must be rotated into the robot's 
        # local perspective (body frame) based on its current heading (yaw).
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ex_body =  cos_y * ex_world + sin_y * ey_world
        ey_body = -sin_y * ex_world + cos_y * ey_world

        # ── 3. Velocity errors (Body frame) ───────────────────────────────────
        # Target velocity is 0 to maintain station.
        # Note: EKF twist is already in body frame (because twist_body: true in config).
        dvx = -s['vx']
        dvy = -s['vy']
        dvz = -s['vz']
        dyaw = -s['vyaw']

        # ── 4. Yaw error handling ─────────────────────────────────────────────
        # Compute shortest rotational path, wrapping error between [-π, π]
        eyaw = TARGET_YAW - yaw
        eyaw = math.atan2(math.sin(eyaw), math.cos(eyaw))

        # ── 5. PD Control Law Calculation ─────────────────────────────────────
        # F = Kp * position_error + Kd * velocity_error
        # Fz includes compensation for the net positive buoyancy and integral error
        Fx = np.clip(KP_XY  * ex_body + KD_XY  * dvx,  -MAX_FORCE,  MAX_FORCE)
        Fy = np.clip(KP_XY  * ey_body + KD_XY  * dvy,  -MAX_FORCE,  MAX_FORCE)
        Fz = np.clip(KP_Z   * ez      + KI_Z   * self.ez_int + KD_Z   * dvz - BUOYANCY_NET,
                     -MAX_FORCE, MAX_FORCE)
        Mz = np.clip(KP_YAW * eyaw    + KD_YAW * dyaw, -MAX_TORQUE, MAX_TORQUE)

        # We assume roll (Mx) and pitch (My) commands are 0 for this simple controller
        tau = np.array([Fx, Fy, Fz, 0.0, 0.0, Mz])

        # ── 6. Thruster Allocation ────────────────────────────────────────────
        # Map the desired 6-DOF wrench to 8 individual thruster forces
        thrusts = TAM_PINV @ tau
        thrusts = np.clip(thrusts, -MAX_THRUST, MAX_THRUST)

        # ── 7. Publish Commands ───────────────────────────────────────────────
        for i, (thrust, coeff) in enumerate(zip(thrusts, THRUST_COEFFS)):
            msg = Float64()
            # Apply coefficient sign to compensate for Gazebo propeller rotation setup
            msg.data = float(thrust) * math.copysign(1.0, coeff)
            self.pubs[i].publish(msg)

        # ── 8. Telemetry Logging (runs at 0.5 Hz) ─────────────────────────────
        if not hasattr(self, '_tick'):
            self._tick = 0
        self._tick += 1
        if self._tick % 40 == 0:
            dist = math.sqrt(s['x']**2 + s['y']**2 + s['z']**2)
            self.get_logger().info(
                f'pos=({s["x"]:.2f},{s["y"]:.2f},{s["z"]:.2f}) '
                f'yaw={math.degrees(yaw):.1f}°  dist={dist:.2f}m  '
                f'F=({Fx:.1f},{Fy:.1f},{Fz:.1f}) Mz={Mz:.1f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StationKeepingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
