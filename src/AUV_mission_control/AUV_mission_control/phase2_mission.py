"""
phase2_mission.py
=================
AUV Net Inspection — Phase 2: Descent and Edge Finding

State machine:
    DESCENDING  →  SCANNING  →  ALIGNING  →  APPROACHING  →  STANDOFF

Control strategy (Option A — direct P-controllers on thrusters):
  - Depth control  : vertical thrusters (T5‑T8) driven by a P-controller on z error.
  - Yaw control    : lateral thruster differential (T1‑T4) driven by P on yaw error.
  - Surge control  : forward thrusters (T1+T4 top pair) driven by P on distance error.

Topics consumed:
  /odom                  (nav_msgs/Odometry)         — position + orientation
  /ping360/scan          (sensor_msgs/LaserScan)     — 360° sonar (edge detection)
  /sonoptix/points       (sensor_msgs/PointCloud2)   — forward multibeam sonar

Topics published:
  /cmd_vel_1 … /cmd_vel_8  (std_msgs/Float64)        — individual thruster commands
  /mission/phase         (std_msgs/String)            — current state name (for monitoring)
  /mission/phase2_done   (std_msgs/Bool)              — True when STANDOFF reached
"""

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2


# ── Constants ─────────────────────────────────────────────────────────────────

TARGET_DEPTH      = -2.0    # [m]  desired diving depth (NED: negative = down)
DEPTH_TOLERANCE   = 0.20    # [m]  we're "at depth" when |z_error| < this
                            #       (widened: P-only ctrl has steady-state error ~0.13 m)
DEPTH_HOLD_TIME   = 2.0     # [s]  must stay within tolerance before transitioning

YAW_TOLERANCE     = math.radians(5.0)  # [rad]  aligned when |yaw_error| < this
YAW_HOLD_TIME     = 1.0                # [s]

STANDOFF_DIST     = 1.5     # [m]  desired distance to net wall
APPROACH_TOL      = 0.10    # [m]  we've reached standoff when within this margin

# P-gain for depth hold (vertical thrusters T5‑T8)
KP_DEPTH  = 12.0   # [N/m] 

# Constant downward force to counteract the net positive buoyancy of the robot
# (From station_keeping.py BUOYANCY_NET = 2.0, but we use a bit more for margin)
BUOYANCY_COMPENSATION = 3.0  # [N] applied continuously downwards
# P-gain for yaw alignment (horizontal thruster differential)
KP_YAW    = 6.0    # [N·m/rad]
KD_YAW    = 4.0    # [N·m·s/rad] D-gain to dampen oscillation
# P-gain for forward approach
KP_SURGE  = 4.0    # [N/m]

MAX_DEPTH_CMD   = 20.0   # [N]   clamp on vertical thrust per thruster
MAX_YAW_CMD     = 15.0   # [N]   clamp on yaw differential per thruster
MAX_SURGE_CMD   = 15.0   # [N]   clamp on surge thrust per thruster

# Ping360 detection: ignore beams that report max-range (likely no return)
PING360_IGNORE_THRESHOLD = 0.95   # fraction of max_range

# Sonoptix: only look at beams within this horizontal half-angle of boresight
SONOPTIX_BORESIGHT_HALF_ANGLE = math.radians(20.0)  # ±20°

CONTROL_RATE_HZ = 10.0   # [Hz]  main control loop rate
# ── Thruster Allocation (from station_keeping.py — proven with Gazebo) ─────────
# Thrust coefficient sign per thruster (from URDF).  The sign must be applied
# to the final command so Gazebo's propeller plugin produces force in the
# correct direction.
THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]

SIN45 = 0.7071
LEVER = 0.1697   # moment arm of horizontal thrusters for yaw

# Rows = [Fx, Fy, Fz, Mx, My, Mz]; Cols = [T1..T8]
TAM = np.array([
    [ SIN45,  SIN45, -SIN45, -SIN45,  0.0,   0.0,   0.0,   0.0 ],  # Fx surge
    [ SIN45, -SIN45,  SIN45, -SIN45,  0.0,   0.0,   0.0,   0.0 ],  # Fy sway
    [ 0.0,    0.0,    0.0,    0.0,   -1.0,   1.0,   1.0,  -1.0 ],  # Fz heave
    [ 0.0,    0.0,    0.0,    0.0,    0.218, 0.218, 0.218, 0.218], # Mx roll
    [ 0.0,    0.0,    0.0,    0.0,    0.12, -0.12,  0.12, -0.12 ], # My pitch
    [ LEVER, -LEVER, -LEVER,  LEVER,  0.0,   0.0,   0.0,   0.0 ],  # Mz yaw
])
TAM_PINV = np.linalg.pinv(TAM)

MAX_INDIVIDUAL_THRUST = 20.0  # [N] per thruster

# State names
class State:
    DESCENDING  = "DESCENDING"
    SCANNING    = "SCANNING"
    ALIGNING    = "ALIGNING"
    APPROACHING = "APPROACHING"
    STANDOFF    = "STANDOFF"


# ── Helper: extract yaw from Odometry message ─────────────────────────────────

def _yaw_from_odom(odom: Odometry) -> float:
    """Extract yaw (heading) in radians from an Odometry message."""
    q = odom.pose.pose.orientation
    # Standard quaternion → yaw formula
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a - b, result in [-π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ── Helper: minimum range from Sonoptix PointCloud2 ──────────────────────────

def _min_sonoptix_range(msg: PointCloud2) -> float | None:
    """
    Extract the minimum range from the central beam cluster of the Sonoptix.
    Returns None if no valid points found.

    PointCloud2 (XYZ) — we look at points whose horizontal angle |atan2(y,x)|
    is within SONOPTIX_BORESIGHT_HALF_ANGLE and compute the Euclidean range.
    """
    # Parse fields to find x, y, z offsets
    field_map = {f.name: f for f in msg.fields}
    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return None

    x_off = field_map['x'].offset
    y_off = field_map['y'].offset
    z_off = field_map['z'].offset
    point_step = msg.point_step
    data = msg.data

    min_range = float('inf')
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
        if horiz_angle > SONOPTIX_BORESIGHT_HALF_ANGLE:
            continue

        r = math.sqrt(px * px + py * py + pz * pz)
        if r > 0.01:
            min_range = min(min_range, r)

    return min_range if math.isfinite(min_range) else None


# ── Main Node ─────────────────────────────────────────────────────────────────

class Phase2MissionNode(Node):
    """
    Phase 2: Descent and Edge Finding.

    Transitions: DESCENDING → SCANNING → ALIGNING → APPROACHING → STANDOFF
    """

    def __init__(self):
        super().__init__('phase2_mission')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.ping360_sub = self.create_subscription(
            LaserScan, '/ping360/scan', self._ping360_cb, best_effort_qos)
        self.sonoptix_sub = self.create_subscription(
            PointCloud2, '/sonoptix/points', self._sonoptix_cb, best_effort_qos)

        # ── Publishers ───────────────────────────────────────────────────────
        self._thrust_pubs = [
            self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            for i in range(1, 9)
        ]
        self.phase_pub  = self.create_publisher(String, '/mission/phase', 10)
        self.done_pub   = self.create_publisher(Bool,   '/mission/phase2_done', 10)

        # ── State ────────────────────────────────────────────────────────────
        self.state: str = State.DESCENDING

        self.current_z:    float = 0.0
        self.current_yaw:  float = 0.0
        self.current_vyaw: float = 0.0  # angular velocity Z

        self.target_yaw: float = 0.0   # bearing to nearest net edge (set in SCANNING)
        self.sonoptix_range: float | None = None

        self._depth_ok_since:  float | None = None   # timestamp when depth first OK
        self._yaw_ok_since:    float | None = None   # timestamp when yaw first OK

        self._have_odom:    bool = False
        self._have_scan:    bool = False
        self._have_points:  bool = False

        # ── Control timer ────────────────────────────────────────────────────
        self.timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ, self._control_loop)

        self.get_logger().info("Phase2MissionNode started → state: DESCENDING")

    # ── Subscribers callbacks ─────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self._have_odom   = True

    def _ping360_cb(self, msg: LaserScan):
        """Find the minimum-range beam in the Ping360 scan (= nearest net wall)."""
        if self.state != State.SCANNING:
            return  # only process during SCANNING to save CPU

        max_r = msg.range_max
        min_r  = float('inf')
        min_idx = 0

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r >= max_r * PING360_IGNORE_THRESHOLD:
                continue  # no return at this angle
            if r < min_r:
                min_r  = r
                min_idx = i

        if math.isinf(min_r):
            self.get_logger().warn("Ping360: no valid return found — retrying scan")
            return

        # Direction to nearest edge in the sensor (= robot) frame
        angle = msg.angle_min + min_idx * msg.angle_increment
        self.target_yaw = self.current_yaw + angle  # convert to world frame

        self.get_logger().info(
            f"[SCANNING] Nearest wall: {min_r:.2f} m at sensor angle "
            f"{math.degrees(angle):.1f}° → target world yaw "
            f"{math.degrees(self.target_yaw):.1f}°"
        )
        self._have_scan = True

    def _sonoptix_cb(self, msg: PointCloud2):
        if self.state == State.APPROACHING or self.state == State.STANDOFF:
            self.sonoptix_range = _min_sonoptix_range(msg)
            self._have_points = True

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._have_odom:
            return  # wait for first odometry

        self._publish_state()

        if self.state == State.DESCENDING:
            self._do_descending()
        elif self.state == State.SCANNING:
            self._do_scanning()
        elif self.state == State.ALIGNING:
            self._do_aligning()
        elif self.state == State.APPROACHING:
            self._do_approaching()
        elif self.state == State.STANDOFF:
            self._do_standoff()

    # ── State implementations ─────────────────────────────────────────────────

    def _do_descending(self):
        """Dive to TARGET_DEPTH using vertical thrusters (T5‑T8)."""
        depth_error = TARGET_DEPTH - self.current_z   # negative = need to go down
        # P-control + constant feedforward to overcome positive buoyancy
        depth_cmd   = (KP_DEPTH * depth_error) - BUOYANCY_COMPENSATION
        depth_cmd   = max(-MAX_DEPTH_CMD, min(MAX_DEPTH_CMD, depth_cmd))

        # T5‑T8 are vertical thrusters (indices 4‑7 in 0-based)
        # Sign convention: positive cmd → upward force on these thrusters
        # (thrust_coeff alternates; we send raw command which the URDF's
        # thrust_coefficient sign-flips internally)
        self._set_Fz(depth_cmd)
        self._set_Mz(0.0)
        self._set_Fx(0.0)

        now  = self.get_clock().now().nanoseconds * 1e-9
        diff = abs(depth_error)

        if diff < DEPTH_TOLERANCE:
            if self._depth_ok_since is None:
                self._depth_ok_since = now
            elif (now - self._depth_ok_since) >= DEPTH_HOLD_TIME:
                self.get_logger().info(
                    f"[DESCENDING → SCANNING] Depth stable at {self.current_z:.2f} m")
                self.state = State.SCANNING
                self._depth_ok_since = None
        else:
            self._depth_ok_since = None

    def _do_scanning(self):
        """Hold depth, zero yaw rate; waiting for Ping360 callback to set target_yaw."""
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = (KP_DEPTH * depth_error) - BUOYANCY_COMPENSATION
        depth_cmd   = max(-MAX_DEPTH_CMD, min(MAX_DEPTH_CMD, depth_cmd))
        self._set_Fz(depth_cmd)
        self._set_Mz(0.0)
        self._set_Fx(0.0)

        if self._have_scan:
            self.get_logger().info(
                f"[SCANNING → ALIGNING] Target yaw: "
                f"{math.degrees(self.target_yaw):.1f}°")
            self.state = State.ALIGNING
            self._have_scan = False

    def _do_aligning(self):
        """Rotate to face the nearest net wall using horizontal thrusters."""
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = (KP_DEPTH * depth_error) - BUOYANCY_COMPENSATION
        depth_cmd   = max(-MAX_DEPTH_CMD, min(MAX_DEPTH_CMD, depth_cmd))
        self._set_Fz(depth_cmd)
        self._set_Fx(0.0)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        pd_cmd    = (KP_YAW * yaw_error) - (KD_YAW * self.current_vyaw)
        mz_cmd    = max(-MAX_YAW_CMD, min(MAX_YAW_CMD, pd_cmd))
        self._set_Mz(mz_cmd)

        now  = self.get_clock().now().nanoseconds * 1e-9

        if abs(yaw_error) < YAW_TOLERANCE:
            if self._yaw_ok_since is None:
                self._yaw_ok_since = now
            elif (now - self._yaw_ok_since) >= YAW_HOLD_TIME:
                self.get_logger().info(
                    f"[ALIGNING → APPROACHING] Yaw error: {math.degrees(yaw_error):.2f}°")
                self.state = State.APPROACHING
                self._yaw_ok_since = None
        else:
            self._yaw_ok_since = None

    def _do_approaching(self):
        """Drive forward until Sonoptix reads ≤ STANDOFF_DIST."""
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = (KP_DEPTH * depth_error) - BUOYANCY_COMPENSATION
        depth_cmd   = max(-MAX_DEPTH_CMD, min(MAX_DEPTH_CMD, depth_cmd))
        self._set_Fz(depth_cmd)

        # Keep heading locked to target_yaw (PD control)
        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        pd_cmd    = (KP_YAW * yaw_error) - (KD_YAW * self.current_vyaw)
        mz_cmd    = max(-MAX_YAW_CMD, min(MAX_YAW_CMD, pd_cmd))
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is None:
            # No sonar return yet — move forward cautiously
            self._set_Fx(MAX_SURGE_CMD * 0.3)
            return

        dist_error = self.sonoptix_range - STANDOFF_DIST
        surge_cmd  = max(0.0, min(MAX_SURGE_CMD, KP_SURGE * dist_error))
        self._set_Fx(surge_cmd)

        self.get_logger().info(
            f"[APPROACHING] Sonoptix range: {self.sonoptix_range:.2f} m "
            f"(target {STANDOFF_DIST:.1f} m, surge={surge_cmd:.1f} N)",
            throttle_duration_sec=1.0
        )

        if (self.sonoptix_range - STANDOFF_DIST) <= APPROACH_TOL:
            self.get_logger().info(
                f"[APPROACHING → STANDOFF] Reached {self.sonoptix_range:.2f} m standoff!")
            self.state = State.STANDOFF

    def _do_standoff(self):
        """Hold position at standoff; publish mission done."""
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = (KP_DEPTH * depth_error) - BUOYANCY_COMPENSATION
        depth_cmd   = max(-MAX_DEPTH_CMD, min(MAX_DEPTH_CMD, depth_cmd))
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        pd_cmd    = (KP_YAW * yaw_error) - (KD_YAW * self.current_vyaw)
        mz_cmd    = max(-MAX_YAW_CMD, min(MAX_YAW_CMD, pd_cmd))
        self._set_Mz(mz_cmd)
        self._set_Fx(0.0)

        # Publish done flag every cycle (idempotent)
        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

    def _set_Fz(self, fz: float):
        """Set desired heave force (+ = up, − = down in body/world Z)."""
        self._cmd_Fz = fz

    def _set_Mz(self, mz: float):
        """Set desired yaw torque (+ = CCW from above)."""
        self._cmd_Mz = mz

    def _set_Fx(self, fx: float):
        """Set desired surge force (+ = forward)."""
        self._cmd_Fx = fx

    def _publish_state(self):
        """
        Convert stored body-frame wrench [Fx, 0, Fz, 0, 0, Mz] into 8 thruster
        forces via TAM pseudo-inverse, apply thrust_coefficient sign correction
        (required by Gazebo’s Thruster plugin), clamp, and publish.
        """
        Fx = getattr(self, '_cmd_Fx', 0.0)
        Fz = getattr(self, '_cmd_Fz', 0.0)
        Mz = getattr(self, '_cmd_Mz', 0.0)

        tau     = np.array([Fx, 0.0, Fz, 0.0, 0.0, Mz])
        thrusts = TAM_PINV @ tau
        thrusts = np.clip(thrusts, -MAX_INDIVIDUAL_THRUST, MAX_INDIVIDUAL_THRUST)

        for i, (thrust, coeff) in enumerate(zip(thrusts, THRUST_COEFFS)):
            msg = Float64()
            # thrust_coefficient sign correction — required so Gazebo’s propeller
            # spins in the correct direction for the intended force.
            msg.data = float(thrust) * math.copysign(1.0, coeff)
            self._thrust_pubs[i].publish(msg)

        # Reset wrench for next cycle
        self._cmd_Fx = 0.0
        self._cmd_Fz = 0.0
        self._cmd_Mz = 0.0

        # Publish current state name
        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Phase2MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
