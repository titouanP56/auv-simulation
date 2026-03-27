#!/usr/bin/env python3
"""
wall_following_node.py
======================
AUV Net Inspection — Circular Wall Following

Implements autonomous navigation along a deformable aquaculture net using:
  - Chou et al., 2023 : Hough Transform wall extraction + d_f/d_b metrics
  - Ghorbani, 2021    : B-Spline smoothing for trajectory generation

State machine:
    WAITING  →  FOLLOWING  →  COMPLETED

Publishes MPC setpoints on /cmd_setpoint (PoseStamped in local frame).
"""

import math
from collections import deque
from enum import Enum, auto

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import median_filter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to (roll, pitch, yaw)."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert (roll, pitch, yaw) to quaternion [x, y, z, w]."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def angle_wrap(angle):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ═══════════════════════════════════════════════════════════════════════════════
# State Enum
# ═══════════════════════════════════════════════════════════════════════════════

class FollowState(Enum):
    WAITING = auto()
    FOLLOWING = auto()
    COMPLETED = auto()


# ═══════════════════════════════════════════════════════════════════════════════
# Wall Extractor (Chou et al., 2023 — Hough‑based perception)
# ═══════════════════════════════════════════════════════════════════════════════

class WallExtractor:
    """Extract the dominant wall segment from a LaserScan using Hough‑like
    analysis.  A median filter cleans up acoustic noise first.

    Instead of depending on OpenCV (cv2.HoughLinesP), we implement a
    lightweight 2‑D Hough accumulator that works directly on the
    polar sonar ranges converted to Cartesian.  This avoids the need
    for an image rasterisation step.
    """

    def __init__(
        self,
        median_kernel: int = 5,
        hough_rho_res: float = 0.1,
        hough_theta_res: float = math.radians(2.0),
        hough_threshold: int = 10,
        max_range: float = 30.0,
    ):
        self.median_kernel = median_kernel
        self.rho_res = hough_rho_res
        self.theta_res = hough_theta_res
        self.threshold = hough_threshold
        self.max_range = max_range

        # Precompute Hough angle bins
        self._theta_bins = np.arange(-math.pi / 2, math.pi / 2, self.theta_res)
        self._cos_t = np.cos(self._theta_bins)
        self._sin_t = np.sin(self._theta_bins)

    def extract(self, scan: LaserScan):
        """Return (wall_distance, wall_angle) in robot frame, or None."""
        ranges = np.array(scan.ranges, dtype=np.float64)

        # --- Median filter for acoustic noise removal ---
        ranges = median_filter(ranges, size=self.median_kernel)

        # Convert to Cartesian (robot body frame: x-forward, y-left)
        angles = np.linspace(
            scan.angle_min, scan.angle_max, len(ranges), endpoint=True
        )
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < self.max_range)
        if valid.sum() < 3:
            return None

        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])

        # --- Hough accumulator ---
        rhos = xs[:, None] * self._cos_t[None, :] + ys[:, None] * self._sin_t[None, :]
        rho_max = float(np.max(np.abs(rhos))) + self.rho_res
        n_rho_bins = int(2.0 * rho_max / self.rho_res) + 1

        rho_idx = ((rhos + rho_max) / self.rho_res).astype(int)
        rho_idx = np.clip(rho_idx, 0, n_rho_bins - 1)

        accumulator = np.zeros((n_rho_bins, len(self._theta_bins)), dtype=int)
        for pt_i in range(len(xs)):
            for t_j in range(len(self._theta_bins)):
                accumulator[rho_idx[pt_i, t_j], t_j] += 1

        peak = np.unravel_index(np.argmax(accumulator), accumulator.shape)
        if accumulator[peak] < self.threshold:
            return None

        best_rho = peak[0] * self.rho_res - rho_max
        best_theta = self._theta_bins[peak[1]]

        # Distance from origin (robot) to detected line
        wall_distance = abs(best_rho)
        # Angle of the line normal w.r.t. robot x-axis
        wall_angle = best_theta if best_rho >= 0 else best_theta + math.pi
        wall_angle = angle_wrap(wall_angle)

        return wall_distance, wall_angle, xs, ys


# ═══════════════════════════════════════════════════════════════════════════════
# Wall Follower (Chou et al., 2023 — d_f / d_b metrics)
# ═══════════════════════════════════════════════════════════════════════════════

class WallFollower:
    """Compute tracking errors using front / back corner distances d_f, d_b."""

    def __init__(
        self,
        target_distance: float = 1.5,
        robot_half_length: float = 0.23,
        robot_half_width: float = 0.20,
    ):
        self.target_distance = target_distance
        self.L = robot_half_length
        self.W = robot_half_width

    def compute_errors(self, wall_distance: float, wall_angle: float):
        """Return (distance_error, angle_error, d_f, d_b).

        d_f : perpendicular distance from the front-right corner to the wall
        d_b : perpendicular distance from the back-right corner to the wall
        """
        cos_a = math.cos(wall_angle)
        sin_a = math.sin(wall_angle)

        # Front-right and back-right corners in body frame
        fr_x, fr_y = self.L, -self.W
        br_x, br_y = -self.L, -self.W

        # Signed distance from each corner to the Hough line
        # (line eq: x*cos(theta) + y*sin(theta) = rho  =>  d = rho - projection)
        d_f = wall_distance - (fr_x * cos_a + fr_y * sin_a)
        d_b = wall_distance - (br_x * cos_a + br_y * sin_a)

        # Ensure positive convention
        d_f = abs(d_f)
        d_b = abs(d_b)

        # Error metrics
        distance_error = (d_f + d_b) / 2.0 - self.target_distance
        angle_error = math.atan2(d_f - d_b, 2.0 * self.L)

        return distance_error, angle_error, d_f, d_b


# ═══════════════════════════════════════════════════════════════════════════════
# Trajectory Generator (Ghorbani, 2021 — B‑Spline smoothing)
# ═══════════════════════════════════════════════════════════════════════════════

class TrajectoryGenerator:
    """Fit a cubic B‑spline to recent wall detections and generate offset
    waypoints for the MPC controller."""

    def __init__(
        self,
        buffer_size: int = 20,
        target_distance: float = 1.5,
        num_eval_points: int = 50,
    ):
        self.buffer_size = buffer_size
        self.target_distance = target_distance
        self.num_eval_points = num_eval_points
        self._wall_points = deque(maxlen=buffer_size)

    def add_wall_point(self, x_global: float, y_global: float):
        """Add a detected wall point in global (odom) coordinates."""
        self._wall_points.append((x_global, y_global))

    def generate_waypoint(self):
        """Fit a spline to buffered wall points and return the next offset
        waypoint (x, y) in global frame, or None if insufficient data."""
        if len(self._wall_points) < 6:
            return None

        pts = np.array(self._wall_points)
        x_pts, y_pts = pts[:, 0], pts[:, 1]

        try:
            tck, _ = splprep([x_pts, y_pts], s=0.5, k=3)
        except (ValueError, TypeError):
            return None

        # Evaluate the spline and its derivatives at the most recent parameter
        u_eval = np.linspace(0.0, 1.0, self.num_eval_points)
        spline_pts = np.array(splev(u_eval, tck))            # (2, N)
        spline_deriv = np.array(splev(u_eval, tck, der=1))    # (2, N)

        # Use the last point as the "current" wall point
        wx, wy = spline_pts[0, -1], spline_pts[1, -1]
        dx, dy = spline_deriv[0, -1], spline_deriv[1, -1]

        # Normal pointing inward (towards robot / centre)
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        # Normal perpendicular to tangent, pointing "left" of the curve
        nx, ny = -dy / norm, dx / norm

        # Offset waypoint
        wp_x = wx + nx * self.target_distance
        wp_y = wy + ny * self.target_distance

        return wp_x, wp_y

    def clear(self):
        self._wall_points.clear()


# ═══════════════════════════════════════════════════════════════════════════════
# Main ROS 2 Node
# ═══════════════════════════════════════════════════════════════════════════════

class WallFollowingNode(Node):
    """Full wall-following node combining perception, control, and trajectory
    generation for circular inspection of a deformable aquaculture net."""

    def __init__(self):
        super().__init__('wall_following_node')

        # ── Declare ROS 2 parameters ──────────────────────────────────────────
        self.declare_parameter('enabled', True)
        self.declare_parameter('net_radius', 25.0)
        self.declare_parameter('target_distance', 1.5)
        self.declare_parameter('nominal_velocity', 0.3)
        self.declare_parameter('lookahead_time', 2.0)
        self.declare_parameter('gain_distance', 0.8)
        self.declare_parameter('gain_angle', 1.0)
        self.declare_parameter('hough_threshold', 10)
        self.declare_parameter('spline_buffer_size', 20)
        self.declare_parameter('robot_half_length', 0.23)
        self.declare_parameter('robot_half_width', 0.20)
        self.declare_parameter('completion_angle_tol', 0.15)

        self._load_parameters()

        # ── Sub-modules ───────────────────────────────────────────────────────
        self.wall_extractor = WallExtractor(
            hough_threshold=self.hough_threshold,
        )
        self.wall_follower = WallFollower(
            target_distance=self.target_distance,
            robot_half_length=self.robot_half_length,
            robot_half_width=self.robot_half_width,
        )
        self.trajectory_gen = TrajectoryGenerator(
            buffer_size=self.spline_buffer_size,
            target_distance=self.target_distance,
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.state = FollowState.WAITING
        self.origin_recorded = False
        self.origin_pose = None        # [x_g, y_g, z_g, yaw_g]
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.current_pose = None       # Odometry PoseStamped
        self.current_yaw = 0.0
        self.z_target = 0.0
        self.last_scan_time = None
        self.latest_scan = None
        self.start_angle = None        # For lap-completion detection
        self.cumulative_angle = 0.0
        self.prev_bearing = None
        self.start_transition_time = None

        # ── QoS ───────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10
        )
        self.create_subscription(
            LaserScan, '/ping360/scan', self._scan_cb, best_effort_qos
        )
        self.create_subscription(
            Bool, '/mission/phase2_done', self._phase2_done_cb, 10
        )
        self.create_subscription(
            PoseStamped, '/mission/local_origin', self._origin_cb, 10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.target_pub = self.create_publisher(PoseStamped, '/cmd_setpoint', 10)
        self.completion_pub = self.create_publisher(
            Bool, '/net/inspection_completed', 10
        )

        # ── Control loop at 10 Hz ────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f"WallFollowingNode initialised (enabled={self.enabled})."
        )

    # ── Parameter loading ────────────────────────────────────────────────────

    def _load_parameters(self):
        self.enabled = self.get_parameter('enabled').value
        self.net_radius = self.get_parameter('net_radius').value
        self.target_distance = self.get_parameter('target_distance').value
        self.nominal_velocity = self.get_parameter('nominal_velocity').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        self.gain_distance = self.get_parameter('gain_distance').value
        self.gain_angle = self.get_parameter('gain_angle').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.spline_buffer_size = self.get_parameter('spline_buffer_size').value
        self.robot_half_length = self.get_parameter('robot_half_length').value
        self.robot_half_width = self.get_parameter('robot_half_width').value
        self.completion_angle_tol = self.get_parameter('completion_angle_tol').value

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        _, _, self.current_yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _phase2_done_cb(self, msg: Bool):
        if msg.data and self.state == FollowState.WAITING:
            if self.enabled:
                self.get_logger().info(
                    "Phase 2 done → activating wall following."
                )
                self.start_transition_time = self.get_clock().now()

    def _origin_cb(self, msg: PoseStamped):
        if self.origin_recorded:
            return

        p = msg.pose.position
        _, _, yaw_origin = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        self.origin_pose = [p.x, p.y, p.z, yaw_origin]
        self.origin_recorded = True
        self.z_target = 0.0

        # Compute cage centre dynamically (same logic as reactive_wall_follower)
        yaw_to_centre = yaw_origin - math.pi
        self.circle_center_x = p.x - self.net_radius * math.cos(yaw_to_centre)
        self.circle_center_y = p.y - self.net_radius * math.sin(yaw_to_centre)

        self.get_logger().info(
            f"Origin recorded: {self.origin_pose}, "
            f"Cage centre: ({self.circle_center_x:.2f}, "
            f"{self.circle_center_y:.2f})"
        )

        # Transition to FOLLOWING if phase2 was already confirmed
        if self.start_transition_time is not None:
            self.state = FollowState.FOLLOWING
            self._init_following()

    def _init_following(self):
        """Initialise lap tracking on first entry into FOLLOWING."""
        if self.current_pose is not None:
            x_g = self.current_pose.position.x
            y_g = self.current_pose.position.y
            bearing = math.atan2(
                y_g - self.circle_center_y,
                x_g - self.circle_center_x,
            )
            self.start_angle = bearing
            self.prev_bearing = bearing
            self.cumulative_angle = 0.0
        self.get_logger().info("Wall following ACTIVE — starting lap.")

    # ── Main control loop ────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.enabled:
            return
        if self.state == FollowState.WAITING:
            # Also handle the case where origin arrives before phase2_done
            if self.origin_recorded and self.start_transition_time is not None:
                self.state = FollowState.FOLLOWING
                self._init_following()
            return
        if self.state == FollowState.COMPLETED:
            done_msg = Bool()
            done_msg.data = True
            self.completion_pub.publish(done_msg)
            return
        if self.current_pose is None or not self.origin_recorded:
            return

        # Refresh configurable parameters
        self._load_parameters()

        # Robot position in Odom frame
        x_g = self.current_pose.position.x
        y_g = self.current_pose.position.y
        psi_0 = self.origin_pose[3]

        # ── Attempt Hough perception ─────────────────────────────────────────
        use_perception = False
        wall_dist = 0.0
        wall_angle = 0.0

        now = self.get_clock().now()
        if self.last_scan_time is not None and self.latest_scan is not None:
            dt_scan = (now - self.last_scan_time).nanoseconds / 1e9
            if dt_scan < 3.0:
                result = self.wall_extractor.extract(self.latest_scan)
                if result is not None:
                    wall_dist, wall_angle, xs, ys = result
                    use_perception = True

                    # Feed wall points into spline generator (global frame)
                    cos_yaw = math.cos(self.current_yaw)
                    sin_yaw = math.sin(self.current_yaw)
                    # Closest wall point in body frame → rotate to global
                    wp_body_x = wall_dist * math.cos(wall_angle)
                    wp_body_y = wall_dist * math.sin(wall_angle)
                    wp_global_x = x_g + wp_body_x * cos_yaw - wp_body_y * sin_yaw
                    wp_global_y = y_g + wp_body_x * sin_yaw + wp_body_y * cos_yaw
                    self.trajectory_gen.add_wall_point(wp_global_x, wp_global_y)

        # ── Compute velocity vector ──────────────────────────────────────────
        # Radial direction from cage centre
        om_x = x_g - self.circle_center_x
        om_y = y_g - self.circle_center_y
        dist_to_center = math.hypot(om_x, om_y)
        if dist_to_center < 0.1:
            dist_to_center = 0.1
        u_om_x = om_x / dist_to_center
        u_om_y = om_y / dist_to_center

        # Tangent vector (CCW orbit — robot inside looking outward)
        v_t_x = -u_om_y
        v_t_y = u_om_x

        if use_perception:
            # --- Chou et al. metrics ---
            e_d, e_theta, d_f, d_b = self.wall_follower.compute_errors(
                wall_dist, wall_angle
            )
            # Normal correction: global frame via wall_angle + robot yaw
            normal_global = self.current_yaw + wall_angle
            v_n_x = self.gain_distance * e_d * math.cos(normal_global)
            v_n_y = self.gain_distance * e_d * math.sin(normal_global)

            # Angular correction applied as yaw offset
            target_yaw_g = self.current_yaw + wall_angle + math.pi / 2.0
            # Add angular correction
            target_yaw_g += self.gain_angle * e_theta

            # B-Spline waypoint (if available, override tangential direction)
            spline_wp = self.trajectory_gen.generate_waypoint()
            if spline_wp is not None:
                sp_x, sp_y = spline_wp
                dx_sp = sp_x - x_g
                dy_sp = sp_y - y_g
                sp_dist = math.hypot(dx_sp, dy_sp)
                if sp_dist > 0.01:
                    v_t_x = dx_sp / sp_dist
                    v_t_y = dy_sp / sp_dist
        else:
            # --- Fallback: geometric circle tracking ---
            tracking_radius = self.net_radius - self.target_distance
            error = dist_to_center - tracking_radius
            v_n_x = -error * u_om_x
            v_n_y = -error * u_om_y
            # Face outward (toward the net)
            target_yaw_g = math.atan2(u_om_y, u_om_x)

        # Resultant velocity vector in odom
        v_res_x = v_t_x + self.gain_distance * v_n_x
        v_res_y = v_t_y + self.gain_distance * v_n_y
        norm_res = math.hypot(v_res_x, v_res_y)
        if norm_res > 0.001:
            v_res_x = (v_res_x / norm_res) * self.nominal_velocity
            v_res_y = (v_res_y / norm_res) * self.nominal_velocity

        # Look-ahead target in odom
        target_g_x = x_g + v_res_x * self.lookahead_time
        target_g_y = y_g + v_res_y * self.lookahead_time

        # ── Transform to local frame ─────────────────────────────────────────
        tdx = target_g_x - self.origin_pose[0]
        tdy = target_g_y - self.origin_pose[1]
        target_x_L = tdx * math.cos(psi_0) + tdy * math.sin(psi_0)
        target_y_L = -tdx * math.sin(psi_0) + tdy * math.cos(psi_0)
        target_yaw_L = target_yaw_g - psi_0

        # ── Soft start (1 second ramp) ────────────────────────────────────────
        elapsed = (now - self.start_transition_time).nanoseconds / 1e9
        if elapsed < 1.0:
            target_x_L *= elapsed
            target_y_L *= elapsed

        # ── Publish PoseStamped for MPC ──────────────────────────────────────
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'local_origin'
        msg.pose.position.x = target_x_L
        msg.pose.position.y = target_y_L
        msg.pose.position.z = self.z_target

        q = quaternion_from_euler(0.0, 0.0, target_yaw_L)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.target_pub.publish(msg)

        # ── Lap completion detection ─────────────────────────────────────────
        self._check_lap_completion(x_g, y_g)

    # ── Lap completion ───────────────────────────────────────────────────────

    def _check_lap_completion(self, x_g: float, y_g: float):
        """Track cumulative bearing change around the cage centre; when it
        exceeds 2*pi the robot has completed one full orbit."""
        bearing = math.atan2(
            y_g - self.circle_center_y,
            x_g - self.circle_center_x,
        )

        if self.prev_bearing is not None:
            delta = angle_wrap(bearing - self.prev_bearing)
            self.cumulative_angle += delta

        self.prev_bearing = bearing

        if abs(self.cumulative_angle) >= 2.0 * math.pi - self.completion_angle_tol:
            self.get_logger().info(
                f"✅ Lap completed! Cumulative angle: "
                f"{math.degrees(self.cumulative_angle):.1f}°"
            )
            self.state = FollowState.COMPLETED


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
