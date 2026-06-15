#!/usr/bin/env python3
"""
phase3_inspection_big_net.py
============================
ROS 2 guidance node for Phase 3 (Inspection) of the AUV mission on the BIG net.

This file is the big-net variant of phase3_inspection.py.  The behaviour is
identical; only the tuning constants and the final depth limit differ.

Key differences vs. phase3_inspection.py (small net)
------------------------------------------------------
  KP_DEPTH              : 20.0   (vs  10.0)  — stiffer depth hold for large net
  KD_DEPTH              : 20.0   (vs   0.2)
  KP_DIST               : 12.0   (vs   4.0)  — faster standoff correction
  KD_DIST               :  1.0   (vs   0.5)
  KP_VEL_SWAY           : 40.0   (vs  15.0)  — stronger lateral push
  KP_YAW                : 10.0   (vs   5.0)
  KD_YAW                :  3.0   (vs   1.0)
  MAX_YAW_CMD           : 20.0   (vs  10.0)
  FINAL_DEPTH_LIMIT     : -29.5  (vs  -6.0)  — big net goes down to ~30 m
  Default initial radius: 25.0 m (vs   5.0 m)

Perception architecture
-----------------------
  Input  : /sonoptix/perception       (geometry_msgs/PoseStamped)
             pose.position.x  = orthogonal distance to the net plane [m]
             pose.orientation = quaternion encoding (pitch, yaw) of the normal
           /sonoptix/perception_valid  (std_msgs/Bool)
             True when RANSAC converged with enough inliers
  Output : /auv/command_wrench        (geometry_msgs/Wrench)
           /mission/phase3_done       (std_msgs/Bool)
           /mission/phase             (std_msgs/String)
           /phase3/*                  diagnostic topics

All sonar processing (RANSAC plane fitting, spatial culling, spike rejection)
is delegated to the auv_perception/sonoptix_perception node, which publishes
the already-processed PoseStamped on /sonoptix/perception.

Author  : titou
Package : AUV_guidance
"""

import math
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Wrench
import tf2_ros


# ── Helper: Euler angles from quaternion ───────────────────────────────────────

def euler_from_quaternion(quaternion):
    """Convert a quaternion [x, y, z, w] into (roll, pitch, yaw) Euler angles."""
    x, y, z, w = quaternion
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


# ── Tuning constants (big-net specific) ───────────────────────────────────────

TARGET_DEPTH          = -2.0    # [m]  initial depth
STANDOFF_DIST         = 1.5     # [m]  desired distance from the net surface
CONTROL_RATE_HZ       = 20.0

# PID gains — tuned for stability and reactivity on the big net simulation
KP_DEPTH              = 20.0
KI_DEPTH              =  0.2
KD_DEPTH              = 20.0
BUOYANCY_COMP         =  3.0    # [N]  static buoyancy offset

KP_DIST               = 12.0
KI_DIST               =  0.2
KD_DIST               =  1.0

KP_VEL_SWAY           = 40.0
KI_VEL_SWAY           =  2.0
KD_VEL_SWAY           =  0.5

KP_YAW                = 10.0
KI_YAW                =  0.02
KD_YAW                =  3.0

KP_PITCH              = 10.0
KI_PITCH              =  0.2
KD_PITCH              =  2.0

MAX_DEPTH_CMD         = 15.0   # [N]
MAX_DIST_CMD          = 15.0   # [N]
MAX_YAW_CMD           = 20.0   # [N·m]   ← larger than small-net variant
MAX_INDIVIDUAL_THRUST = 40.0   # [N]  per-thruster saturation

MZ_RATE_LIMIT         =  3.0   # [N·m/step]  max Mz change per control cycle

ORBIT_DIRECTION       =  1     # +1 = counter-clockwise, -1 = clockwise

# Lap / wall-loss parameters
LAP_YAW_THRESHOLD     = 2.0 * math.pi
LAP_START_DELAY       = 2.0             # [s] grace period before lap tracking
LOST_WALL_TIMEOUT     = 2.0             # [s] sonar silence before LOST_WALL
LOST_WALL_GRACE_S     = 2.0             # [s] grace period at orbit start
RECOVERY_YAW_CMD      = 4.0             # [N·m] recovery yaw torque

DEPTH_STEP            = 0.5             # [m]  depth decrement per lap
FINAL_DEPTH_LIMIT     = -29.5           # [m]  ← big net goes down to ~30 m


# ── State labels ───────────────────────────────────────────────────────────────

class State:
    WAITING         = "WAITING"
    WALKING_THE_NET = "WALKING_THE_NET"
    LOST_WALL       = "LOST_WALL"
    LAP_COMPLETED   = "LAP_COMPLETED"


# ── Helper functions ───────────────────────────────────────────────────────────

def _yaw_from_odom(odom: Odometry) -> float:
    """Extract yaw from the filtered odometry quaternion."""
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a − b in (−π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class PID:
    """Simple discrete PID with anti-windup clamping on the integral."""

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float = 50.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0
        self._integral_limit = integral_limit

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return self.kp * error
        self._integral += error * dt
        self._integral = float(np.clip(
            self._integral, -self._integral_limit, self._integral_limit
        ))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        return (self.kp * error
                + self.ki * self._integral
                + self.kd * derivative)

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


# ── Main node ──────────────────────────────────────────────────────────────────

class Phase3InspectionBigNetNode(Node):
    """
    ROS 2 guidance node for Phase 3 (Inspection) of the AUV mission — BIG NET.

    Subscribes to the output of auv_perception/sonoptix_perception:
      /sonoptix/perception       → PoseStamped (distance + normal quaternion)
      /sonoptix/perception_valid → Bool (RANSAC validity gate)

    State machine:
      WAITING → WALKING_THE_NET → (LOST_WALL ↔ WALKING_THE_NET) → LAP_COMPLETED

    On each completed lap the target depth is decremented by DEPTH_STEP until
    FINAL_DEPTH_LIMIT is reached, at which point the mission ends.
    """

    def __init__(self):
        super().__init__('phase3_inspection')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10
        )
        # Processed perception result from sonoptix_perception node
        self.create_subscription(
            PoseStamped, '/sonoptix/perception',
            self._perception_cb, best_effort_qos
        )
        self.create_subscription(
            Bool, '/sonoptix/perception_valid',
            self._perception_valid_cb, best_effort_qos
        )
        self.create_subscription(
            Bool, '/mission/phase2_done', self._phase2_done_cb, latching_qos
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self.wrench_pub      = self.create_publisher(Wrench,      '/auv/command_wrench',   10)
        self.phase3_done_pub = self.create_publisher(Bool,        '/mission/phase3_done',  10)
        self.phase_pub       = self.create_publisher(String,      '/mission/phase',        10)

        # Diagnostic topics (Foxglove / PlotJuggler)
        self.wall_dist_pub          = self.create_publisher(Float64, '/phase3/wall_distance',          10)
        self.wall_dist_error_pub    = self.create_publisher(Float64, '/phase3/wall_dist_error',        10)
        self.wall_dist_smoothed_pub = self.create_publisher(Float64, '/phase3/wall_distance_smoothed', 10)
        self.yaw_pub                = self.create_publisher(Float64, '/phase3/yaw',                    10)
        self.yaw_error_pub          = self.create_publisher(Float64, '/phase3/yaw_error',              10)
        self.depth_pub              = self.create_publisher(Float64, '/phase3/depth',                  10)
        self.depth_error_pub        = self.create_publisher(Float64, '/phase3/depth_error',            10)
        self.cmd_fx_pub             = self.create_publisher(Float64, '/phase3/cmd_Fx',                 10)
        self.cmd_fy_pub             = self.create_publisher(Float64, '/phase3/cmd_Fy',                 10)
        self.cmd_fz_pub             = self.create_publisher(Float64, '/phase3/cmd_Fz',                 10)
        self.cmd_mz_pub             = self.create_publisher(Float64, '/phase3/cmd_Mz',                 10)
        self.cmd_my_pub             = self.create_publisher(Float64, '/phase3/cmd_My',                 10)
        self.yaw_accum_pub          = self.create_publisher(Float64, '/phase3/yaw_accumulated',        10)
        self.current_radius_pub     = self.create_publisher(Float64, '/phase3/current_radius',         10)
        self.r_ref_pub              = self.create_publisher(Float64, '/phase3/r_ref',                  10)
        self.accumulated_dist_pub   = self.create_publisher(Float64, '/phase3/accumulated_dist',       10)
        self.walking_time_pub       = self.create_publisher(Float64, '/phase3/walking_time',           10)
        self.real_time_elapsed_pub  = self.create_publisher(Float64, '/phase3/real_time_elapsed',      10)
        self.sim_time_elapsed_pub   = self.create_publisher(Float64, '/phase3/sim_time_elapsed',       10)
        self.rtf_pub                = self.create_publisher(Float64, '/phase3/real_time_factor',       10)

        # TF2 & relative telemetry
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.relative_pose_pub = self.create_publisher(
            PoseStamped, '/phase3/relative_pose', 10
        )

        # ── Internal state ────────────────────────────────────────────────────
        self.state: str = State.WAITING

        self._have_odom   = False
        self.current_z    = 0.0
        self.current_yaw  = 0.0
        self.current_vyaw = 0.0
        self.current_vy   = 0.0

        self.target_depth = TARGET_DEPTH

        # Perception data from sonoptix_perception (post-RANSAC)
        self._raw_net_range:     float | None = None
        self.net_range:          float | None = None
        self._last_sonar_time:   float | None = None
        self.net_angle_error:    float = 0.0
        self.pitch_error:        float = 0.0   # RANSAC provides this directly
        self._smoothed_yaw_error: float = 0.0  # EMA on yaw error
        self._last_Mz:            float = 0.0  # for Mz rate limiting
        self._perception_valid:   bool  = False
        self._last_perception_invalid_time: float | None = None

        # Spike filter for the range channel
        self._smoothed_net_range: float | None = None
        self.declare_parameter('max_valid_jump_m', 0.3)
        self._max_valid_jump_m: float = self.get_parameter('max_valid_jump_m').value
        self.declare_parameter('max_consecutive_rejections', 5)
        self._max_consecutive_rejections: int = (
            self.get_parameter('max_consecutive_rejections').value
        )
        self._consecutive_rejections: int = 0
        self.declare_parameter('range_ema_alpha', 0.5)
        self._range_ema_alpha: float = self.get_parameter('range_ema_alpha').value

        # Cone mode (conical net geometry)
        self.in_cone_mode              = False
        self.cone_transition_start_time = None
        self.apex_condition_start_time  = None
        self.R_ref         = None
        self.radius_samples = []

        # PIDs
        self._pid_depth         = PID(KP_DEPTH,     KI_DEPTH,     KD_DEPTH,     integral_limit=50.0)
        self._pid_dist          = PID(KP_DIST,      KI_DIST,      KD_DIST,      integral_limit=10.0)
        self._pid_yaw           = PID(KP_YAW,       KI_YAW,       KD_YAW,       integral_limit=10.0)
        self._pid_pitch         = PID(KP_PITCH,     KI_PITCH,     KD_PITCH,     integral_limit=10.0)
        self._pid_velocity_sway = PID(KP_VEL_SWAY, KI_VEL_SWAY, KD_VEL_SWAY, integral_limit=20.0)

        self._last_fx = 0.0

        # Lap / orbit tracking
        self._start_yaw:                float | None = None
        self._prev_yaw:                 float | None = None
        self._accumulated_yaw:          float = 0.0
        self._lap_start_time:           float | None = None
        self._walking_start_time:       float | None = None
        self._first_walking_start_time: float | None = None
        self._accumulated_walking_time: float = 0.0
        self._accumulated_dist:         float = 0.0
        self._last_R_calculated:        float = 0.0

        self._last_loop_time: float | None = None
        self._target_yaw:     float | None = None
        self._yaw_error_prev: float = 0.0

        # Parameters
        self.declare_parameter('control_rate_hz', CONTROL_RATE_HZ)
        _rate = float(self.get_parameter('control_rate_hz').value)
        self._dt = 1.0 / _rate

        self.declare_parameter('yaw_ema_alpha', 1.0)
        self._yaw_ema_alpha = float(self.get_parameter('yaw_ema_alpha').value)

        # Timing
        self._start_real_time = time.time()
        self._start_sim_time  = self.get_clock().now().nanoseconds * 1e-9

        self.create_timer(self._dt, self._control_loop)
        self.get_logger().info(
            f'[Phase3BigNet] Node started — state: WAITING\n'
            f'  Control rate    : {_rate:.0f} Hz\n'
            f'  Yaw EMA alpha   : {self._yaw_ema_alpha:.2f}\n'
            f'  Final depth     : {FINAL_DEPTH_LIMIT} m\n'
            f'  Perception input: /sonoptix/perception  +  /sonoptix/perception_valid'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        """Update current pose and velocities from the filtered odometry."""
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self.current_vy   = msg.twist.twist.linear.y
        self._have_odom   = True

    def _perception_cb(self, msg: PoseStamped):
        """
        Receive the RANSAC perception result from sonoptix_perception.

        Pipeline applied to the raw range:
          raw → spike filter → EMA → self._smoothed_net_range

        Field mapping:
          net distance = msg.pose.position.x              [m]
          yaw error    = euler_from_quaternion(...)[2]     [rad]
          pitch error  = euler_from_quaternion(...)[1]     [rad]
        """
        if self.state not in (State.WALKING_THE_NET, State.LOST_WALL):
            return

        raw = float(msg.pose.position.x)
        self._raw_net_range   = raw
        self._last_sonar_time = self.get_clock().now().nanoseconds * 1e-9

        # Publish raw (unfiltered) for diagnostics
        self._pub_float(self.wall_dist_pub,       raw)
        self._pub_float(self.wall_dist_error_pub, raw - STANDOFF_DIST)

        # ── Spike filter ─────────────────────────────────────────────────────
        if self._smoothed_net_range is not None:
            jump = abs(raw - self._smoothed_net_range)
            if jump > self._max_valid_jump_m:
                self._consecutive_rejections += 1
                if self._consecutive_rejections < self._max_consecutive_rejections:
                    self.get_logger().debug(
                        f'[SpikeFilt] Spike #{self._consecutive_rejections} rejected: '
                        f'jump={jump:.3f}m > {self._max_valid_jump_m:.3f}m '
                        f'(raw={raw:.3f}m, smooth={self._smoothed_net_range:.3f}m)',
                        throttle_duration_sec=1.0,
                    )
                    return   # discard — do not update EMA or angular errors
                else:
                    # Jump accepted after repeated rejections (new physical reality)
                    self.get_logger().debug(
                        f'[SpikeFilt] Jump accepted after {self._consecutive_rejections} '
                        f'rejections (jump={jump:.3f}m). Resetting EMA.'
                    )
                    self._smoothed_net_range = raw
                    self._consecutive_rejections = 0
            else:
                self._consecutive_rejections = 0

        # ── EMA filter ───────────────────────────────────────────────────────
        if self._smoothed_net_range is None:
            self._smoothed_net_range = raw
        else:
            self._smoothed_net_range += self._range_ema_alpha * (
                raw - self._smoothed_net_range
            )

        self._pub_float(self.wall_dist_smoothed_pub, self._smoothed_net_range)

        # ── Angular errors from the RANSAC quaternion ─────────────────────────
        q = msg.pose.orientation
        _, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.net_angle_error = yaw
        self.pitch_error     = -pitch

    def _perception_valid_cb(self, msg: Bool):
        """
        Track RANSAC validity from sonoptix_perception.
        A prolonged False triggers the LOST_WALL recovery state.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        self._perception_valid = msg.data
        if not msg.data:
            if self._last_perception_invalid_time is None:
                self._last_perception_invalid_time = now
        else:
            self._last_perception_invalid_time = None

    def _phase2_done_cb(self, msg: Bool):
        """Start the inspection orbit once Phase 2 signals completion."""
        if msg.data and self.state == State.WAITING:
            self.get_logger().info(
                '[PHASE3-BigNet] Phase 2 done — activating inspection orbit.'
            )
            self.state = State.WALKING_THE_NET
            now = self.get_clock().now().nanoseconds * 1e-9
            self._walking_start_time       = now
            self._first_walking_start_time = now
            self._last_sonar_time = None
            self._raw_net_range   = None
            self._smoothed_net_range = None

    # ── Control loop ───────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._have_odom:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = self._dt
        if self._last_loop_time is not None:
            real_dt = now - self._last_loop_time
            if 0.005 < real_dt < 0.5:
                dt = real_dt
        self._last_loop_time = now

        # ── Real-time factor diagnostics ──────────────────────────────────────
        real_elapsed = time.time() - self._start_real_time
        sim_elapsed  = now - self._start_sim_time
        self._pub_float(self.real_time_elapsed_pub, real_elapsed)
        self._pub_float(self.sim_time_elapsed_pub,  sim_elapsed)
        if real_elapsed > 0:
            self._pub_float(self.rtf_pub, sim_elapsed / real_elapsed)

        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)

        # TF2 relative pose
        try:
            t = self.tf_buffer.lookup_transform(
                'local_origin', 'base_link', rclpy.time.Time()
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'local_origin'
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            self.relative_pose_pub.publish(pose_msg)
        except tf2_ros.TransformException:
            pass

        if self.state == State.WAITING:
            return

        # ── LAP_COMPLETED: hold depth and signal done ─────────────────────────
        if self.state == State.LAP_COMPLETED:
            done_msg = Bool()
            done_msg.data = True
            self.phase3_done_pub.publish(done_msg)
            depth_error = self.target_depth - self.current_z
            fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
            Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))
            self._publish_thrusters(0.0, 0.0, Fz, 0.0, 0.0)
            return

        # ── Lap / orbit tracking ──────────────────────────────────────────────
        if self._start_yaw is None:
            self._start_yaw       = self.current_yaw
            self._prev_yaw        = self.current_yaw
            self._accumulated_yaw = 0.0
            self._lap_start_time  = now
            self._target_yaw      = self.current_yaw
            self.get_logger().info(
                f'[WALKING_THE_NET] Orbit initialised at '
                f'yaw={math.degrees(self._start_yaw):.1f}°'
            )
        else:
            delta = _angle_diff(self.current_yaw, self._prev_yaw)
            if abs(delta) > math.radians(0.05):
                self._accumulated_yaw += delta
            self._prev_yaw = self.current_yaw

        self._pub_float(self.yaw_accum_pub, self._accumulated_yaw)
        self._accumulated_dist += abs(self.current_vy) * dt
        self._pub_float(self.accumulated_dist_pub, self._accumulated_dist)

        if abs(self._accumulated_yaw) > 0.4:
            current_radius = self._accumulated_dist / abs(self._accumulated_yaw)
        elif self._last_R_calculated > 0.0:
            current_radius = self._last_R_calculated
        else:
            current_radius = 25.0   # large initial default for big net
        self._pub_float(self.current_radius_pub, current_radius)

        if self.state == State.WALKING_THE_NET:
            self._accumulated_walking_time += dt
        self._pub_float(self.walking_time_pub, self._accumulated_walking_time)

        # ── R_ref estimation and cone detection ───────────────────────────────
        if self._first_walking_start_time is not None:
            if self.R_ref is None:
                if self._accumulated_walking_time < 30.0:
                    if self.state == State.WALKING_THE_NET:
                        self.radius_samples.append(current_radius)
                elif self.radius_samples:
                    self.R_ref = float(np.mean(self.radius_samples))
                    self.get_logger().info(
                        f'[PHASE3-BigNet] R_ref estimated: {self.R_ref:.2f} m '
                        f'(mean over {len(self.radius_samples)} samples)'
                    )

            if self.R_ref is not None and not self.in_cone_mode:
                if current_radius < 0.9 * self.R_ref:
                    if self.cone_transition_start_time is None:
                        self.cone_transition_start_time = now
                    elif now - self.cone_transition_start_time > 3.0:
                        self.in_cone_mode = True
                        self.get_logger().info(
                            '[PHASE3-BigNet] Cone transition detected — enabling pitch control.'
                        )
                else:
                    self.cone_transition_start_time = None

            if self.in_cone_mode:
                if current_radius < 1.0:
                    if self.apex_condition_start_time is None:
                        self.apex_condition_start_time = now
                    elif now - self.apex_condition_start_time > 5.0:
                        self.get_logger().info(
                            '[PHASE3-BigNet] Apex reached (radius < 1 m for 5 s). '
                            'Mission complete — ascending.'
                        )
                        self.state = State.LAP_COMPLETED
                        self.target_depth = -2.0
                else:
                    self.apex_condition_start_time = None

        if self.R_ref is not None:
            self._pub_float(self.r_ref_pub, self.R_ref)

        # ── Sonar / perception availability ───────────────────────────────────
        sonar_age = (
            now - self._last_sonar_time
            if self._last_sonar_time is not None
            else float('inf')
        )
        sonar_ok = sonar_age < LOST_WALL_TIMEOUT and self._raw_net_range is not None

        # Use EMA-smoothed range when available; clear on loss to prevent stale data
        if sonar_ok:
            self.net_range = self._smoothed_net_range
        else:
            self.net_range = None
            self._smoothed_net_range = None   # reset EMA on signal loss

        walking_age = (
            now - self._walking_start_time
            if self._walking_start_time is not None
            else 0.0
        )
        past_grace = walking_age > LOST_WALL_GRACE_S

        # LOST_WALL trigger: sonar gone OR RANSAC persistently invalid
        perception_timeout = (
            self._last_perception_invalid_time is not None
            and (now - self._last_perception_invalid_time) >= LOST_WALL_TIMEOUT
        )
        lost_condition = (not sonar_ok or perception_timeout) and past_grace

        if self.state == State.WALKING_THE_NET and lost_condition:
            reason = (
                'sonar signal lost' if not sonar_ok
                else 'RANSAC invalid (perception_valid=False)'
            )
            self.get_logger().warn(
                f'[LOST_WALL] {reason} (sonar_age={sonar_age:.2f}s) — entering recovery.'
            )
            self.state = State.LOST_WALL
            self._pid_dist.reset()

        elif self.state == State.LOST_WALL and sonar_ok and self._perception_valid:
            self.get_logger().info(
                '[WALKING_THE_NET] Sonar + RANSAC recovered — resuming orbit.'
            )
            self.state = State.WALKING_THE_NET
            self._walking_start_time = now
            self._last_perception_invalid_time = None

        # ── Dispatch to the appropriate state handler ─────────────────────────
        if self.state == State.LOST_WALL:
            self._do_lost_wall(dt)
        else:
            self._do_walking(dt)

        # ── Lap completion check ──────────────────────────────────────────────
        elapsed_since_start = (
            now - self._lap_start_time if self._lap_start_time else 0.0
        )
        if (elapsed_since_start > LAP_START_DELAY
                and abs(self._accumulated_yaw) >= LAP_YAW_THRESHOLD):

            self._last_R_calculated = self._accumulated_dist / (2 * math.pi)
            self.get_logger().info(
                f'[LAP_COMPLETED] Full orbit at depth {self.target_depth}! '
                f'Accumulated yaw: {math.degrees(self._accumulated_yaw):.1f}°  '
                f'Estimated radius: {self._last_R_calculated:.2f} m'
            )
            self._accumulated_dist = 0.0

            if self.target_depth - DEPTH_STEP + 0.01 >= FINAL_DEPTH_LIMIT:
                self.target_depth -= DEPTH_STEP
                self._accumulated_yaw = 0.0
                self._lap_start_time  = now
                self.apex_condition_start_time = None
                cone_status = ' (cone mode maintained)' if self.in_cone_mode else ''
                self.get_logger().info(
                    f'[DESCENDING] New target depth: {self.target_depth} m{cone_status}'
                )
            else:
                self.state = State.LAP_COMPLETED
                self._publish_thrusters(0.0, 0.0, 0.0, 0.0)
                done_msg = Bool()
                done_msg.data = True
                self.phase3_done_pub.publish(done_msg)

    # ── State handlers ─────────────────────────────────────────────────────────

    def _do_walking(self, dt: float):
        """Normal orbit: depth hold + standoff control + yaw alignment."""
        depth_error = self.target_depth - self.current_z
        fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
        Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))

        if self.net_range is not None:
            dist_error = self.net_range - STANDOFF_DIST
            Fx = float(np.clip(
                self._pid_dist.compute(dist_error, dt), -MAX_DIST_CMD, MAX_DIST_CMD
            ))
            self._last_fx = Fx
        else:
            Fx = self._last_fx

        if abs(depth_error) < 0.15:
            target_vy = float(ORBIT_DIRECTION * 0.25)
            Fy = float(np.clip(
                self._pid_velocity_sway.compute(target_vy - self.current_vy, dt),
                -15.0, 15.0,
            ))
        else:
            Fy = float(np.clip(
                self._pid_velocity_sway.compute(0.0 - self.current_vy, dt),
                -10.0, 10.0,
            ))

        # EMA-smooth yaw error to absorb 2 Hz sonar update jitter
        self._smoothed_yaw_error += self._yaw_ema_alpha * (
            self.net_angle_error - self._smoothed_yaw_error
        )
        yaw_error = self._smoothed_yaw_error
        Mz_raw = float(np.clip(
            self._pid_yaw.compute(yaw_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD
        ))
        # Rate-limit Mz to prevent abrupt torque spikes
        Mz_delta = float(np.clip(Mz_raw - self._last_Mz, -MZ_RATE_LIMIT, MZ_RATE_LIMIT))
        Mz = self._last_Mz + Mz_delta
        self._last_Mz = Mz

        # Pitch: RANSAC provides this directly
        My = float(np.clip(
            self._pid_pitch.compute(self.pitch_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD
        )) if self.in_cone_mode else 0.0

        # Thruster saturation guard
        total_cmd = abs(Fx) + abs(Fz) + abs(Mz) + abs(My)
        if total_cmd + abs(Fy) > MAX_INDIVIDUAL_THRUST:
            max_fy = max(0.0, MAX_INDIVIDUAL_THRUST - total_cmd)
            Fy = float(np.clip(Fy, -max_fy, max_fy))

        self._publish_thrusters(Fx, Fy, Fz, Mz, My)
        self._publish_diagnostics(
            Fx, Fy, Fz, Mz,
            depth_error,
            self.net_range - STANDOFF_DIST if self.net_range else 0.0,
            yaw_error,
            My,
        )

    def _do_lost_wall(self, dt: float):
        """Recovery mode: hold depth and rotate slowly to re-acquire the net."""
        depth_error = self.target_depth - self.current_z
        fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
        Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))

        # Pull back slightly and rotate to search for the net
        Fx = -5.0
        Fy = 0.0
        Mz = float(ORBIT_DIRECTION * 0.5)

        self._publish_thrusters(Fx, Fy, Fz, Mz, 0.0)

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _pub_float(self, publisher, value: float):
        m = Float64()
        m.data = float(value)
        publisher.publish(m)

    def _publish_diagnostics(
        self,
        Fx: float, Fy: float, Fz: float, Mz: float,
        depth_error: float,
        dist_error: float,
        yaw_error: float,
        My: float = 0.0,
    ):
        if self.net_range is not None:
            self._pub_float(self.wall_dist_pub,       self.net_range)
            self._pub_float(self.wall_dist_error_pub, dist_error)
        self._pub_float(self.depth_pub,       self.current_z)
        self._pub_float(self.depth_error_pub, depth_error)
        self._pub_float(self.yaw_pub,         self.current_yaw)
        self._pub_float(self.yaw_error_pub,   yaw_error)
        self._pub_float(self.cmd_fx_pub, Fx)
        self._pub_float(self.cmd_fy_pub, Fy)
        self._pub_float(self.cmd_fz_pub, Fz)
        self._pub_float(self.cmd_mz_pub, Mz)
        self._pub_float(self.cmd_my_pub, My)

    def _publish_thrusters(
        self,
        Fx: float, Fy: float, Fz: float, Mz: float, My: float = 0.0,
    ):
        wrench_msg = Wrench()
        wrench_msg.force.x  = float(Fx)
        wrench_msg.force.y  = float(Fy)
        wrench_msg.force.z  = float(Fz)
        wrench_msg.torque.y = float(My)
        wrench_msg.torque.z = float(Mz)
        self.wrench_pub.publish(wrench_msg)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Phase3InspectionBigNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[Phase3BigNet] Keyboard interrupt — shutting down.')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
