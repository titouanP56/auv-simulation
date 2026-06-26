#!/usr/bin/env python3
"""
phase3_inspection_2D_sono.py
============================
ROS 2 guidance node — Phase 3 of the AUV mission: orbital inspection of an
aquaculture net using the **Sonoptix ECHO 2D** sonar.

This is the **active Phase 3 node** used by `net_full_inspection_true_auv.launch.py`.

Mission State Machine
---------------------
  WAITING
    ↓ /mission/phase2_done = True received
  WALKING_THE_NET  ←──────────────────────────┐
    ↓ sonar lost > LOST_WALL_TIMEOUT (2 s)    │ sonar recovered
  LOST_WALL ─── pulls back (Fx=−5) + rotates ┘
    ↓ accumulated yaw ≥ 2π → decrement depth, reset yaw accumulator
    ↓ depth ≤ FINAL_DEPTH_LIMIT
  LAP_COMPLETED ─── publishes /mission/phase3_done = True

Five simultaneous PID controllers
----------------------------------
  Fz  (depth)      : PID — TARGET_DEPTH (decrements by DEPTH_STEP each lap)
  Fx  (standoff)   : PID — STANDOFF_DIST from /perception/net_distance
  Fy  (sway vel.)  : PID — 0.25 m/s lateral orbit speed
  Mz  (yaw)        : PID + EMA + rate-limit — from /perception/net_yaw_target
  My  (pitch)      : PID — only when in_cone_mode = True (conical net apex)

Sonar distance filtering
-------------------------
  Spike filter: rejects jumps > SPIKE_THRESHOLD (0.3 m), up to MAX_CONSEC_SPIKES.
  EMA filter (α = 0.5): smooths validated distance values.

Cone mode detection
--------------------
  The node estimates orbit radius from odometry + local_origin TF over the first
  RADIUS_REF_WINDOW (30 s). If current radius < 90% of R_ref for more than
  CONE_CONFIRM_TIME (3 s), `in_cone_mode` is activated and the pitch PID engages.

Subscriptions
-------------
  /odometry/filtered          nav_msgs/Odometry
  /perception/net_distance    std_msgs/Float32
  /perception/net_yaw_target  std_msgs/Float32
  /perception/perception_valid std_msgs/Bool
  /mission/phase2_done        std_msgs/Bool

Publications
------------
  /auv/command_wrench  geometry_msgs/Wrench
  /mission/phase        std_msgs/String
  /mission/phase3_done  std_msgs/Bool
  /phase3/*             std_msgs/Float64 (diagnostic telemetry)

Key constants (top of file, no recompile needed)
------------------------------------------------
  STANDOFF_DIST    1.5 m     Desired distance from net surface
  ORBIT_DIRECTION  +1        +1 = CCW, −1 = CW orbit
  DEPTH_STEP       0.5 m     Depth decrement after each full lap
  FINAL_DEPTH_LIMIT −6.0 m  Stop depth
  KP_DIST/KI/KD    4.0/0.2/0.5  Standoff PID
  KP_YAW/KI/KD     5.0/0.02/1.0 Yaw alignment PID
  KP_VEL_SWAY      15.0     Lateral orbit speed gain

Author  : titou
Package : AUV_guidance
"""

import math
import numpy as np
import time

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    """
    x, y, z, w = quaternion
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Float32, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Wrench
import tf2_ros

# ── Physical constants and PID gains ───────────────────────────────────────────

TARGET_DEPTH          = -2.0    # [m] target inspection depth
STANDOFF_DIST         = 1.5     # [m] desired distance from the net surface
CONTROL_RATE_HZ       = 10.0   

KP_DEPTH              = 10.0
KI_DEPTH              = 0.2
KD_DEPTH              = 0.2
BUOYANCY_COMP         = 3.0    # [N] static buoyancy force offset

KP_DIST               = 4.0
KI_DIST               = 0.2
KD_DIST               = 0.5

KP_VEL_SWAY           = 15.0
KI_VEL_SWAY           = 2.0
KD_VEL_SWAY           = 0.5

KP_YAW                = 5.0
KI_YAW                = 0.02
KD_YAW                = 1.0

KP_PITCH              = 10.0
KI_PITCH              = 0.2
KD_PITCH              = 2.0

MAX_DEPTH_CMD         = 15.0   # [N]
MAX_DIST_CMD          = 15.0   # [N]
MAX_YAW_CMD           = 10.0   # [N·m]
MAX_INDIVIDUAL_THRUST = 40.0   # [N] per-thruster saturation

MZ_RATE_LIMIT         = 3.0    # [N·m/step] maximum Mz change per control cycle

ORBIT_DIRECTION       = 1      # +1 = counter-clockwise, -1 = clockwise
PERCENTILE_FRACTION   = 0.10   # fraction of closest range readings to average
MEDIAN_WINDOW         = 7      # window size for the range median filter
SPIKE_THRESHOLD       = 0.5    # [m] jump threshold for the spike rejection filter

LAP_YAW_THRESHOLD     = 2.0 * math.pi  # [rad] accumulated yaw to declare a completed lap
LAP_START_DELAY       = 2.0            # [s] grace period before lap tracking begins
LOST_WALL_TIMEOUT     = 2.0            # [s] sonar silence before entering LOST_WALL
LOST_WALL_GRACE_S     = 2.0            # [s] grace period at walk start before LOST_WALL fires
RECOVERY_YAW_CMD      = 4.0            # [N·m] recovery yaw torque during LOST_WALL

DEPTH_STEP            = 0.5            # [m] depth increment per completed lap
FINAL_DEPTH_LIMIT     = -6.0           # [m] maximum depth (mission ends here)


# ── State labels ───────────────────────────────────────────────────────────────

class State:
    WAITING        = "WAITING"
    WALKING_THE_NET = "WALKING_THE_NET"
    LOST_WALL      = "LOST_WALL"
    LAP_COMPLETED  = "LAP_COMPLETED"


# ── Helper functions ───────────────────────────────────────────────────────────

def _yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a − b ∈ (−π, π]."""
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
        self._integral = np.clip(
            self._integral, -self._integral_limit, self._integral_limit
        )
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        return (self.kp * error
                + self.ki * self._integral
                + self.kd * derivative)

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


# ── Main node ──────────────────────────────────────────────────────────────────

class Phase3InspectionNode(Node):
    """
    ROS 2 node for Phase 3 (Inspection) of the AUV mission.

    Orbits ("walks") along the net surface at a fixed standoff distance and
    target depth. PID controllers regulate depth, distance to net, sway
    velocity, yaw, and pitch. 
    Uses sonoptix 2D for distance and yaw, and calculates pitch based on 
    forward movement during descent.
    """

    def __init__(self):
        super().__init__('phase3_inspection_2D_sono')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10
        )
        # 2D Perception topics
        self.create_subscription(
            Float32, '/perception/net_distance', self._distance_cb, best_effort_qos
        )
        self.create_subscription(
            Float32, '/perception/net_yaw_target', self._yaw_target_cb, best_effort_qos
        )
        self.create_subscription(
            Bool, '/perception/perception_valid', self._perception_valid_cb, best_effort_qos
        )

        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            Bool, '/mission/phase2_done', self._phase2_done_cb, latching_qos
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.wrench_pub = self.create_publisher(Wrench, '/auv/command_wrench', 10)
        self.phase3_done_pub = self.create_publisher(Bool, '/mission/phase3_done', 10)
        self.phase_pub       = self.create_publisher(String, '/mission/phase', 10)

        # ── Diagnostic topics (Foxglove) ─────────────────────────────────────
        self.wall_dist_pub         = self.create_publisher(Float64, '/phase3/wall_distance',         10)
        self.wall_dist_error_pub   = self.create_publisher(Float64, '/phase3/wall_dist_error',       10)
        self.wall_dist_smoothed_pub = self.create_publisher(Float64, '/phase3/wall_distance_smoothed', 10)

        self.yaw_pub             = self.create_publisher(Float64, '/phase3/yaw',              10)
        self.yaw_error_pub       = self.create_publisher(Float64, '/phase3/yaw_error',        10)

        self.depth_pub           = self.create_publisher(Float64, '/phase3/depth',            10)
        self.depth_error_pub     = self.create_publisher(Float64, '/phase3/depth_error',      10)

        self.cmd_fx_pub          = self.create_publisher(Float64, '/phase3/cmd_Fx',           10)
        self.cmd_fy_pub          = self.create_publisher(Float64, '/phase3/cmd_Fy',           10)
        self.cmd_fz_pub          = self.create_publisher(Float64, '/phase3/cmd_Fz',           10)
        self.cmd_mz_pub          = self.create_publisher(Float64, '/phase3/cmd_Mz',           10)
        self.cmd_my_pub          = self.create_publisher(Float64, '/phase3/cmd_My',           10)
        self.target_pitch_pub    = self.create_publisher(Float64, '/phase3/target_pitch',     10)
        
        self.yaw_accum_pub       = self.create_publisher(Float64, '/phase3/yaw_accumulated',  10)
        self.current_radius_pub  = self.create_publisher(Float64, '/phase3/current_radius',   10)
        self.r_ref_pub           = self.create_publisher(Float64, '/phase3/r_ref',            10)
        self.accumulated_dist_pub = self.create_publisher(Float64, '/phase3/accumulated_dist', 10)
        self.walking_time_pub    = self.create_publisher(Float64, '/phase3/walking_time',     10)

        self.real_time_elapsed_pub = self.create_publisher(Float64, '/phase3/real_time_elapsed', 10)
        self.sim_time_elapsed_pub  = self.create_publisher(Float64, '/phase3/sim_time_elapsed',  10)
        self.rtf_pub               = self.create_publisher(Float64, '/phase3/real_time_factor', 10)

        # ── TF2 & relative telemetry ──────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.relative_pose_pub = self.create_publisher(PoseStamped, '/phase3/relative_pose', 10)

        self.state: str = State.WAITING

        self._have_odom = False
        self.current_z   = 0.0
        self.current_yaw = 0.0
        self.current_vyaw = 0.0
        self.current_vy  = 0.0
        self.current_vx  = 0.0
        self.current_pitch = 0.0

        self.target_depth = TARGET_DEPTH

        self._raw_net_range: float | None = None
        self.net_range: float | None = None
        self._last_sonar_time: float | None = None
        self.net_angle_error: float = 0.0
        self.pitch_error: float = 0.0           
        self.target_pitch: float = 0.0
        
        # Descent tracking
        self._is_descending = False
        self._descent_start_z = 0.0
        self._accumulated_dx_descent = 0.0

        self._smoothed_yaw_error: float = 0.0   # EMA-filtered yaw error
        self._last_Mz: float = 0.0              # for Mz rate limiting

        # Validity of the last perception estimate
        self._perception_valid: bool = False
        self._last_perception_invalid_time: float | None = None

        self.in_cone_mode = False
        self.cone_transition_start_time = None
        self.apex_condition_start_time = None
        self.R_ref = None
        self.radius_samples = []

        self._pid_depth = PID(KP_DEPTH, KI_DEPTH, KD_DEPTH, integral_limit=50.0)
        self._pid_dist  = PID(KP_DIST,  KI_DIST,  KD_DIST,  integral_limit=10.0)
        self._pid_yaw   = PID(KP_YAW,   KI_YAW,   KD_YAW,   integral_limit=10.0)
        self._pid_pitch = PID(KP_PITCH, KI_PITCH, KD_PITCH, integral_limit=10.0)
        self._pid_velocity_sway = PID(KP_VEL_SWAY, KI_VEL_SWAY, KD_VEL_SWAY, integral_limit=20.0)

        self._last_fx = 0.0

        self._start_yaw: float | None = None
        self._prev_yaw: float | None = None
        self._accumulated_yaw = 0.0
        self._lap_start_time: float | None = None
        self._walking_start_time: float | None = None
        self._first_walking_start_time: float | None = None
        self._accumulated_walking_time = 0.0
        self._accumulated_dist = 0.0
        self._last_R_calculated = 0.0

        self.declare_parameter('control_rate_hz', CONTROL_RATE_HZ)
        _rate = self.get_parameter('control_rate_hz').value
        self._dt = 1.0 / _rate
        self.declare_parameter('yaw_ema_alpha', 1.0)
        self._yaw_ema_alpha = self.get_parameter('yaw_ema_alpha').value
        self.declare_parameter('range_ema_alpha', 0.5)
        self._range_ema_alpha: float = self.get_parameter('range_ema_alpha').value
        self._smoothed_net_range: float | None = None

        # ── Spike filter parameters ──────────────────────────────────────────
        self.declare_parameter('max_valid_jump_m', 0.3)
        self._max_valid_jump_m: float = self.get_parameter('max_valid_jump_m').value
        self.declare_parameter('max_consecutive_rejections', 5)
        self._max_consecutive_rejections: int = self.get_parameter('max_consecutive_rejections').value
        self._consecutive_rejections: int = 0
        self._last_loop_time: float | None = None
        self._target_yaw: float | None = None   
        self._yaw_error_prev = 0.0

        self._start_real_time = time.time()
        self._start_sim_time  = self.get_clock().now().nanoseconds * 1e-9

        # ── Control timer ────────────────────────────────────────────────────
        self.create_timer(self._dt, self._control_loop)
        self.get_logger().info(f"Phase3InspectionNode (2D Sono) started — state: WAITING")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self.current_vy   = msg.twist.twist.linear.y
        self.current_vx   = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        _, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pitch = pitch
        self._have_odom   = True

    def _distance_cb(self, msg: Float32):
        if self.state not in (State.WALKING_THE_NET, State.LOST_WALL):
            return

        raw = float(msg.data)
        self._raw_net_range   = raw
        self._last_sonar_time = self.get_clock().now().nanoseconds * 1e-9

        # ── 1. Publish raw value (before any filtering) ───────────────────────
        wd_raw = Float64()
        wd_raw.data = raw
        self.wall_dist_pub.publish(wd_raw)

        err_raw = Float64()
        err_raw.data = raw - STANDOFF_DIST
        self.wall_dist_error_pub.publish(err_raw)

        # ── 2. Spike filter (outlier rejection) ───────────────────────────────
        if self._smoothed_net_range is not None:
            jump = abs(raw - self._smoothed_net_range)
            if jump > self._max_valid_jump_m:
                self._consecutive_rejections += 1
                if self._consecutive_rejections < self._max_consecutive_rejections:
                    return  
                else:
                    self._smoothed_net_range = raw  
                    self._consecutive_rejections = 0
            else:
                self._consecutive_rejections = 0

        # ── 3. EMA filter on the validated range ──────────────────────────────
        if self._smoothed_net_range is None:
            self._smoothed_net_range = raw  
        else:
            self._smoothed_net_range += self._range_ema_alpha * (raw - self._smoothed_net_range)

        # ── 4. Publish smoothed value (after spike filter + EMA) ──────────────
        wd_smooth = Float64()
        wd_smooth.data = self._smoothed_net_range
        self.wall_dist_smoothed_pub.publish(wd_smooth)

    def _yaw_target_cb(self, msg: Float32):
        if self.state not in (State.WALKING_THE_NET, State.LOST_WALL):
            return
        self.net_angle_error = _angle_diff(float(msg.data), math.pi)

    def _perception_valid_cb(self, msg: Bool):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._perception_valid = msg.data
        if not msg.data:
            if self._last_perception_invalid_time is None:
                self._last_perception_invalid_time = now
        else:
            self._last_perception_invalid_time = None

    def _phase2_done_cb(self, msg: Bool):
        if msg.data and self.state == State.WAITING:
            self.get_logger().info("[PHASE3] Phase 2 done received — activating inspection orbit.")
            self.state = State.WALKING_THE_NET
            self._walking_start_time = self.get_clock().now().nanoseconds * 1e-9
            self._first_walking_start_time = self._walking_start_time
            self._last_sonar_time = None
            self._raw_net_range   = None

    # ── Control loop ──────────────────────────────────────────────────────────

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

        # ── Real-time vs simulation-time metrics ─────────────────────────────
        real_elapsed = time.time() - self._start_real_time
        sim_elapsed  = now - self._start_sim_time
        
        re_msg = Float64()
        re_msg.data = real_elapsed
        self.real_time_elapsed_pub.publish(re_msg)

        se_msg = Float64()
        se_msg.data = sim_elapsed
        self.sim_time_elapsed_pub.publish(se_msg)

        if real_elapsed > 0:
            rtf_msg = Float64()
            rtf_msg.data = sim_elapsed / real_elapsed
            self.rtf_pub.publish(rtf_msg)

        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)

        try:
            t = self.tf_buffer.lookup_transform(
                'local_origin', 'base_link', rclpy.time.Time()
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
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

        if self.state == State.LAP_COMPLETED:
            done_msg = Bool()
            done_msg.data = True
            self.phase3_done_pub.publish(done_msg)
            depth_error = self.target_depth - self.current_z
            fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
            Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))
            self._publish_thrusters(0.0, 0.0, Fz, 0.0, 0.0)
            return

        if self._start_yaw is None:
            self._start_yaw    = self.current_yaw
            self._prev_yaw     = self.current_yaw
            self._accumulated_yaw = 0.0
            self._lap_start_time  = now
            self._target_yaw = self.current_yaw
            self.get_logger().info(
                f"[WALKING_THE_NET] Strafing mode (lap tracking) initialised at yaw={math.degrees(self._start_yaw):.1f}°"
            )
        else:
            delta = _angle_diff(self.current_yaw, self._prev_yaw)
            if abs(delta) > math.radians(0.05):
                self._accumulated_yaw += delta
            self._prev_yaw = self.current_yaw

        yaw_acc_msg = Float64()
        yaw_acc_msg.data = self._accumulated_yaw
        self.yaw_accum_pub.publish(yaw_acc_msg)
        self._accumulated_dist += abs(self.current_vy) * dt

        acc_dist_msg = Float64()
        acc_dist_msg.data = self._accumulated_dist
        self.accumulated_dist_pub.publish(acc_dist_msg)

        if abs(self._accumulated_yaw) > 0.4:
            current_radius = self._accumulated_dist / abs(self._accumulated_yaw)
        elif self._last_R_calculated > 0.0:
            current_radius = self._last_R_calculated
        else:
            current_radius = 5.0

        radius_msg = Float64()
        radius_msg.data = current_radius
        self.current_radius_pub.publish(radius_msg)

        if self.state == State.WALKING_THE_NET:
            self._accumulated_walking_time += dt

        walking_time_msg = Float64()
        walking_time_msg.data = self._accumulated_walking_time
        self.walking_time_pub.publish(walking_time_msg)

        if self._first_walking_start_time is not None:
            if self.R_ref is None:
                if self._accumulated_walking_time < 30.0:
                    if self.state == State.WALKING_THE_NET:
                        self.radius_samples.append(current_radius)
                elif self.radius_samples:
                    self.R_ref = float(np.mean(self.radius_samples))
                    self.get_logger().info(f"[PHASE3] R_ref estimated: {self.R_ref:.2f} m (mean over {len(self.radius_samples)} samples from the first 30 active seconds).")

            if self.R_ref is not None and not self.in_cone_mode:
                if current_radius < 0.9 * self.R_ref:
                    if self.cone_transition_start_time is None:
                        self.cone_transition_start_time = now
                        self.get_logger().info(f"[CONE MODE CHECK] Rayon {current_radius:.2f}m est < 90% de R_ref ({0.9 * self.R_ref:.2f}m). Démarrage du timer de 3s...")
                    elif now - self.cone_transition_start_time > 3.0:
                        self.in_cone_mode = True
                        self.get_logger().info("[CONE MODE ACTIVATED] Transition vers le cône confirmée (rayon stable). Activation du contrôle de pitch !")
                else:
                    if self.cone_transition_start_time is not None:
                        self.get_logger().info("[CONE MODE CHECK] Le rayon a ré-augmenté. Timer annulé.")
                    self.cone_transition_start_time = None

            if self.in_cone_mode:
                if current_radius < 1.0:
                    if self.apex_condition_start_time is None:
                        self.apex_condition_start_time = now
                    elif now - self.apex_condition_start_time > 5.0:
                        self.get_logger().info("Apex reached and stable (radius < 1m for 5s). Ending mission and ascending.")
                        self.state = State.LAP_COMPLETED
                        self.target_depth = -2.0
                else:
                    self.apex_condition_start_time = None

        if self.R_ref is not None:
            r_ref_msg = Float64()
            r_ref_msg.data = self.R_ref
            self.r_ref_pub.publish(r_ref_msg)

        sonar_age = (
            now - self._last_sonar_time
            if self._last_sonar_time is not None
            else float('inf')
        )
        sonar_ok = sonar_age < LOST_WALL_TIMEOUT and self._raw_net_range is not None

        if sonar_ok:
            self.net_range = self._smoothed_net_range
        else:
            self.net_range = None
            self._smoothed_net_range = None

        walking_age = (
            now - self._walking_start_time
            if self._walking_start_time is not None
            else 0.0
        )
        past_grace = walking_age > LOST_WALL_GRACE_S

        perception_timeout = (
            self._last_perception_invalid_time is not None
            and (now - self._last_perception_invalid_time) >= LOST_WALL_TIMEOUT
        )
        lost_condition = (not sonar_ok or perception_timeout) and past_grace

        if self.state == State.WALKING_THE_NET and lost_condition:
            reason = "sonar signal lost" if not sonar_ok else "RANSAC invalid (perception_valid=False)"
            self.get_logger().warn(
                f"[LOST_WALL] {reason} (sonar_age={sonar_age:.2f}s) — entering recovery."
            )
            self.state = State.LOST_WALL
            self._pid_dist.reset()

        elif self.state == State.LOST_WALL and sonar_ok and self._perception_valid:
            self.get_logger().info("[WALKING_THE_NET] Sonar signal + RANSAC recovered — resuming orbit.")
            self.state = State.WALKING_THE_NET
            self._walking_start_time = now
            self._last_perception_invalid_time = None
            
        if self.state == State.LOST_WALL:
            self._do_lost_wall(dt)
        else:
            self._do_walking(dt)

        elapsed_since_start = now - self._lap_start_time if self._lap_start_time else 0.0
        if (elapsed_since_start > LAP_START_DELAY
                and abs(self._accumulated_yaw) >= LAP_YAW_THRESHOLD):

            self._last_R_calculated = self._accumulated_dist / (2 * math.pi)
            
            self.get_logger().info(
                f"[LAP_COMPLETED] Full orbit done at depth {self.target_depth}! "
                f"Accumulated yaw: {math.degrees(self._accumulated_yaw):.1f}°, "
                f"Estimated radius: {self._last_R_calculated:.2f}m"
            )
            
            self._accumulated_dist = 0.0
            
            if self.target_depth - DEPTH_STEP + 0.01 >= FINAL_DEPTH_LIMIT:
                self._descent_start_z = self.current_z
                self.target_depth -= DEPTH_STEP
                self._accumulated_yaw = 0.0
                self._lap_start_time = now
                self.apex_condition_start_time = None
                
                self._is_descending = True
                self._accumulated_dx_descent = 0.0

                cone_status = " (cone mode maintained)" if self.in_cone_mode else ""
                self.get_logger().info(f"[DESCENDING] New target depth: {self.target_depth}{cone_status}")
            else:
                self.state = State.LAP_COMPLETED
                self._publish_thrusters(0.0, 0.0, 0.0, 0.0)
                done_msg = Bool()
                done_msg.data = True
                self.phase3_done_pub.publish(done_msg)

    # ── Walking state ─────────────────────────────────────────────────────────

    def _do_walking(self, dt: float):

        if self._is_descending:
            self._accumulated_dx_descent += self.current_vx * dt
            if abs(self.current_z - self.target_depth) < 0.15:
                self._is_descending = False
                delta_z = abs(self.current_z - self._descent_start_z)
                if delta_z > 0.1:
                    # Inversion de signe corrigée selon la convention du robot
                    self.target_pitch = -math.atan2(self._accumulated_dx_descent, delta_z)
                    self.get_logger().info(f"[PITCH UPDATE] New pitch calculated: {math.degrees(self.target_pitch):.1f}° (Forward dx = {self._accumulated_dx_descent:.2f}m, dz = {delta_z:.2f}m)")

        depth_error = self.target_depth - self.current_z
        fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
        Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))

        if self.net_range is not None:
            dist_error = self.net_range - STANDOFF_DIST
            Fx = float(np.clip(self._pid_dist.compute(dist_error, dt), -MAX_DIST_CMD, MAX_DIST_CMD))
            self._last_fx = Fx
        else:
            Fx = self._last_fx

        if abs(depth_error) < 0.15:
            target_vy = float(ORBIT_DIRECTION * 0.25)
            vy_error = target_vy - self.current_vy
            Fy = float(np.clip(self._pid_velocity_sway.compute(vy_error, dt), -15.0, 15.0))
        else:
            sway_vel_error = 0.0 - self.current_vy
            Fy = float(np.clip(self._pid_velocity_sway.compute(sway_vel_error, dt), -10.0, 10.0))
        
        self._smoothed_yaw_error += self._yaw_ema_alpha * (self.net_angle_error - self._smoothed_yaw_error)
        yaw_error = self._smoothed_yaw_error
        Mz_raw = float(np.clip(self._pid_yaw.compute(yaw_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD))
        Mz_delta = np.clip(Mz_raw - self._last_Mz, -MZ_RATE_LIMIT, MZ_RATE_LIMIT)
        Mz = self._last_Mz + Mz_delta
        self._last_Mz = Mz

        self.pitch_error = self.target_pitch - self.current_pitch
        if self.in_cone_mode:
            My = float(np.clip(self._pid_pitch.compute(self.pitch_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD))
        else:
            My = 0.0

        total_cmd = abs(Fx) + abs(Fz) + abs(Mz) + abs(My)
        if total_cmd + abs(Fy) > MAX_INDIVIDUAL_THRUST:
            max_fy = max(0.0, MAX_INDIVIDUAL_THRUST - total_cmd)
            Fy = float(np.clip(Fy, -max_fy, max_fy))

        self._publish_thrusters(Fx, Fy, Fz, Mz, My)
        self._publish_diagnostics(Fx, Fy, Fz, Mz, depth_error, 
                                  self.net_range - STANDOFF_DIST if self.net_range else 0.0, 
                                  yaw_error, My)

    def _do_lost_wall(self, dt: float):
        depth_error = self.target_depth - self.current_z
        fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
        Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))

        Fx = -5.0
        Fy = 0.0
        Mz = float(ORBIT_DIRECTION * 0.5)

        self._publish_thrusters(Fx, Fy, Fz, Mz, 0.0)


    # ── Diagnostic publisher ──────────────────────────────────────────────────

    def _publish_diagnostics(
        self,
        Fx: float, Fy: float, Fz: float, Mz: float,
        depth_error: float,
        dist_error: float,
        yaw_error: float,
        My: float = 0.0
    ):
        def _pub(publisher, value: float):
            m = Float64()
            m.data = float(value)
            publisher.publish(m)

        if self.net_range is not None:
            _pub(self.wall_dist_pub,       self.net_range)
            _pub(self.wall_dist_error_pub, dist_error)
        _pub(self.depth_pub,       self.current_z)
        _pub(self.depth_error_pub, depth_error)
        _pub(self.yaw_pub,         self.current_yaw)
        _pub(self.yaw_error_pub,   yaw_error)
        _pub(self.target_pitch_pub, self.target_pitch)

        _pub(self.cmd_fx_pub, Fx)
        _pub(self.cmd_fy_pub, Fy)
        _pub(self.cmd_fz_pub, Fz)
        _pub(self.cmd_mz_pub, Mz)
        _pub(self.cmd_my_pub, My)
        
    # ── Thruster allocation ───────────────────────────────────────────────────

    def _publish_thrusters(self, Fx: float, Fy: float, Fz: float, Mz: float, My: float = 0.0):
        wrench_msg = Wrench()
        wrench_msg.force.x = float(Fx)
        wrench_msg.force.y = float(Fy)
        wrench_msg.force.z = float(Fz)
        wrench_msg.torque.y = float(My)
        wrench_msg.torque.z = float(Mz)
        self.wrench_pub.publish(wrench_msg)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Phase3InspectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()