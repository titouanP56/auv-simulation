

import math
import struct
import collections
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Wrench
import tf2_ros

# ── Physical constants ─────────────────────────

TARGET_DEPTH          = -2.0   
STANDOFF_DIST         = 1.5    
CONTROL_RATE_HZ       = 10.0   

KP_DEPTH              = 10.0
KI_DEPTH              = 0.2
KD_DEPTH              = 0.2
BUOYANCY_COMP         = 3.0    

KP_DIST               = 4.0
KI_DIST               = 0.2
KD_DIST               = 1.5

KP_VEL_SWAY           = 15.0
KI_VEL_SWAY           = 2.0
KD_VEL_SWAY           = 0.5

KP_YAW                = 5.0
KI_YAW                = 0.02
KD_YAW                = 1.0

KP_PITCH              = 10.0
KI_PITCH              = 0.2
KD_PITCH              = 2.0

MAX_DEPTH_CMD         = 15.0   
MAX_DIST_CMD          = 15.0   
MAX_YAW_CMD           = 10.0   
MAX_INDIVIDUAL_THRUST = 40.0   


MZ_RATE_LIMIT         = 3.0     # Max change in Mz per control step [N·m/step]

ORBIT_DIRECTION       = 1  
PERCENTILE_FRACTION   = 0.10 
MEDIAN_WINDOW         = 7      
SPIKE_THRESHOLD       = 0.5    

LAP_YAW_THRESHOLD     = 2.0 * math.pi  
LAP_START_DELAY       = 2.0             
LOST_WALL_TIMEOUT     = 2.0             
LOST_WALL_GRACE_S     = 2.0             
RECOVERY_YAW_CMD      = 4.0 

DEPTH_STEP            = 0.5             
FINAL_DEPTH_LIMIT     = -6.0            


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


class MovingMedian:
    """Fixed-size circular buffer implementing a moving-median filter."""

    def __init__(self, window: int = 7):
        self._window = window
        self._buf: collections.deque = collections.deque(maxlen=window)

    def update(self, value: float) -> float:
        self._buf.append(value)
        return float(np.median(list(self._buf)))

    def is_ready(self) -> bool:
        return len(self._buf) >= self._window // 2 + 1


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


# ── Sonar processing ───────────────────────────────────────────────────────────

def _extract_wall_distance(msg: PointCloud2) -> tuple[float | None, float, float | None, float | None]:
    if msg.width * msg.height == 0:
        return None, 0.0, None, None

    # Déballage rapide du buffer binaire en tableau de float32
    payload = np.frombuffer(msg.data, dtype=np.float32)
    floats_per_point = msg.point_step // 4
    points = payload.reshape(-1, floats_per_point)
    
    px = points[:, 0]
    py = points[:, 1]
    pz = points[:, 2]

    # Calculs vectoriels des distances et angles
    dists = np.sqrt(px**2 + py**2 + pz**2)
    angles = np.arctan2(py, px)

    # Filtrage par masque booléen NumPy (équivalent aux filtres d'origine)
    valid_mask = (dists >= 0.3) & (dists <= 7.0) & (np.abs(angles) <= 1.57) & np.isfinite(dists)
    
    if not np.any(valid_mask):
        return None, 0.0, None, None

    valid_dists = dists[valid_mask]
    valid_angles = angles[valid_mask]
    valid_pz = pz[valid_mask]

    # Tri pour extraire le percentile inférieur (points les plus proches)
    sort_idx = np.argsort(valid_dists)
    n_use = max(1, int(len(sort_idx) * PERCENTILE_FRACTION))
    closest_idx = sort_idx[:n_use]

    avg_dist = float(np.mean(valid_dists[closest_idx]))
    avg_angle = float(np.mean(valid_angles[closest_idx]))

    # Séparation Top / Bottom pour le contrôle du Pitch en mode cône
    top_mask = valid_pz > 0
    bot_mask = valid_pz < 0

    dist_top = float(np.median(valid_dists[top_mask][:n_use])) if np.any(top_mask) else None
    dist_bottom = float(np.median(valid_dists[bot_mask][:n_use])) if np.any(bot_mask) else None

    return avg_dist, avg_angle, dist_top, dist_bottom


# ── Main node ──────────────────────────────────────────────────────────────────

class Phase3InspectionNode(Node):
    """
    ROS 2 Node for the Inspection Phase (Phase 3) of the AUV mission.
    
    This node controls the robot to orbit ("walk") along the net surface at a fixed 
    standoff distance and depth. It uses PID controllers to maintain depth, distance, 
    sway velocity, yaw, and pitch based on Sonoptix sonar data. It tracks the 
    completed laps and can transition to cone mode when reaching the bottom of the net.
    """

    def __init__(self):
        super().__init__('phase3_inspection')

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
            PointCloud2, '/sonoptix/points', self._sonoptix_cb, best_effort_qos
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
        self.wall_dist_pub       = self.create_publisher(Float64, '/phase3/wall_distance',    10)
        self.wall_dist_error_pub = self.create_publisher(Float64, '/phase3/wall_dist_error',  10)

        self.yaw_pub             = self.create_publisher(Float64, '/phase3/yaw',              10)
        self.yaw_error_pub       = self.create_publisher(Float64, '/phase3/yaw_error',        10)

        self.depth_pub           = self.create_publisher(Float64, '/phase3/depth',            10)
        self.depth_error_pub     = self.create_publisher(Float64, '/phase3/depth_error',      10)

        self.cmd_fx_pub          = self.create_publisher(Float64, '/phase3/cmd_Fx',           10)
        self.cmd_fy_pub          = self.create_publisher(Float64, '/phase3/cmd_Fy',           10)
        self.cmd_fz_pub          = self.create_publisher(Float64, '/phase3/cmd_Fz',           10)
        self.cmd_mz_pub          = self.create_publisher(Float64, '/phase3/cmd_Mz',           10)
        self.cmd_my_pub          = self.create_publisher(Float64, '/phase3/cmd_My',           10)
        
        self.yaw_accum_pub       = self.create_publisher(Float64, '/phase3/yaw_accumulated',  10)
        self.current_radius_pub  = self.create_publisher(Float64, '/phase3/current_radius',   10)
        self.r_ref_pub           = self.create_publisher(Float64, '/phase3/r_ref',            10)
        self.accumulated_dist_pub = self.create_publisher(Float64, '/phase3/accumulated_dist', 10)
        self.walking_time_pub    = self.create_publisher(Float64, '/phase3/walking_time',     10)

        self.real_time_elapsed_pub = self.create_publisher(Float64, '/phase3/real_time_elapsed', 10)
        self.sim_time_elapsed_pub  = self.create_publisher(Float64, '/phase3/sim_time_elapsed',  10)
        self.rtf_pub               = self.create_publisher(Float64, '/phase3/real_time_factor', 10)

        # ── TF2 & Relative Télémétrie ─────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.relative_pose_pub = self.create_publisher(PoseStamped, '/phase3/relative_pose', 10)

        self.state: str = State.WAITING

        self._have_odom = False
        self.current_z   = 0.0
        self.current_yaw = 0.0
        self.current_vyaw = 0.0
        self.current_vy  = 0.0

        self.target_depth = TARGET_DEPTH

        self._raw_net_range: float | None = None 
        self.net_range: float | None = None        
        self._prev_net_range: float | None = None
        self._sonar_median = MovingMedian(MEDIAN_WINDOW)
        self._last_sonar_time: float | None = None
        self.net_angle_error: float = 0.0
        self._smoothed_yaw_error: float = 0.0  # EMA-filtered yaw error
        self._last_Mz: float = 0.0             # For rate limiting
        self.dist_top: float | None = None
        self.dist_bottom: float | None = None

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
        self._last_loop_time: float | None = None
        self._target_yaw: float | None = None   # Updated continuously using DVL lateral velocity
        self._yaw_error_prev = 0.0

        self._start_real_time = time.time()
        self._start_sim_time  = self.get_clock().now().nanoseconds * 1e-9

        # ── Control timer ────────────────────────────────────────────────────
        self.create_timer(self._dt, self._control_loop)
        self.get_logger().info(f"Phase3InspectionNode started — state: WAITING (rate={_rate:.0f} Hz, yaw_ema_alpha={self._yaw_ema_alpha:.2f})")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self.current_vy   = msg.twist.twist.linear.y
        self._have_odom   = True

    def _sonoptix_cb(self, msg: PointCloud2):
        if self.state not in (State.WALKING_THE_NET, State.LOST_WALL):
            return

        raw, angle, dist_top, dist_bottom = _extract_wall_distance(msg)
        if raw is None:
            self.get_logger().debug("[SONAR] Valid frame received but yielded None")
            self._last_sonar_time = self.get_clock().now().nanoseconds * 1e-9
            return

        if self._prev_net_range is not None:
            jump = abs(raw - self._prev_net_range)
            if jump > SPIKE_THRESHOLD:
                self.get_logger().debug(
                    f"[SONAR] Spike rejected: {raw:.3f}m (prev={self._prev_net_range:.3f}m, Δ={jump:.3f}m)"
                )
                raw = self._prev_net_range

        filtered = self._sonar_median.update(raw)

        self._prev_net_range = filtered
        self._raw_net_range  = filtered
        self._last_sonar_time = self.get_clock().now().nanoseconds * 1e-9
        self.net_angle_error = angle
        self.dist_top = dist_top
        self.dist_bottom = dist_bottom

    def _phase2_done_cb(self, msg: Bool):
        if msg.data and self.state == State.WAITING:
            self.get_logger().info("[PHASE3] Phase 2 done received — activating inspection orbit.")
            self.state = State.WALKING_THE_NET
            self._walking_start_time = self.get_clock().now().nanoseconds * 1e-9
            self._first_walking_start_time = self._walking_start_time
            self._last_sonar_time = None
            self._raw_net_range   = None
            self._prev_net_range  = None
            self._sonar_median    = MovingMedian(MEDIAN_WINDOW)

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

        # ── Real-time vs Simulation-time metrics ─────────────────────────────
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
                    self.get_logger().info(f"[PHASE3] R_ref calculated: {self.R_ref:.2f} m (average over {len(self.radius_samples)} samples from the first 30 active seconds).")

            if self.R_ref is not None and not self.in_cone_mode:
                if current_radius < 0.9 * self.R_ref:
                    if self.cone_transition_start_time is None:
                        self.cone_transition_start_time = now
                    elif now - self.cone_transition_start_time > 3.0:
                        self.in_cone_mode = True
                        self.get_logger().info("Cone transition detected")
                else:
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
        self.net_range = self._raw_net_range if sonar_ok else None

        if self.net_range is not None:
            wd_msg = Float64()
            wd_msg.data = self.net_range
            self.wall_dist_pub.publish(wd_msg)

        walking_age = (
            now - self._walking_start_time
            if self._walking_start_time is not None
            else 0.0
        )
        past_grace = walking_age > LOST_WALL_GRACE_S

        if self.state == State.WALKING_THE_NET and not sonar_ok and past_grace:
            self.get_logger().warn(
                f"[LOST_WALL] Sonar signal lost (age={sonar_age:.2f}s) — entering recovery."
            )
            self.state = State.LOST_WALL
            self._pid_dist.reset()

        elif self.state == State.LOST_WALL and sonar_ok:
            self.get_logger().info("[WALKING_THE_NET] Sonar recovered — resuming orbit.")
            self.state = State.WALKING_THE_NET
            self._walking_start_time = now  
            
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
                f"Calculated radius: {self._last_R_calculated:.2f}m"
            )
            
            self._accumulated_dist = 0.0
            
            if self.target_depth - DEPTH_STEP + 0.01 >= FINAL_DEPTH_LIMIT:
                self.target_depth -= DEPTH_STEP
                self._accumulated_yaw = 0.0
                self._lap_start_time = now
                self.apex_condition_start_time = None
                cone_status = " (Cone mode maintained)" if self.in_cone_mode else ""
                self.get_logger().info(f"[DESCENDING] New target depth: {self.target_depth}{cone_status}")
            else:
                self.state = State.LAP_COMPLETED
                self._publish_thrusters(0.0, 0.0, 0.0, 0.0)
                done_msg = Bool()
                done_msg.data = True
                self.phase3_done_pub.publish(done_msg)

    # ── Walking state ─────────────────────────────────────────────────────────

    def _do_walking(self, dt: float):

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
        
        # Smooth yaw error with EMA to avoid step-change spikes from 2 Hz sonar
        self._smoothed_yaw_error += self._yaw_ema_alpha * (self.net_angle_error - self._smoothed_yaw_error)
        yaw_error = self._smoothed_yaw_error
        Mz_raw = float(np.clip(self._pid_yaw.compute(yaw_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD))
        # Rate-limit Mz to prevent sharp torque spikes
        Mz_delta = np.clip(Mz_raw - self._last_Mz, -MZ_RATE_LIMIT, MZ_RATE_LIMIT)
        Mz = self._last_Mz + Mz_delta
        self._last_Mz = Mz

        if self.in_cone_mode and self.dist_top is not None and self.dist_bottom is not None:
            pitch_error = self.dist_top - self.dist_bottom
            My = float(np.clip(self._pid_pitch.compute(pitch_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD))
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

        sway_vel_error = 0.0 - self.current_vy
        Fy = float(np.clip(self._pid_velocity_sway.compute(sway_vel_error, dt), -10.0, 10.0))

        Mz = float(ORBIT_DIRECTION * RECOVERY_YAW_CMD)

        self._publish_thrusters(0.0, Fy, Fz, Mz, 0.0)


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
