"""
phase3_inspection.py
=====================
AUV Net Inspection — Phase 3: Reactive 360° Wall-Following Orbit

Strategy
--------
  * Waits for /mission/phase2_done → True
  * Performs a full 360° lap at constant depth (TARGET_DEPTH = -2.0 m)
  * Maintains STANDOFF_DIST = 1.5 m from the net using the Sonoptix sonar
    without any pre-computed trajectory.

Control axes (body frame)
  Fz  – depth PID
  Fx  – constant forward surge (orbit speed)
  Fy  – sway PID to regulate sonar distance (wall following)
  Mz  – yaw PID to keep robot tangent to the net

Sonar robustness
  • Rejects points outside [SONAR_MIN_RANGE, SONAR_MAX_RANGE]
  • Rejects low-intensity points (if intensity field available)
  • Uses the mean of the 10 % closest valid points as wall distance estimate
  • Moving-median filter (window = MEDIAN_WINDOW) on history of estimates
  • Spike rejection: ignores jumps > SPIKE_THRESHOLD m between cycles

Lap completion
  • Tracks cumulative yaw integrated from odometry (no GPS)
  • When |cumulative_yaw| ≥ 2π the lap is declared complete
  • Publishes True on /mission/phase3_done

State machine
  WAITING → WALKING_THE_NET → LOST_WALL → LAP_COMPLETED
"""

import math
import struct
import collections
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import tf2_ros

# ── Physical constants (identical to net_approach.py) ─────────────────────────

TARGET_DEPTH          = -2.0   
STANDOFF_DIST         = 1.5    
CONTROL_RATE_HZ       = 20.0   

# Gains optimisés pour la stabilité et la réactivité
KP_DEPTH              = 150.0
KI_DEPTH              = 10.0
KD_DEPTH              = 50.0
BUOYANCY_COMP         = -10.0    

# PID Distance (Fx - Surge) : Maintient le robot à 1.5m
KP_DIST               = 12.0
KI_DIST               = 0.2
KD_DIST               = 1.0

# PID Vitesse Latérale (Fy - Sway) : Pour atteindre 0.2 m/s malgré la traînée
KP_VEL_SWAY           = 40.0
KI_VEL_SWAY           = 2.0
KD_VEL_SWAY           = 0.5

# PID Alignement (Mz - Yaw) : Verrouillage sur le point le plus proche
KP_YAW                = 10.0
KI_YAW                = 0.02
KD_YAW                = 3.0  # Damping augmenté pour éviter les oscillations

MAX_DEPTH_CMD         = 120.0   
MAX_DIST_CMD          = 15.0   
MAX_YAW_CMD           = 20.0   
MAX_INDIVIDUAL_THRUST = 40.0   

ORBIT_DIRECTION       = 1  # 1: CCW, -1: CW
PERCENTILE_FRACTION   = 0.10 #
MEDIAN_WINDOW         = 7      # moving-median filter size [cycles]
SPIKE_THRESHOLD       = 0.5    # [m]   max allowed jump per cycle

# Lost-wall recovery

LAP_YAW_THRESHOLD     = 2.0 * math.pi   # Un tour complet
LAP_START_DELAY       = 2.0             # Délai de sécurité au début (en secondes)
LOST_WALL_TIMEOUT     = 2.0             # Temps avant de déclarer le mur "perdu"
LOST_WALL_GRACE_S     = 2.0             # Délai de grâce au lancement
RECOVERY_YAW_CMD      = 4.0 # [s] Temps d'attente avant de commencer à compter le tour (yaw)

DEPTH_STEP            = 1.0             # Incrément de profondeur pour chaque palier
FINAL_DEPTH_LIMIT     = -6.0            # Profondeur finale d'arrêt de la mission

# ── Thruster allocation (identical to net_approach.py) ────────────────────────

THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]
SIN45 = 0.7071
LEVER = 0.1697

TAM = np.array([
    [ SIN45,  SIN45, -SIN45, -SIN45,  0.0,   0.0,   0.0,   0.0 ],
    [ SIN45, -SIN45,  SIN45, -SIN45,  0.0,   0.0,   0.0,   0.0 ],
    [ 0.0,    0.0,    0.0,    0.0,   -1.0,  1.0,   1.0,  -1.0 ],
    [ 0.0,    0.0,    0.0,    0.0,    0.218, 0.218, 0.218, 0.218],
    [ 0.0,    0.0,    0.0,    0.0,    0.12, -0.12,  0.12, -0.12 ],
    [ LEVER, -LEVER, -LEVER,  LEVER,  0.0,   0.0,   0.0,   0.0 ],
], dtype=float)
TAM_PINV = np.linalg.pinv(TAM)


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

def _extract_wall_distance(msg: PointCloud2) -> tuple[float | None, float]:
    """
    Extrait la distance ET l'angle du point le plus proche pour la perpendicularité.
    """
    field_map = {f.name: f for f in msg.fields}
    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return None, 0.0

    point_step, data = msg.point_step, msg.data
    x_off, y_off, z_off = field_map['x'].offset, field_map['y'].offset, field_map['z'].offset

    valid_points = []

    for i in range(msg.width * msg.height):
        base = i * point_step
        try:
            px = struct.unpack_from('f', data, base + x_off)[0]
            py = struct.unpack_from('f', data, base + y_off)[0]
            pz = struct.unpack_from('f', data, base + z_off)[0]
        except: continue

        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            continue

        # Filtrage Boresight large (90°) pour ne pas perdre le filet
        angle = math.atan2(py, px)
        dist = math.sqrt(px**2 + py**2 + pz**2)

        if dist < 0.3 or dist > 7.0 or abs(angle) > 1.57:
            continue

        valid_points.append((dist, angle))

    if not valid_points:
        return None, 0.0

    # TRI PAR DISTANCE : On veut s'aligner sur la partie la plus proche
    valid_points.sort(key=lambda p: p[0])
    
    # On prend les 10% les plus proches
    n_use = max(1, int(len(valid_points) * PERCENTILE_FRACTION))
    closest_points = valid_points[:n_use]

    avg_dist = np.mean([p[0] for p in closest_points])
    avg_angle = np.mean([p[1] for p in closest_points])

    return float(avg_dist), float(avg_angle)


# ── Main node ──────────────────────────────────────────────────────────────────

class Phase3InspectionNode(Node):
    """
    Phase 3 — Reactive 360° wall-following inspection.

    Activated by /mission/phase2_done.  Publishes /mission/phase3_done
    after one complete orbit detected by cumulative yaw integration.
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
        # Changed to VOLATILE so tools like Foxglove and ros2 topic pub can trigger it reliably
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
        self._thrust_pubs = [
            self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            for i in range(1, 9)
        ]
        self.phase3_done_pub = self.create_publisher(Bool, '/mission/phase3_done', 10)
        self.phase_pub       = self.create_publisher(String, '/mission/phase', 10)

        # ── Diagnostic topics (Foxglove) ─────────────────────────────────────
        # Distance au filet
        self.wall_dist_pub       = self.create_publisher(Float64, '/phase3/wall_distance',    10)
        self.wall_dist_error_pub = self.create_publisher(Float64, '/phase3/wall_dist_error',  10)
        # Profondeur
        self.depth_pub           = self.create_publisher(Float64, '/phase3/depth',            10)
        self.depth_error_pub     = self.create_publisher(Float64, '/phase3/depth_error',      10)
        # Cap (yaw)
        self.yaw_pub             = self.create_publisher(Float64, '/phase3/yaw',              10)
        self.yaw_error_pub       = self.create_publisher(Float64, '/phase3/yaw_error',        10)
        # Efforts de commande pré-TAM
        self.cmd_fx_pub          = self.create_publisher(Float64, '/phase3/cmd_Fx',           10)
        self.cmd_fy_pub          = self.create_publisher(Float64, '/phase3/cmd_Fy',           10)
        self.cmd_fz_pub          = self.create_publisher(Float64, '/phase3/cmd_Fz',           10)
        self.cmd_mz_pub          = self.create_publisher(Float64, '/phase3/cmd_Mz',           10)
        # Yaw cumulé (progression du tour)
        self.yaw_accum_pub       = self.create_publisher(Float64, '/phase3/yaw_accumulated',  10)

        # ── TF2 & Relative Télémétrie ─────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.relative_pose_pub = self.create_publisher(PoseStamped, '/phase3/relative_pose', 10)

        # ── Internal state ───────────────────────────────────────────────────

        # Mission state
        self.state: str = State.WAITING

        # Odometry
        self._have_odom = False
        self.current_z   = 0.0
        self.current_yaw = 0.0
        self.current_vyaw = 0.0
        self.current_vy  = 0.0    # lateral sway velocity (body y)

        self.target_depth = TARGET_DEPTH  # Set initial depth target

        # Sonar
        self._raw_net_range: float | None = None   # latest raw estimate this cycle
        self.net_range: float | None = None         # filtered estimate
        self._prev_net_range: float | None = None   # for spike rejection
        self._sonar_median = MovingMedian(MEDIAN_WINDOW)
        self._last_sonar_time: float | None = None
        self.net_angle_error: float = 0.0

        # PID controllers
        self._pid_depth = PID(KP_DEPTH, KI_DEPTH, KD_DEPTH, integral_limit=50.0)
        self._pid_dist  = PID(KP_DIST,  KI_DIST,  KD_DIST,  integral_limit=10.0)
        self._pid_yaw   = PID(KP_YAW,   KI_YAW,   KD_YAW,   integral_limit=10.0)
        
        self._pid_velocity_sway = PID(15.0, 0.5, 0.0, integral_limit=20.0)

        self._last_fx = 0.0

        # Yaw-accumulation for lap tracking
        self._start_yaw: float | None = None
        self._prev_yaw: float | None = None
        self._accumulated_yaw = 0.0
        self._lap_start_time: float | None = None
        self._walking_start_time: float | None = None  # set when WALKING_THE_NET begins

        # Timing
        self._dt = 1.0 / CONTROL_RATE_HZ
        self._last_loop_time: float | None = None

        # Yaw target for tracing the circle (center-facing)
        self._target_yaw: float | None = None   # Updated continuously using DVL lateral velocity
        self._yaw_error_prev = 0.0

        # ── Control timer ────────────────────────────────────────────────────
        self.create_timer(self._dt, self._control_loop)
        self.get_logger().info("Phase3InspectionNode started — state: WAITING")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self.current_vy   = msg.twist.twist.linear.y
        self._have_odom   = True

    def _sonoptix_cb(self, msg: PointCloud2):
        """Process Sonoptix PointCloud2 with full robustness pipeline.
        Accepted in WALKING_THE_NET and LOST_WALL states only.
        """
        if self.state not in (State.WALKING_THE_NET, State.LOST_WALL):
            return

        raw, angle = _extract_wall_distance(msg)
        if raw is None:
            # We explicitly update the timestamp anyway, because receiving
            # an "empty" pointcloud proves the sensor is alive, just seeing nothing.
            # (Allows robot to keep swaying to find the net rather than aborting)
            self.get_logger().debug("[SONAR] Valid frame received but yielded None")
            self._last_sonar_time = self.get_clock().now().nanoseconds * 1e-9
            return

        # ── 1. Spike rejection ────────────────────────────────────────────
        if self._prev_net_range is not None:
            jump = abs(raw - self._prev_net_range)
            if jump > SPIKE_THRESHOLD:
                self.get_logger().debug(
                    f"[SONAR] Spike rejected: {raw:.3f}m (prev={self._prev_net_range:.3f}m, Δ={jump:.3f}m)"
                )
                raw = self._prev_net_range  # keep previous value

        # ── 2. Moving-median filter ───────────────────────────────────────
        filtered = self._sonar_median.update(raw)

        self._prev_net_range = filtered
        self._raw_net_range  = filtered
        self._last_sonar_time = self.get_clock().now().nanoseconds * 1e-9
        self.net_angle_error = angle

    def _phase2_done_cb(self, msg: Bool):
        if msg.data and self.state == State.WAITING:
            self.get_logger().info("[PHASE3] Phase 2 done received — activating inspection orbit.")
            self.state = State.WALKING_THE_NET
            self._walking_start_time = self.get_clock().now().nanoseconds * 1e-9
            # Reset sonar history so stale Phase-2 timestamps don't cause
            # an immediate sonar_age > LOST_WALL_TIMEOUT at the first loop cycle.
            self._last_sonar_time = None
            self._raw_net_range   = None
            self._prev_net_range  = None
            self._sonar_median    = MovingMedian(MEDIAN_WINDOW)  # fresh filter

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._have_odom:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = self._dt  # nominal; use fixed rate for PID stability
        if self._last_loop_time is not None:
            real_dt = now - self._last_loop_time
            if 0.005 < real_dt < 0.5:
                dt = real_dt
        self._last_loop_time = now

        # Publish phase label
        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)

        # ── Relative Télémétrie ───────────────────────────────────────────
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
            pass  # Fail silently if the transform isn't available yet

        # ── State dispatch ────────────────────────────────────────────────
        if self.state == State.WAITING:
            return

        if self.state == State.LAP_COMPLETED:
            done_msg = Bool()
            done_msg.data = True
            self.phase3_done_pub.publish(done_msg)
            self._publish_thrusters(0.0, 0.0, 0.0, 0.0)
            return

        # ── Yaw-accumulation for lap tracking ─────────────────────────────
        if self._start_yaw is None:
            # First active cycle: initialise lap tracking
            self._start_yaw    = self.current_yaw
            self._prev_yaw     = self.current_yaw
            self._accumulated_yaw = 0.0
            self._lap_start_time  = now
            # In Strafing mode, the robot starts already facing the net.
            self._target_yaw = self.current_yaw
            self.get_logger().info(
                f"[WALKING_THE_NET] Strafing mode (lap tracking) initialised at yaw={math.degrees(self._start_yaw):.1f}°"
            )
        else:
            # Integrate yaw delta
            delta = _angle_diff(self.current_yaw, self._prev_yaw)
            # Filter tiny numerical noise
            if abs(delta) > math.radians(0.05):
                self._accumulated_yaw += delta
            self._prev_yaw = self.current_yaw

        # Publish accumulated yaw for monitoring
        yaw_acc_msg = Float64()
        yaw_acc_msg.data = self._accumulated_yaw
        self.yaw_accum_pub.publish(yaw_acc_msg)

        # ── Evaluate sonar availability ────────────────────────────────────
        sonar_age = (
            now - self._last_sonar_time
            if self._last_sonar_time is not None
            else float('inf')
        )
        sonar_ok = sonar_age < LOST_WALL_TIMEOUT and self._raw_net_range is not None
        self.net_range = self._raw_net_range if sonar_ok else None

        # Publish wall distance diagnostic
        if self.net_range is not None:
            wd_msg = Float64()
            wd_msg.data = self.net_range
            self.wall_dist_pub.publish(wd_msg)

        # ── State transition: WALKING ↔ LOST_WALL ─────────────────────────
        # Startup grace: do not trigger LOST_WALL until sonar has had time to
        # deliver the first frame (avoids immediate LOST_WALL on activation).
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
            self._walking_start_time = now  # reset grace for the resumed phase

        # ── Compute control commands ───────────────────────────────────────
        if self.state == State.LOST_WALL:
            self._do_lost_wall(dt)
        else:
            self._do_walking(dt)

        # ── Lap completion check ───────────────────────────────────────────
        elapsed_since_start = now - self._lap_start_time if self._lap_start_time else 0.0
        if (elapsed_since_start > LAP_START_DELAY
                and abs(self._accumulated_yaw) >= LAP_YAW_THRESHOLD):
            self.get_logger().info(
                f"[LAP_COMPLETED] Full orbit done at depth {self.target_depth}! "
                f"Accumulated yaw: {math.degrees(self._accumulated_yaw):.1f}°"
            )
            
            # Check if we can descend to a new plateau
            if self.target_depth - DEPTH_STEP + 0.01 >= FINAL_DEPTH_LIMIT:
                # Adding +0.01 margin to avoid floating point issues (e.g. -2.0 - 4.0 = -6.0 >= -6.0)
                self.target_depth -= DEPTH_STEP
                self._accumulated_yaw = 0.0
                self._lap_start_time = now # reset delay to avoid immediate re-trigger
                self.get_logger().info(f"[DESCENDING] New target depth: {self.target_depth}")
            else:
                self.state = State.LAP_COMPLETED
                self._publish_thrusters(0.0, 0.0, 0.0, 0.0)
                done_msg = Bool()
                done_msg.data = True
                self.phase3_done_pub.publish(done_msg)

    # ── Walking state ─────────────────────────────────────────────────────────

    def _do_walking(self, dt: float):
        """
        Calcul des commandes pour l'inspection frontale.
        """
        # 1. Profondeur (Fz)
        depth_error = self.target_depth - self.current_z
        fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
        Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))

        # 2. Distance au filet (Fx - Surge) : Robot face au filet
        if self.net_range is not None:
            dist_error = self.net_range - STANDOFF_DIST
            Fx = float(np.clip(self._pid_dist.compute(dist_error, dt), -MAX_DIST_CMD, MAX_DIST_CMD))
            self._last_fx = Fx
        else:
            Fx = self._last_fx

        # 3. Vitesse de progression (Fy - Sway) : Objectif 0.2 m/s
        # On ne bouge latéralement que si la profondeur est atteinte (marge de 15cm)
        if abs(depth_error) < 0.15:
            target_vy = float(ORBIT_DIRECTION * 0.30)
            vy_error = target_vy - self.current_vy
            Fy = float(np.clip(self._pid_velocity_sway.compute(vy_error, dt), -15.0, 15.0))
        else:
            # On descend sans progresser autour du filet, pour ne pas dévier de la trajectoire idéale
            sway_vel_error = 0.0 - self.current_vy
            Fy = float(np.clip(self._pid_velocity_sway.compute(sway_vel_error, dt), -10.0, 10.0))
            
            # On empêche de compter ce tour en réinitialisant le délai
            # (Pour utiliser l'horloge de façon propre on peut stocker le temps mais "reset" le yaw)
            

        # 4. Perpendicularité (Mz - Yaw) : S'aligner sur l'angle des points les plus proches
        # net_angle_error est maintenant l'angle moyen des points proches uniquement
        yaw_error = self.net_angle_error
        Mz = float(np.clip(self._pid_yaw.compute(yaw_error, dt), -MAX_YAW_CMD, MAX_YAW_CMD))

        self._publish_thrusters(Fx, Fy, Fz, Mz)
        self._publish_diagnostics(Fx, Fy, Fz, Mz, depth_error, 
                                  self.net_range - STANDOFF_DIST if self.net_range else 0.0, 
                                  yaw_error)

    def _do_lost_wall(self, dt: float):
        """
        En cas de perte du filet : on arrête la progression latérale et on pivote
        doucement pour retrouver la paroi avec le sonar frontal.
        """
        depth_error = self.target_depth - self.current_z
        fz_raw = self._pid_depth.compute(depth_error, dt) - BUOYANCY_COMP
        Fz = float(np.clip(fz_raw, -MAX_DEPTH_CMD, MAX_DEPTH_CMD))

        # Freinage actif sur la vitesse latérale
        sway_vel_error = 0.0 - self.current_vy
        Fy = float(np.clip(self._pid_velocity_sway.compute(sway_vel_error, dt), -10.0, 10.0))

        # Rotation de recherche
        Mz = float(ORBIT_DIRECTION * RECOVERY_YAW_CMD)

        self._publish_thrusters(0.0, Fy, Fz, Mz)


    # ── Diagnostic publisher ──────────────────────────────────────────────────

    def _publish_diagnostics(
        self,
        Fx: float, Fy: float, Fz: float, Mz: float,
        depth_error: float,
        dist_error: float,
        yaw_error: float,
    ):
        """Publish all Foxglove monitoring topics in one call."""
        def _pub(publisher, value: float):
            m = Float64()
            m.data = float(value)
            publisher.publish(m)

        # Distance au filet
        if self.net_range is not None:
            _pub(self.wall_dist_pub,       self.net_range)
            _pub(self.wall_dist_error_pub, dist_error)          # net_range − 1.5 m

        # Profondeur
        _pub(self.depth_pub,       self.current_z)
        _pub(self.depth_error_pub, depth_error)                 # TARGET_DEPTH − current_z

        # Cap (yaw)
        _pub(self.yaw_pub,         self.current_yaw)
        _pub(self.yaw_error_pub,   yaw_error)                   # tangent_yaw − current_yaw [rad]

        # Efforts de commande pré-TAM
        _pub(self.cmd_fx_pub, Fx)
        _pub(self.cmd_fy_pub, Fy)
        _pub(self.cmd_fz_pub, Fz)
        _pub(self.cmd_mz_pub, Mz)

    # ── Thruster allocation ───────────────────────────────────────────────────

    def _publish_thrusters(self, Fx: float, Fy: float, Fz: float, Mz: float):
        """
        Mix generalised forces → individual thrust commands using TAM_PINV.

        tau = [Fx, Fy, Fz, Mx=0, My=0, Mz]
        """
        tau = np.array([Fx, Fy, Fz, 0.0, 0.0, Mz], dtype=float)
        raw_thrusts = TAM_PINV @ tau
        thrusts = np.clip(raw_thrusts, -MAX_INDIVIDUAL_THRUST, MAX_INDIVIDUAL_THRUST)

        for i, (thrust, coeff) in enumerate(zip(thrusts, THRUST_COEFFS)):
            msg = Float64()
            msg.data = float(thrust) * math.copysign(1.0, coeff)
            self._thrust_pubs[i].publish(msg)


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
