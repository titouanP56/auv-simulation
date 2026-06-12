

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped, PoseStamped, Wrench
from tf2_ros import TransformBroadcaster


# ── Constants ─────────────────────────────────────────────────────────────────

TARGET_DEPTH      = -2.0    # [m]
DEPTH_TOLERANCE   = 0.2    # [m]
DEPTH_HOLD_TIME   = 2.0     # [s]

YAW_TOLERANCE     = math.radians(10.0)
YAW_HOLD_TIME     = 1.0     # [s]

STANDOFF_DIST     = 1.5     # [m]
APPROACH_TOL      = 0.10    # [m]
STABILIZE_TIME    = 3.0     # [s]

KP_DEPTH  = 15.0
BUOYANCY_COMPENSATION = 3.0
KP_YAW    = 5.0
KD_YAW    = 2.0
KP_SURGE  = 6.0

MAX_DEPTH_CMD   = 20.0
MAX_YAW_CMD     = 40.0
MAX_SURGE_CMD   = 25.0

PING360_IGNORE_THRESHOLD = 0.95
SONOPTIX_BORESIGHT_HALF_ANGLE = math.radians(20.0)

CONTROL_RATE_HZ = 10.0
SCAN_ACCUMULATION_TIME = 4.0   # [s] wait for multiple Ping360 scans before choosing direction

MAX_INDIVIDUAL_THRUST = 20.0

class State:
    DESCENDING  = "DESCENDING"
    SCANNING    = "SCANNING"
    ALIGNING    = "ALIGNING"
    APPROACHING = "APPROACHING"
    STABILIZING = "STABILIZING"
    STANDOFF    = "STANDOFF"


# ── Helpers ───────────────────────────────────────────────────────────────────

def _yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))

def _min_sonoptix_range(msg: PointCloud2) -> float | None:
    field_map = {f.name: f for f in msg.fields}
    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return None

    x_off, y_off, z_off = field_map['x'].offset, field_map['y'].offset, field_map['z'].offset
    point_step, data = msg.point_step, msg.data

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
    ROS 2 Node for the Approach Phase (Phase 2) of the AUV mission.
    
    This node controls the robot to dive to a target depth, scan the environment 
    using a Ping360 sonar to find the net, align with it, and approach it until
    a specific standoff distance is reached (using a Sonoptix sonar). Finally, 
    it establishes a local coordinate frame (origin) for the next phase.
    """
    def __init__(self):
        super().__init__('phase2_mission')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.ping360_sub = self.create_subscription(LaserScan, '/ping360/scan', self._ping360_cb, best_effort_qos)
        self.sonoptix_sub = self.create_subscription(PointCloud2, '/sonoptix/points', self._sonoptix_cb, best_effort_qos)

        self.wrench_pub = self.create_publisher(Wrench, '/auv/command_wrench', 10)

        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.phase_pub   = self.create_publisher(String, '/mission/phase', 10)
        self.done_pub    = self.create_publisher(Bool, '/mission/phase2_done', latching_qos)
        self.origin_pub  = self.create_publisher(PoseStamped, '/mission/local_origin', latching_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.state: str = State.DESCENDING

        self._publish_done_status()
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.current_yaw, self.current_vyaw = 0.0, 0.0
        self.target_yaw = 0.0
        self.sonoptix_range: float | None = None
        self._stabilize_start_time: float | None = None

        self._depth_ok_since: float | None = None
        self._yaw_ok_since: float | None = None
        self._have_odom, self._have_scan, self._have_points = False, False, False
        self._scan_start_time: float | None = None
        self._scan_angles: list[float] = []

        self.declare_parameter('control_rate_hz', CONTROL_RATE_HZ)
        self.declare_parameter('test_enable_z', True)
        self.declare_parameter('test_enable_yaw', False)
        self.declare_parameter('test_enable_x', False)
        _rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / _rate, self._control_loop)
        self.get_logger().info(f"Phase2MissionNode started → state: DESCENDING (rate={_rate:.0f} Hz)")

    def _odom_cb(self, msg: Odometry):
        self.current_x    = msg.pose.pose.position.x
        self.current_y    = msg.pose.pose.position.y
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self._have_odom   = True

    def _ping360_cb(self, msg: LaserScan):
        if self.state != State.SCANNING:
            return

        min_r = min((r for r in msg.ranges if math.isfinite(r) and r < msg.range_max * PING360_IGNORE_THRESHOLD), default=float('inf'))
        if math.isinf(min_r):
            self.get_logger().warn("Ping360: no valid return found — retrying scan")
            return

        min_idx = msg.ranges.index(min_r)
        angle = msg.angle_min + min_idx * msg.angle_increment
        world_yaw = self.current_yaw + angle

        # Accumulate scan readings instead of using the first one
        self._scan_angles.append(world_yaw)
        self.get_logger().info(
            f"[SCANNING] Reading {len(self._scan_angles)}: nearest wall {min_r:.2f} m at yaw {math.degrees(world_yaw):.1f}°"
        )

    def _sonoptix_cb(self, msg: PointCloud2):
        if self.state in (State.APPROACHING, State.STABILIZING, State.STANDOFF):
            self.sonoptix_range = _min_sonoptix_range(msg)
            self._have_points = True

    def _control_loop(self):
        if not self._have_odom:
            return

        self._publish_state()
        self._publish_done_status()

        if self.state == State.DESCENDING:
            self._do_descending()
        elif self.state == State.SCANNING:
            self._do_scanning()
        elif self.state == State.ALIGNING:
            self._do_aligning()
        elif self.state == State.APPROACHING:
            self._do_approaching()
        elif self.state == State.STABILIZING:
            self._do_stabilizing()
        elif self.state == State.STANDOFF:
            self._do_standoff()

    def _do_descending(self):
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd = np.clip((KP_DEPTH * depth_error) - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)
        self._set_Fz(depth_cmd); self._set_Mz(0.0); self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(depth_error) < DEPTH_TOLERANCE:
            if self._depth_ok_since is None:
                self._depth_ok_since = now
            elif (now - self._depth_ok_since) >= DEPTH_HOLD_TIME:
                self.get_logger().info(f"[DESCENDING → SCANNING] Depth stable at {self.current_z:.2f} m")
                self.state = State.SCANNING
                self._depth_ok_since = None
        else:
            self._depth_ok_since = None

    def _do_scanning(self):
        depth_cmd = np.clip((KP_DEPTH * (TARGET_DEPTH - self.current_z)) - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)
        self._set_Fz(depth_cmd); self._set_Mz(0.0); self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._scan_start_time is None:
            self._scan_start_time = now

        elapsed = now - self._scan_start_time
        if elapsed >= SCAN_ACCUMULATION_TIME and len(self._scan_angles) >= 2:
            # Use median of accumulated angles for robustness
            import statistics
            self.target_yaw = statistics.median(self._scan_angles)
            self.get_logger().info(
                f"[SCANNING → ALIGNING] Median yaw from {len(self._scan_angles)} readings: "
                f"{math.degrees(self.target_yaw):.1f}°"
            )
            self.state = State.ALIGNING
            self._scan_angles.clear()
        elif elapsed >= SCAN_ACCUMULATION_TIME and len(self._scan_angles) == 1:
            self.target_yaw = self._scan_angles[0]
            self.get_logger().info(
                f"[SCANNING → ALIGNING] Single reading yaw: {math.degrees(self.target_yaw):.1f}°"
            )
            self.state = State.ALIGNING
            self._scan_angles.clear()

    def _do_aligning(self):
        depth_cmd = np.clip((KP_DEPTH * (TARGET_DEPTH - self.current_z)) - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)
        self._set_Fz(depth_cmd); self._set_Fx(0.0)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd = np.clip((KP_YAW * yaw_error) - (KD_YAW * self.current_vyaw), -MAX_YAW_CMD, MAX_YAW_CMD)
        self._set_Mz(mz_cmd)

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(yaw_error) < YAW_TOLERANCE:
            if self._yaw_ok_since is None:
                self._yaw_ok_since = now
            elif (now - self._yaw_ok_since) >= YAW_HOLD_TIME:
                self.get_logger().info(f"[ALIGNING → APPROACHING] Yaw error: {math.degrees(yaw_error):.2f}°")
                self.state = State.APPROACHING
                self._yaw_ok_since = None
        else:
            self._yaw_ok_since = None

    def _do_approaching(self):
        depth_cmd = np.clip((KP_DEPTH * (TARGET_DEPTH - self.current_z)) - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd = np.clip((KP_YAW * yaw_error) - (KD_YAW * self.current_vyaw), -MAX_YAW_CMD, MAX_YAW_CMD)
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is None:
            self._set_Fx(MAX_SURGE_CMD * 0.3)
            return

        surge_cmd = np.clip(KP_SURGE * (self.sonoptix_range - STANDOFF_DIST), 0.0, MAX_SURGE_CMD)
        self._set_Fx(surge_cmd)

        if (self.sonoptix_range - STANDOFF_DIST) <= APPROACH_TOL:
            self.get_logger().info(f"[APPROACHING → STABILIZING] Reached {self.sonoptix_range:.2f} m standoff! Stabilizing...")
            self.state = State.STABILIZING
            self._stabilize_start_time = self.get_clock().now().nanoseconds * 1e-9

    def _do_stabilizing(self):

        depth_cmd = np.clip((KP_DEPTH * (TARGET_DEPTH - self.current_z)) - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd = np.clip((KP_YAW * yaw_error) - (KD_YAW * self.current_vyaw), -MAX_YAW_CMD, MAX_YAW_CMD)
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is not None:
            surge_cmd = np.clip(KP_SURGE * (self.sonoptix_range - STANDOFF_DIST), -MAX_SURGE_CMD, MAX_SURGE_CMD)
            self._set_Fx(surge_cmd)
        else:
            self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._stabilize_start_time and (now - self._stabilize_start_time) >= STABILIZE_TIME:
            self.get_logger().info("[STABILIZING → STANDOFF] Robot stabilized. Defining local origin.")
            self._define_local_origin()
            self.state = State.STANDOFF

    def _define_local_origin(self):
        origin_x = self.current_x + STANDOFF_DIST * math.cos(self.target_yaw)
        origin_y = self.current_y + STANDOFF_DIST * math.sin(self.target_yaw)
        origin_z = self.current_z 
        origin_yaw = self.target_yaw + math.pi

        self.get_logger().info(f"[PHASE 3] Defined Local Origin at: x={origin_x:.2f}, y={origin_y:.2f}, z={origin_z:.2f}, yaw={math.degrees(origin_yaw):.1f}°")

        self._local_origin_transform = TransformStamped()
        self._local_origin_transform.header.frame_id = 'odom'
        self._local_origin_transform.child_frame_id = 'local_origin'
        self._local_origin_transform.transform.translation.x = origin_x
        self._local_origin_transform.transform.translation.y = origin_y
        self._local_origin_transform.transform.translation.z = origin_z
        
        qx, qy = 0.0, 0.0
        qz = math.sin(origin_yaw / 2.0)
        qw = math.cos(origin_yaw / 2.0)

        self._local_origin_transform.transform.rotation.x = qx
        self._local_origin_transform.transform.rotation.y = qy
        self._local_origin_transform.transform.rotation.z = qz
        self._local_origin_transform.transform.rotation.w = qw

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = origin_x
        pose_msg.pose.position.y = origin_y
        pose_msg.pose.position.z = origin_z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        self._local_origin_pose = pose_msg

    def _publish_done_status(self):
        done_msg = Bool()
        done_msg.data = (self.state == State.STANDOFF)
        self.done_pub.publish(done_msg)

    def _do_standoff(self):
        if hasattr(self, '_local_origin_pose'):
            self._local_origin_pose.header.stamp = self.get_clock().now().to_msg()
            self.origin_pub.publish(self._local_origin_pose)

        if hasattr(self, '_local_origin_transform'):
            self._local_origin_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self._local_origin_transform)

    def _set_Fz(self, fz: float): self._cmd_Fz = fz
    def _set_Mz(self, mz: float): self._cmd_Mz = mz
    def _set_Fx(self, fx: float): self._cmd_Fx = fx

    def _publish_state(self):
        """Publishes the current state and corresponding Wrench commands."""
        # Hand over control to the MPC once in STANDOFF state
        if self.state == State.STANDOFF:
            phase_msg = String()
            phase_msg.data = self.state
            self.phase_pub.publish(phase_msg)
            return

        Fx = getattr(self, '_cmd_Fx', 0.0)
        Fz = getattr(self, '_cmd_Fz', 0.0)
        Mz = getattr(self, '_cmd_Mz', 0.0)

        # Apply Safety Mask
        test_enable_z = self.get_parameter('test_enable_z').value
        test_enable_yaw = self.get_parameter('test_enable_yaw').value
        test_enable_x = self.get_parameter('test_enable_x').value

        if not test_enable_x:
            Fx = 0.0
        if not test_enable_z:
            Fz = 0.0
        if not test_enable_yaw:
            Mz = 0.0

        wrench_msg = Wrench()
        wrench_msg.force.x = Fx
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = Fz
        wrench_msg.torque.z = Mz
        
        self.wrench_pub.publish(wrench_msg)

        self._cmd_Fx, self._cmd_Fz, self._cmd_Mz = 0.0, 0.0, 0.0

        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)

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