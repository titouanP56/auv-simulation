#!/usr/bin/env python3
"""
net_approach.py
===============
ROS 2 guidance node for Phase 2 of the AUV mission: approach to the
aquaculture net.

State machine
-------------
  DESCENDING    → Descend to target depth and hold.
  GLOBAL_SEARCH → Hold position while a full 360° sonar rotation is completed
                  by ping360_nearest. The AUV does NOT move laterally: the
                  perception node accumulates a full rotation and publishes a
                  robust estimate (signal /perception/full_scan_ready = True).
                  As soon as this signal is received alongside a valid orientation
                  on /perception/net_orientation, the node immediately transitions
                  to ALIGNING without waiting for further estimates.
  ALIGNING      → PD control on yaw until the robot faces the net.
  APPROACHING   → Advance toward the net until standoff distance is reached.
  STABILIZING   → Hold standoff position for STABILIZE_TIME seconds.
  STANDOFF      → Final state: continuously broadcast local origin TF for Phase 3.

ROS 2 Topics
------------
  Inputs:
    /odometry/filtered              → nav_msgs/Odometry
    /perception/net_orientation     → geometry_msgs/PoseStamped
    /perception/full_scan_ready     → std_msgs/Bool
    /sonoptix/points                → sensor_msgs/PointCloud2

  Outputs:
    /auv/command_wrench             → geometry_msgs/Wrench
    /mission/phase                  → std_msgs/String
    /mission/phase2_done            → std_msgs/Bool
    /mission/local_origin           → geometry_msgs/PoseStamped

Author  : titou
Package : AUV_guidance
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Wrench
from tf2_ros import TransformBroadcaster


# ── Constants ──────────────────────────────────────────────────────────────────

TARGET_DEPTH      = -2.0    # [m] target depth
DEPTH_TOLERANCE   = 0.2     # [m] acceptable depth error
DEPTH_HOLD_TIME   = 2.0     # [s] required duration at target depth before transitioning

YAW_TOLERANCE     = math.radians(10.0)   # [rad] acceptable yaw error
YAW_HOLD_TIME     = 1.0                  # [s] required duration aligned before transitioning

STANDOFF_DIST     = 1.5     # [m] desired standoff distance from the net
APPROACH_TOL      = 0.10    # [m] distance tolerance to accept standoff as reached
STABILIZE_TIME    = 3.0     # [s] stabilisation duration before Phase 2 ends

# ── Controller gains ───────────────────────────────────────────────────────────
KP_DEPTH              = 15.0
BUOYANCY_COMPENSATION = 3.0    # [N] static buoyancy correction
KP_YAW                = 5.0
KD_YAW                = 2.0
KP_SURGE              = 6.0

# ── Command limits ─────────────────────────────────────────────────────────────
MAX_DEPTH_CMD  = 20.0   # [N]
MAX_YAW_CMD    = 40.0   # [N·m]
MAX_SURGE_CMD  = 25.0   # [N]

# ── Sonoptix (net distance — provided by sonoptix_perception node) ─────────────
# PointCloud2 processing is delegated to auv_perception/sonoptix_perception.
# This node only subscribes to /sonoptix/perception (PoseStamped) and
# /sonoptix/perception_valid (Bool).

# ── Control ────────────────────────────────────────────────────────────────────
CONTROL_RATE_HZ = 10.0

# ── GLOBAL_SEARCH timeout ──────────────────────────────────────────────────────
# If no estimate is received after this delay, emit a warning and reset the
# timer (no hard lock-up).
GLOBAL_SEARCH_TIMEOUT_SEC = 60.0   # [s]


# ── State machine labels ───────────────────────────────────────────────────────

class State:
    DESCENDING    = "DESCENDING"
    GLOBAL_SEARCH = "GLOBAL_SEARCH"   # Replaces SCANNING: waits for one full rotation
    ALIGNING      = "ALIGNING"
    APPROACHING   = "APPROACHING"
    STABILIZING   = "STABILIZING"
    STANDOFF      = "STANDOFF"


# ── Utility functions ──────────────────────────────────────────────────────────

def _yaw_from_odom(odom: Odometry) -> float:
    """Extract the current yaw from the odometry quaternion."""
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Normalised signed angle difference a − b in [-π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ── Main node ──────────────────────────────────────────────────────────────────

class Phase2MissionNode(Node):
    """
    ROS 2 guidance node for Phase 2 of the AUV mission (net approach).

    State machine:
      DESCENDING → GLOBAL_SEARCH → ALIGNING → APPROACHING → STABILIZING → STANDOFF

    Key design choices:
    - SCANNING replaced by GLOBAL_SEARCH (no median filter over multiple scans).
    - Transition to ALIGNING fires immediately on the first robust estimate from a
      full sonar rotation (signalled by /perception/full_scan_ready Bool topic).
    - Target yaw is extracted directly from the perception PoseStamped.
    """

    def __init__(self) -> None:
        super().__init__('phase2_mission')

        # ── QoS ─────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ───────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            10,
        )

        # Net orientation (PoseStamped) from ping360_nearest
        self.ping360_sub = self.create_subscription(
            PoseStamped,
            '/perception/net_orientation',
            self._net_orientation_cb,
            best_effort_qos,
        )

        # "Full rotation ready" signal from ping360_nearest.
        # Receives True when a fresh 360° estimate has just been published.
        self.full_scan_sub = self.create_subscription(
            Bool,
            '/perception/full_scan_ready',
            self._full_scan_ready_cb,
            best_effort_qos,
        )

        # Perception results from sonoptix_perception
        self.perception_sub = self.create_subscription(
            PoseStamped,
            '/sonoptix/perception',
            self._perception_cb,
            best_effort_qos,
        )
        self.perception_valid_sub = self.create_subscription(
            Bool,
            '/sonoptix/perception_valid',
            self._perception_valid_cb,
            best_effort_qos,
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self.wrench_pub = self.create_publisher(Wrench, '/auv/command_wrench', 10)

        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.phase_pub  = self.create_publisher(String,      '/mission/phase',        10)
        self.done_pub   = self.create_publisher(Bool,        '/mission/phase2_done',  latching_qos)
        self.origin_pub = self.create_publisher(PoseStamped, '/mission/local_origin', latching_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Internal state ──────────────────────────────────────────────────
        self.state: str = State.DESCENDING

        # Odometry
        self.current_x    = 0.0
        self.current_y    = 0.0
        self.current_z    = 0.0
        self.current_yaw  = 0.0
        self.current_vyaw = 0.0

        # Target yaw (determined in GLOBAL_SEARCH, used from ALIGNING onwards)
        self.target_yaw: float = 0.0

        # Net distance from sonoptix_perception (updated in APPROACHING/STABILIZING)
        self.sonoptix_range: float | None = None
        self._perception_valid: bool = False

        # Data availability flags
        self._have_odom:   bool = False
        self._have_orient: bool = False   # True once a valid orientation has been received

        # ── GLOBAL_SEARCH state ──────────────────────────────────────────────
        # _pending_yaw       : latest orientation from ping360_nearest,
        #                      waiting for confirmation via full_scan_ready.
        # _pending_yaw_valid : True if _pending_yaw has been populated.
        # _full_scan_flag    : True if /perception/full_scan_ready just fired.
        # _search_start_time : timestamp of GLOBAL_SEARCH entry (for timeout).
        self._pending_yaw:        float       = 0.0
        self._pending_yaw_valid:  bool        = False
        self._full_scan_flag:     bool        = False
        self._search_start_time:  float | None = None
        # Timestamp of the first received orientation (for fallback logic)
        self._orient_received_at: float | None = None

        # Stabilisation timers
        self._depth_ok_since:    float | None = None
        self._yaw_ok_since:      float | None = None
        self._stabilize_start:   float | None = None

        # ── Control timer ────────────────────────────────────────────────────
        self.declare_parameter('control_rate_hz', CONTROL_RATE_HZ)
        _rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / _rate, self._control_loop)

        self._publish_done_status()
        self.get_logger().info(
            f"[Phase2MissionNode] Started → initial state: DESCENDING "
            f"(control rate = {_rate:.0f} Hz)\n"
            f"  Target depth   : {TARGET_DEPTH} m\n"
            f"  Standoff dist  : {STANDOFF_DIST} m\n"
            f"  Search timeout : {GLOBAL_SEARCH_TIMEOUT_SEC} s"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        """Update current pose and yaw rate from the filtered odometry."""
        self.current_x    = msg.pose.pose.position.x
        self.current_y    = msg.pose.pose.position.y
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self._have_odom   = True

    def _net_orientation_cb(self, msg: PoseStamped) -> None:
        """
        Callback for /perception/net_orientation (from ping360_nearest).

        The received yaw is stored as a pending estimate. The transition to
        ALIGNING is only triggered once _full_scan_flag is also set (i.e. the
        /perception/full_scan_ready signal has been received), ensuring the
        estimate comes from a complete rotation.

        Messages received in states other than GLOBAL_SEARCH are discarded
        to avoid overwriting the target yaw once alignment has begun.
        """
        if self.state != State.GLOBAL_SEARCH:
            return

        # Extract yaw from the quaternion (roll = pitch = 0)
        q = msg.pose.orientation
        world_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self._pending_yaw       = world_yaw
        self._pending_yaw_valid = True

        self.get_logger().debug(
            f"[GLOBAL_SEARCH] Orientation received: "
            f"net_yaw={math.degrees(world_yaw):.1f}°  "
            f"net_pt=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})  "
            f"full_scan_ready={self._full_scan_flag}"
        )

        # Attempt immediate transition in case the full_scan_ready signal
        # arrived before this message (possible with Best-Effort QoS ordering).
        self._try_transition_to_aligning()

    def _full_scan_ready_cb(self, msg: Bool) -> None:
        """
        Callback for /perception/full_scan_ready.

        Receives True when ping360_nearest has completed a full rotation and
        published a fresh estimate. Raise the _full_scan_flag and attempt
        the transition to ALIGNING.
        """
        if not msg.data:
            return   # ignore False messages

        if self.state != State.GLOBAL_SEARCH:
            return

        self._full_scan_flag = True
        self.get_logger().info(
            "[GLOBAL_SEARCH] 'full_scan_ready' signal received → "
            f"valid orientation available: {self._pending_yaw_valid}"
        )

        self._try_transition_to_aligning()

    def _try_transition_to_aligning(self) -> None:
        """
        Trigger the GLOBAL_SEARCH → ALIGNING transition as soon as a valid
        orientation estimate is available.

        Note: we no longer gate on _full_scan_flag because ping360_nearest
        publishes both topics (/perception/net_orientation and
        /perception/full_scan_ready) nearly simultaneously, and waiting for
        both could dead-lock if one message is dropped (Best-Effort QoS).

        The quality of the estimate is already guaranteed by the fact that
        ping360_nearest only runs its pipeline at the end of a full rotation
        (or every _min_period_sec via the temporal fallback).
        """
        if self._pending_yaw_valid:
            self.target_yaw = self._pending_yaw
            self.get_logger().info(
                f"[GLOBAL_SEARCH → ALIGNING] Estimate received. "
                f"Target yaw: {math.degrees(self.target_yaw):.1f}°  "
                f"(full_scan_flag={self._full_scan_flag})"
            )
            self.state = State.ALIGNING

            # Reset flags
            self._pending_yaw_valid = False
            self._full_scan_flag    = False
            self._search_start_time = None

    def _perception_cb(self, msg: PoseStamped) -> None:
        """
        Receive perception output from sonoptix_perception.
        The orthogonal distance to the net plane is in pose.position.x.
        Only active during APPROACHING, STABILIZING, and STANDOFF.
        """
        if self.state in (State.APPROACHING, State.STABILIZING, State.STANDOFF):
            self.sonoptix_range = msg.pose.position.x

    def _perception_valid_cb(self, msg: Bool) -> None:
        """Track the validity flag from sonoptix_perception."""
        self._perception_valid = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    # Main control loop
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """Main control loop, called at CONTROL_RATE_HZ Hz."""
        if not self._have_odom:
            return

        self._publish_state()
        self._publish_done_status()

        if self.state == State.DESCENDING:
            self._do_descending()
        elif self.state == State.GLOBAL_SEARCH:
            self._do_global_search()
        elif self.state == State.ALIGNING:
            self._do_aligning()
        elif self.state == State.APPROACHING:
            self._do_approaching()
        elif self.state == State.STABILIZING:
            self._do_stabilizing()
        elif self.state == State.STANDOFF:
            self._do_standoff()

    # ─────────────────────────────────────────────────────────────────────────
    # State handlers
    # ─────────────────────────────────────────────────────────────────────────

    def _do_descending(self) -> None:
        """
        Descend to TARGET_DEPTH and transition to GLOBAL_SEARCH once depth
        has been held within tolerance for DEPTH_HOLD_TIME seconds.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)
        self._set_Mz(0.0)
        self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(depth_error) < DEPTH_TOLERANCE:
            if self._depth_ok_since is None:
                self._depth_ok_since = now
            elif (now - self._depth_ok_since) >= DEPTH_HOLD_TIME:
                self.get_logger().info(
                    f"[DESCENDING → GLOBAL_SEARCH] Depth stable at "
                    f"{self.current_z:.2f} m"
                )
                self.state = State.GLOBAL_SEARCH
                self._depth_ok_since = None
        else:
            self._depth_ok_since = None

    def _do_global_search(self) -> None:
        """
        Hold the AUV in place (depth + zero yaw rate) while the Ping360 sonar
        completes a full 360° rotation.

        The transition to ALIGNING is triggered by the callbacks
        (_net_orientation_cb and _full_scan_ready_cb) once both conditions are met.

        A GLOBAL_SEARCH_TIMEOUT_SEC timeout emits an error log if no estimate
        is received (e.g. perception node not running, TF tree missing).
        """
        # Strict depth hold (Fx = 0: no forward translation)
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)
        self._set_Mz(0.0)   # no rotation either: let the sonar sweep on its own
        self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._search_start_time is None:
            self._search_start_time = now
            self.get_logger().info(
                "[GLOBAL_SEARCH] Waiting for a robust estimate from "
                "ping360_nearest (full 360° rotation)…\n"
                "  AUV holding position. Do not command any movement."
            )

        elapsed = now - self._search_start_time

        # ── Timeout guard ─────────────────────────────────────────────────────
        if elapsed >= GLOBAL_SEARCH_TIMEOUT_SEC and not self._pending_yaw_valid:
            self.get_logger().error(
                f"[GLOBAL_SEARCH] Timeout ({GLOBAL_SEARCH_TIMEOUT_SEC:.0f} s): "
                "no estimate received from /perception/net_orientation. "
                "Check that ping360_nearest is running and TF2 is available."
            )
            # Reset timer — do not lock up the state machine permanently
            self._search_start_time = now

    def _do_aligning(self) -> None:
        """
        PD yaw control until the robot faces the net.

        Depth is maintained; no forward motion.
        Transitions to APPROACHING once yaw error is within tolerance for
        YAW_HOLD_TIME seconds.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)
        self._set_Fx(0.0)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd    = float(np.clip(
            KP_YAW * yaw_error - KD_YAW * self.current_vyaw,
            -MAX_YAW_CMD, MAX_YAW_CMD,
        ))
        self._set_Mz(mz_cmd)

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(yaw_error) < YAW_TOLERANCE:
            if self._yaw_ok_since is None:
                self._yaw_ok_since = now
            elif (now - self._yaw_ok_since) >= YAW_HOLD_TIME:
                self.get_logger().info(
                    f"[ALIGNING → APPROACHING] Aligned: "
                    f"yaw error = {math.degrees(yaw_error):.2f}°"
                )
                self.state = State.APPROACHING
                self._yaw_ok_since = None
        else:
            self._yaw_ok_since = None

    def _do_approaching(self) -> None:
        """
        Advance toward the net while maintaining heading and depth.

        If Sonoptix data is not yet available, advance at 30% of maximum
        surge thrust (blind approach fallback).
        Transitions to STABILIZING once the standoff distance is reached.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd    = float(np.clip(
            KP_YAW * yaw_error - KD_YAW * self.current_vyaw,
            -MAX_YAW_CMD, MAX_YAW_CMD,
        ))
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is None:
            # No distance measurement yet → slow blind advance
            self._set_Fx(MAX_SURGE_CMD * 0.3)
            return

        surge_cmd = float(np.clip(
            KP_SURGE * (self.sonoptix_range - STANDOFF_DIST),
            0.0, MAX_SURGE_CMD,
        ))
        self._set_Fx(surge_cmd)

        if (self.sonoptix_range - STANDOFF_DIST) <= APPROACH_TOL:
            self.get_logger().info(
                f"[APPROACHING → STABILIZING] Standoff distance reached: "
                f"{self.sonoptix_range:.2f} m. Stabilising…"
            )
            self.state = State.STABILIZING
            self._stabilize_start = self.get_clock().now().nanoseconds * 1e-9

    def _do_stabilizing(self) -> None:
        """
        Hold the standoff position for STABILIZE_TIME seconds.
        Then transition to STANDOFF and broadcast the local origin for Phase 3.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd    = float(np.clip(
            KP_YAW * yaw_error - KD_YAW * self.current_vyaw,
            -MAX_YAW_CMD, MAX_YAW_CMD,
        ))
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is not None:
            surge_cmd = float(np.clip(
                KP_SURGE * (self.sonoptix_range - STANDOFF_DIST),
                -MAX_SURGE_CMD, MAX_SURGE_CMD,
            ))
            self._set_Fx(surge_cmd)
        else:
            self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._stabilize_start and (now - self._stabilize_start) >= STABILIZE_TIME:
            self.get_logger().info(
                "[STABILIZING → STANDOFF] Robot stabilised. Defining local origin."
            )
            self._define_local_origin()
            self.state = State.STANDOFF

    def _do_standoff(self) -> None:
        """
        Final state: continuously broadcast the local origin pose and TF
        for the Phase 3 inspection node.
        """
        if hasattr(self, '_local_origin_pose'):
            self._local_origin_pose.header.stamp = self.get_clock().now().to_msg()
            self.origin_pub.publish(self._local_origin_pose)

        if hasattr(self, '_local_origin_transform'):
            self._local_origin_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self._local_origin_transform)

    # ─────────────────────────────────────────────────────────────────────────
    # Local origin definition (Phase 2 → Phase 3 handoff)
    # ─────────────────────────────────────────────────────────────────────────

    def _define_local_origin(self) -> None:
        """
        Compute and store the local_origin frame used by Phase 3. The origin is
        placed at the standoff point projected onto the net surface, facing the AUV.
        """
        origin_x   = self.current_x + STANDOFF_DIST * math.cos(self.target_yaw)
        origin_y   = self.current_y + STANDOFF_DIST * math.sin(self.target_yaw)
        origin_z   = self.current_z
        origin_yaw = self.target_yaw + math.pi   # face the AUV from the net

        self.get_logger().info(
            f"[PHASE 3] Local origin defined: "
            f"x={origin_x:.2f}  y={origin_y:.2f}  z={origin_z:.2f}  "
            f"yaw={math.degrees(origin_yaw):.1f}°"
        )

        # ── TF transform ───────────────────────────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.frame_id    = 'odom'
        tf_msg.child_frame_id     = 'local_origin'
        tf_msg.transform.translation.x = origin_x
        tf_msg.transform.translation.y = origin_y
        tf_msg.transform.translation.z = origin_z
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(origin_yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(origin_yaw / 2.0)
        self._local_origin_transform = tf_msg

        # ── PoseStamped ────────────────────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.frame_id         = 'odom'
        pose_msg.pose.position.x         = origin_x
        pose_msg.pose.position.y         = origin_y
        pose_msg.pose.position.z         = origin_z
        pose_msg.pose.orientation.x      = 0.0
        pose_msg.pose.orientation.y      = 0.0
        pose_msg.pose.orientation.z      = math.sin(origin_yaw / 2.0)
        pose_msg.pose.orientation.w      = math.cos(origin_yaw / 2.0)
        self._local_origin_pose = pose_msg

    # ─────────────────────────────────────────────────────────────────────────
    # Command helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _set_Fz(self, fz: float) -> None:
        self._cmd_Fz = float(fz)

    def _set_Mz(self, mz: float) -> None:
        self._cmd_Mz = float(mz)

    def _set_Fx(self, fx: float) -> None:
        self._cmd_Fx = float(fx)

    # ─────────────────────────────────────────────────────────────────────────
    # State publishers
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_done_status(self) -> None:
        """Publish True on /mission/phase2_done when STANDOFF state is reached."""
        done_msg = Bool()
        done_msg.data = (self.state == State.STANDOFF)
        self.done_pub.publish(done_msg)

    def _publish_state(self) -> None:
        """Publish the current state name and the computed Wrench command."""
        # In STANDOFF the guidance wrench is zeroed (Phase 3 takes over control)
        if self.state == State.STANDOFF:
            phase_msg = String()
            phase_msg.data = self.state
            self.phase_pub.publish(phase_msg)
            return

        Fx = getattr(self, '_cmd_Fx', 0.0)
        Fz = getattr(self, '_cmd_Fz', 0.0)
        Mz = getattr(self, '_cmd_Mz', 0.0)

        wrench_msg = Wrench()
        wrench_msg.force.x  = Fx
        wrench_msg.force.y  = 0.0
        wrench_msg.force.z  = Fz
        wrench_msg.torque.z = Mz
        self.wrench_pub.publish(wrench_msg)

        # Reset commands to avoid unintended repetition next cycle
        self._cmd_Fx = 0.0
        self._cmd_Fz = 0.0
        self._cmd_Mz = 0.0

        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Standard ROS 2 entry point."""
    rclpy.init(args=args)
    node = Phase2MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Phase2MissionNode] Keyboard interrupt — shutting down.")
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