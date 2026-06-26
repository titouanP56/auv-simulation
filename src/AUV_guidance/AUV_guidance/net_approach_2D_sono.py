#!/usr/bin/env python3
"""
net_approach_2d.py
==================

ROS 2 guidance node for the AUV's Phase 2 mission: autonomous approach to an
aquaculture net. 

This node utilizes a 2D Sonoptix sonar and Polynomial RANSAC detection logic 
to navigate the AUV relative to net structures.

State Machine:
    - DESCENDING: Maintain target depth until stable.
    - GLOBAL_SEARCH: Await net orientation detection from perception.
    - ALIGNING: Yaw to match the detected orientation of the net.
    - APPROACHING: Surge forward while maintaining orientation and depth.
    - STABILIZING: Hold position upon reaching proximity.
    - STANDOFF: Maintain final distance from the net.

Sensor Sources & Topics:
    - Inputs: /odometry/filtered (Nav), /perception/net_orientation (Pose),
              /perception/net_distance (Float32), /perception/perception_valid (Bool).
    - Outputs: /auv/command_wrench (Wrench), /mission/phase (String).

Key Parameters:
    - Control gains: KP_DEPTH, KP_YAW, KD_YAW, KP_SURGE.
    - Tolerance thresholds: DEPTH_TOLERANCE, YAW_TOLERANCE, APPROACH_TOL.
    - Time constants: STABILIZE_TIME, DEPTH_HOLD_TIME, YAW_HOLD_TIME.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Float32, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Wrench
from tf2_ros import TransformBroadcaster

TARGET_DEPTH      = -2.0    
DEPTH_TOLERANCE   = 0.2     
DEPTH_HOLD_TIME   = 2.0     
YAW_TOLERANCE     = math.radians(10.0)   
YAW_HOLD_TIME     = 1.0                  
STANDOFF_DIST     = 1.5     
APPROACH_TOL      = 0.10    
STABILIZE_TIME    = 3.0     

KP_DEPTH              = 15.0
BUOYANCY_COMPENSATION = 3.0    
KP_YAW                = 5.0
KD_YAW                = 2.0
KP_SURGE              = 6.0

MAX_DEPTH_CMD  = 20.0   
MAX_YAW_CMD    = 40.0   
MAX_SURGE_CMD  = 25.0   

CONTROL_RATE_HZ = 10.0
GLOBAL_SEARCH_TIMEOUT_SEC = 60.0   

class State:
    DESCENDING    = "DESCENDING"
    GLOBAL_SEARCH = "GLOBAL_SEARCH"   
    ALIGNING      = "ALIGNING"
    APPROACHING   = "APPROACHING"
    STABILIZING   = "STABILIZING"
    STANDOFF      = "STANDOFF"

def _yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))

class Phase2MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('phase2_mission')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10
        )
        self.ping360_sub = self.create_subscription(
            PoseStamped, '/perception/net_orientation', self._net_orientation_cb, best_effort_qos
        )
        self.full_scan_sub = self.create_subscription(
            Bool, '/perception/full_scan_ready', self._full_scan_ready_cb, best_effort_qos
        )
        
        # --- MODIFIÉ : Abonnement aux topics Float32 du Sonoptix 2D ---
        self.perception_sub = self.create_subscription(
            Float32, '/perception/net_distance', self._perception_cb, best_effort_qos
        )
        self.perception_valid_sub = self.create_subscription(
            Bool, '/perception/perception_valid', self._perception_valid_cb, best_effort_qos
        )

        self.wrench_pub = self.create_publisher(Wrench, '/auv/command_wrench', 10)

        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.phase_pub  = self.create_publisher(String, '/mission/phase', 10)
        self.done_pub   = self.create_publisher(Bool, '/mission/phase2_done', latching_qos)
        self.origin_pub = self.create_publisher(PoseStamped, '/mission/local_origin', latching_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.state: str = State.DESCENDING
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.current_yaw, self.current_vyaw = 0.0, 0.0
        self.target_yaw: float = 0.0
        self.sonoptix_range: float | None = None
        self._perception_valid: bool = False

        self._have_odom, self._have_orient = False, False   
        self._pending_yaw: float = 0.0
        self._pending_yaw_valid: bool = False
        self._full_scan_flag: bool = False
        self._search_start_time: float | None = None
        self._depth_ok_since: float | None = None
        self._yaw_ok_since: float | None = None
        self._stabilize_start: float | None = None

        self.declare_parameter('control_rate_hz', CONTROL_RATE_HZ)
        _rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / _rate, self._control_loop)
        self._publish_done_status()

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self._have_odom = True

    def _net_orientation_cb(self, msg: PoseStamped) -> None:
        if self.state != State.GLOBAL_SEARCH: return
        q = msg.pose.orientation
        world_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pending_yaw = world_yaw
        self._pending_yaw_valid = True
        self._try_transition_to_aligning()

    def _full_scan_ready_cb(self, msg: Bool) -> None:
        if not msg.data or self.state != State.GLOBAL_SEARCH: return
        self._full_scan_flag = True
        self._try_transition_to_aligning()

    def _try_transition_to_aligning(self) -> None:
        if self._pending_yaw_valid:
            self.target_yaw = self._pending_yaw
            self.state = State.ALIGNING
            self.get_logger().info(f"[net_approach_2D] Transitioning to {self.state}")
            self._pending_yaw_valid, self._full_scan_flag, self._search_start_time = False, False, None

    # --- MODIFIÉ : Traitement du Float32 ---
    def _perception_cb(self, msg: Float32) -> None:
        if self.state in (State.APPROACHING, State.STABILIZING, State.STANDOFF):
            self.sonoptix_range = msg.data

    def _perception_valid_cb(self, msg: Bool) -> None:
        self._perception_valid = msg.data

    def _control_loop(self) -> None:
        if not self._have_odom: return
        self._publish_state()
        self._publish_done_status()

        if self.state == State.DESCENDING: self._do_descending()
        elif self.state == State.GLOBAL_SEARCH: self._do_global_search()
        elif self.state == State.ALIGNING: self._do_aligning()
        elif self.state == State.APPROACHING: self._do_approaching()
        elif self.state == State.STABILIZING: self._do_stabilizing()
        elif self.state == State.STANDOFF: self._do_standoff()

    def _do_descending(self) -> None:
        depth_error = TARGET_DEPTH - self.current_z
        self._set_Fz(float(np.clip(KP_DEPTH * depth_error - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)))
        self._set_Mz(0.0); self._set_Fx(0.0)
        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(depth_error) < DEPTH_TOLERANCE:
            if self._depth_ok_since is None: self._depth_ok_since = now
            elif (now - self._depth_ok_since) >= DEPTH_HOLD_TIME:
                self.state = State.GLOBAL_SEARCH
                self.get_logger().info(f"[net_approach_2D] Transitioning to {self.state}")
                self._depth_ok_since = None
        else: self._depth_ok_since = None

    def _do_global_search(self) -> None:
        depth_error = TARGET_DEPTH - self.current_z
        self._set_Fz(float(np.clip(KP_DEPTH * depth_error - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)))
        self._set_Mz(0.0); self._set_Fx(0.0)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._search_start_time is None: self._search_start_time = now
        if (now - self._search_start_time) >= GLOBAL_SEARCH_TIMEOUT_SEC and not self._pending_yaw_valid:
            self.get_logger().error("Timeout GLOBAL_SEARCH")
            self._search_start_time = now

    def _do_aligning(self) -> None:
        depth_error = TARGET_DEPTH - self.current_z
        self._set_Fz(float(np.clip(KP_DEPTH * depth_error - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)))
        self._set_Fx(0.0)
        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        self._set_Mz(float(np.clip(KP_YAW * yaw_error - KD_YAW * self.current_vyaw, -MAX_YAW_CMD, MAX_YAW_CMD)))
        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(yaw_error) < YAW_TOLERANCE:
            if self._yaw_ok_since is None: self._yaw_ok_since = now
            elif (now - self._yaw_ok_since) >= YAW_HOLD_TIME:
                self.state = State.APPROACHING
                self.get_logger().info(f"[net_approach_2D] Transitioning to {self.state}")
                self._yaw_ok_since = None
        else: self._yaw_ok_since = None

    def _do_approaching(self) -> None:
        depth_error = TARGET_DEPTH - self.current_z
        self._set_Fz(float(np.clip(KP_DEPTH * depth_error - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)))
        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        self._set_Mz(float(np.clip(KP_YAW * yaw_error - KD_YAW * self.current_vyaw, -MAX_YAW_CMD, MAX_YAW_CMD)))
        if self.sonoptix_range is None:
            # Pas de mesure encore → avance doucement à l'aveugle
            self._set_Fx(MAX_SURGE_CMD * 0.3)
            return
        # Surge proportionnel à l'erreur de distance — borné des DEUX côtés
        # → le robot peut reculer (Fx < 0) si trop proche
        surge_cmd = float(np.clip(
            KP_SURGE * (self.sonoptix_range - STANDOFF_DIST),
            -MAX_SURGE_CMD, MAX_SURGE_CMD,
        ))
        self._set_Fx(surge_cmd)
        # Transition vers STABILIZING uniquement quand la distance est atteinte
        if abs(self.sonoptix_range - STANDOFF_DIST) <= APPROACH_TOL:
            self.state = State.STABILIZING
            self.get_logger().info(f"[net_approach_2D] Transitioning to {self.state}")
            self._stabilize_start = self.get_clock().now().nanoseconds * 1e-9

    def _do_stabilizing(self) -> None:
        depth_error = TARGET_DEPTH - self.current_z
        self._set_Fz(float(np.clip(KP_DEPTH * depth_error - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)))
        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        self._set_Mz(float(np.clip(KP_YAW * yaw_error - KD_YAW * self.current_vyaw, -MAX_YAW_CMD, MAX_YAW_CMD)))
        if self.sonoptix_range is not None:
            self._set_Fx(float(np.clip(KP_SURGE * (self.sonoptix_range - STANDOFF_DIST), -MAX_SURGE_CMD, MAX_SURGE_CMD)))
        else: self._set_Fx(0.0)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._stabilize_start and (now - self._stabilize_start) >= STABILIZE_TIME:
            self._define_local_origin()
            self.state = State.STANDOFF
            self.get_logger().info(f"[net_approach_2D] Transitioning to {self.state}")

    def _do_standoff(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if not hasattr(self, '_standoff_start'):
            self._standoff_start = now
        
        if now - self._standoff_start >= 5.0:
            self._phase3_ready = True
        else:
            self._phase3_ready = False

        depth_error = TARGET_DEPTH - self.current_z
        self._set_Fz(float(np.clip(KP_DEPTH * depth_error - BUOYANCY_COMPENSATION, -MAX_DEPTH_CMD, MAX_DEPTH_CMD)))
        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        self._set_Mz(float(np.clip(KP_YAW * yaw_error - KD_YAW * self.current_vyaw, -MAX_YAW_CMD, MAX_YAW_CMD)))
        if self.sonoptix_range is not None:
            self._set_Fx(float(np.clip(KP_SURGE * (self.sonoptix_range - STANDOFF_DIST), -MAX_SURGE_CMD, MAX_SURGE_CMD)))
        else: self._set_Fx(0.0)

        if hasattr(self, '_local_origin_pose'):
            self._local_origin_pose.header.stamp = self.get_clock().now().to_msg()
            self.origin_pub.publish(self._local_origin_pose)
        if hasattr(self, '_local_origin_transform'):
            self._local_origin_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self._local_origin_transform)

    def _define_local_origin(self) -> None:
        origin_x = self.current_x + STANDOFF_DIST * math.cos(self.target_yaw)
        origin_y = self.current_y + STANDOFF_DIST * math.sin(self.target_yaw)
        origin_z = self.current_z
        origin_yaw = self.target_yaw + math.pi   
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'local_origin'
        tf_msg.transform.translation.x = origin_x
        tf_msg.transform.translation.y = origin_y
        tf_msg.transform.translation.z = origin_z
        tf_msg.transform.rotation.z = math.sin(origin_yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(origin_yaw / 2.0)
        self._local_origin_transform = tf_msg
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = origin_x, origin_y, origin_z
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = math.sin(origin_yaw / 2.0), math.cos(origin_yaw / 2.0)
        self._local_origin_pose = pose_msg

    def _set_Fz(self, fz: float) -> None: self._cmd_Fz = float(fz)
    def _set_Mz(self, mz: float) -> None: self._cmd_Mz = float(mz)
    def _set_Fx(self, fx: float) -> None: self._cmd_Fx = float(fx)

    def _publish_done_status(self) -> None:
        done_msg = Bool()
        done_msg.data = (self.state == State.STANDOFF and getattr(self, '_phase3_ready', False))
        self.done_pub.publish(done_msg)

    def _publish_state(self) -> None:
        # On continue de publier la phase
        if self.state == State.STANDOFF:
            phase_msg = String()
            phase_msg.data = self.state
            self.phase_pub.publish(phase_msg)
            # On arrête d'envoyer des Wrench une fois que la phase 3 est prête à prendre le relais !
            if getattr(self, '_phase3_ready', False):
                return

        Fx = getattr(self, '_cmd_Fx', 0.0)
        Fz = getattr(self, '_cmd_Fz', 0.0)
        Mz = getattr(self, '_cmd_Mz', 0.0)
        wrench_msg = Wrench()
        wrench_msg.force.x = Fx
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = Fz
        wrench_msg.torque.z = Mz
        self.wrench_pub.publish(wrench_msg)
        
        self._cmd_Fx, self._cmd_Fz, self._cmd_Mz = 0.0, 0.0, 0.0
        phase_msg = String(); phase_msg.data = self.state; self.phase_pub.publish(phase_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Phase2MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__': main()