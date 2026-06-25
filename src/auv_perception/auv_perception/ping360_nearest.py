#!/usr/bin/env python3
"""
ping360_nearest.py
==================
Advanced ROS 2 perception node for the Ping360 mechanical sonar.

Processing pipeline (v3 – RANSAC inlier-ratio cluster selection)
-----------------------------------------------------------------
  1. Receive LaserScan sweeps from /ping360/scan.
  2. For each valid range in the scan, use TF2 to transform the point from the
     sensor frame (ping360_link) into the fixed world frame (odom), compensating
     for robot motion during the accumulation window.
  3. Accumulate transformed points in a rolling time window equal to ONE full
     sonar rotation. The AUV holds position during this rotation (no forward
     motion commanded).
  4. At the end of a full rotation (fresh estimate trigger):
       a. DBSCAN   : cluster the point cloud to isolate distinct objects.
       b. RANSAC inlier-ratio selection : for each cluster, fit a degree-2
          polynomial and measure the inlier ratio. A net (even heavily curved)
          fits the curve well → high inlier ratio. A fish school (volumetric) →
          very low inlier ratio. Select the cluster with the best ratio ≥
          ransac_min_inlier_ratio.
       c. Closest point : find the point on the RANSAC curve closest to the AUV
          and compute the tangent normal.
       d. Publish : target yaw as a Quaternion in a PoseStamped on
          /perception/net_orientation.
          → Publish a Bool on /perception/full_scan_ready to signal to
            net_approach.py that a robust full-rotation estimate is ready.

ROS 2 Parameters (all configurable via CLI or YAML)
----------------------------------------------------
  sonar_topic             : input LaserScan topic      (default: /ping360/scan)
  output_topic            : output PoseStamped topic   (default: /perception/net_orientation)
  ready_topic             : full-scan Bool topic        (default: /perception/full_scan_ready)
  source_frame            : sensor frame                (default: ping360_link)
  target_frame            : fixed reference frame       (default: odom)
  window_sec              : accumulation window in s (0 = auto from scan) (default: 0.0)
  ignore_fraction         : fraction of range_max to reject saturated echoes (default: 0.95)
  min_range_m             : minimum valid range in metres (default: 0.3)
  dbscan_eps              : DBSCAN neighbourhood radius in metres (default: 0.25)
  dbscan_min_samples      : minimum samples to form a DBSCAN cluster (default: 5)
  ransac_min_inlier_ratio : minimum inlier/cluster ratio to validate as net (default: 0.30)
  ransac_residual         : RANSAC residual threshold in metres (default: 0.1)
  ransac_min_samples      : minimum points required for RANSAC fit (default: 10)
  min_cluster_pts         : minimum cluster size to be a candidate (default: 15)
  max_range_m             : absolute maximum valid range [m] (default: 5.0)
                            Used to isolate the net from pool walls.
                            Set to 0.0 to disable (uses ignore_fraction instead).

Author  : titou
Package : auv_perception
Topics  : input  → /ping360/scan
          outputs → /perception/net_orientation
                  → /perception/full_scan_ready
"""

# ── Standard library ──────────────────────────────────────────────────────────
import math
from collections import deque

# ── Scientific computing ──────────────────────────────────────────────────────
import numpy as np
from sklearn.cluster import DBSCAN

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Messages & TF2 ────────────────────────────────────────────────────────────
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped with tf2


# ── Default constants (overridden by ROS 2 parameters) ───────────────────────

_DEFAULT_SONAR_TOPIC       = "/ping360/scan"
_DEFAULT_OUTPUT_TOPIC      = "/perception/net_orientation"
_DEFAULT_READY_TOPIC       = "/perception/full_scan_ready"
_DEFAULT_SOURCE_FRAME      = "ping360_link"
_DEFAULT_TARGET_FRAME      = "odom"
_DEFAULT_WINDOW_SEC        = 0.0    # 0 = auto-detect from full rotation duration
_DEFAULT_IGNORE_FRAC       = 0.95   # discard echoes beyond 95% of range_max
_DEFAULT_MIN_RANGE_M       = 0.3    # dead zone close to the robot
_DEFAULT_DBSCAN_EPS        = 0.25   # [m] DBSCAN neighbourhood radius
_DEFAULT_DBSCAN_MIN_PTS    = 5      # minimum points to form a cluster
_DEFAULT_RANSAC_MIN_INLIER = 0.30   # minimum inlier ratio to validate as net
_DEFAULT_RANSAC_RESID      = 0.10   # [m] RANSAC residual threshold
_DEFAULT_RANSAC_MIN_PTS    = 10     # minimum points for polynomial fit
_DEFAULT_MIN_CLUSTER       = 15     # minimum cluster size to be considered
_DEFAULT_MAX_RANGE_M       = 5.0    # [m] hard cut-off — isolates net (3-5m) from walls (>5m)
                                    # set to 0.0 to disable


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """
    Convert a yaw angle (rad) to a quaternion (x, y, z, w).
    Roll and pitch are assumed to be zero (horizontal plane).
    """
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _poly2_closest_point(coeffs: np.ndarray, pts: np.ndarray) -> tuple[float, float]:
    """
    Find the point on the parabola y = a*x^2 + b*x + c closest to the
    centroid of the cluster points.

    The search is done by dense sampling over the X range of the cluster,
    which is accurate enough for sonar data (~mm resolution).

    Args:
        coeffs : [a, b, c] polynomial coefficients (degree 2, X as independent axis).
        pts    : cluster point cloud (N, 2).

    Returns:
        (x_closest, y_closest) on the fitted curve.
    """
    x_min, x_max = float(pts[:, 0].min()), float(pts[:, 0].max())
    # 500 samples over the range spanned by the cluster
    x_samples = np.linspace(x_min, x_max, 500)
    y_samples = np.polyval(coeffs, x_samples)

    centroid = pts.mean(axis=0)   # reference point = cluster centroid

    dist2 = (x_samples - centroid[0]) ** 2 + (y_samples - centroid[1]) ** 2
    idx = int(np.argmin(dist2))
    return float(x_samples[idx]), float(y_samples[idx])


def _poly2_normal_yaw(coeffs: np.ndarray, x0: float, toward_origin: np.ndarray) -> float:
    """
    Compute the yaw angle of the normal to the parabola tangent at point
    (x0, y(x0)), oriented toward the origin (i.e., toward the AUV).

    The tangent slope at x0 is dy/dx = 2*a*x0 + b.
    The perpendicular normal is in direction (-dy/dx, 1), normalised.

    Args:
        coeffs        : [a, b, c] polynomial coefficients.
        x0            : x-coordinate of the point of interest on the curve.
        toward_origin : reference vector pointing toward the AUV
                        (typically [0,0] - centroid, unnormalised).

    Returns:
        yaw_rad : yaw angle (rad) of the normal, oriented toward the AUV.
    """
    a, b = float(coeffs[0]), float(coeffs[1])
    slope_tangent = 2.0 * a * x0 + b          # dy/dx at x0

    # Normal perpendicular to the tangent (90-degree rotation)
    normal = np.array([-slope_tangent, 1.0])
    norm_mag = np.linalg.norm(normal)
    if norm_mag < 1e-9:
        normal = np.array([0.0, 1.0])
    else:
        normal /= norm_mag

    # Orient the normal toward the AUV
    if np.dot(normal, toward_origin) < 0:
        normal = -normal

    return math.atan2(normal[1], normal[0])


# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class Ping360NearestNode(Node):
    """
    ROS 2 perception node for aquaculture net detection using the Ping360 sonar.

    v3 highlights:
    - Accumulation window = duration of ONE full sonar rotation (auto-calibrated).
    - Net cluster selected by RANSAC inlier ratio (robust against strong net
      curvature from currents where global PCA fails).
    - Degree-2 polynomial RANSAC to model net curvature.
    - Closest point on the fitted curve + normal → target yaw.
    - Publishes a Bool on /perception/full_scan_ready to signal to the
      guidance node that a full-rotation estimate is available.
    """

    def __init__(self) -> None:
        super().__init__("ping360_nearest")

        # ── Parameter declarations ────────────────────────────────────────────
        self.declare_parameter("sonar_topic",          _DEFAULT_SONAR_TOPIC)
        self.declare_parameter("output_topic",         _DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("ready_topic",          _DEFAULT_READY_TOPIC)
        self.declare_parameter("source_frame",         _DEFAULT_SOURCE_FRAME)
        self.declare_parameter("target_frame",         _DEFAULT_TARGET_FRAME)
        self.declare_parameter("window_sec",           _DEFAULT_WINDOW_SEC)
        self.declare_parameter("ignore_fraction",      _DEFAULT_IGNORE_FRAC)
        self.declare_parameter("min_range_m",          _DEFAULT_MIN_RANGE_M)
        self.declare_parameter("dbscan_eps",              _DEFAULT_DBSCAN_EPS)
        self.declare_parameter("dbscan_min_samples",      _DEFAULT_DBSCAN_MIN_PTS)
        self.declare_parameter("ransac_min_inlier_ratio", _DEFAULT_RANSAC_MIN_INLIER)
        self.declare_parameter("ransac_residual",         _DEFAULT_RANSAC_RESID)
        self.declare_parameter("ransac_min_samples",      _DEFAULT_RANSAC_MIN_PTS)
        self.declare_parameter("min_cluster_pts",         _DEFAULT_MIN_CLUSTER)
        self.declare_parameter("max_range_m",             _DEFAULT_MAX_RANGE_M)

        # ── Read parameters ───────────────────────────────────────────────────
        self._sonar_topic         = self.get_parameter("sonar_topic").value
        self._output_topic        = self.get_parameter("output_topic").value
        self._ready_topic         = self.get_parameter("ready_topic").value
        self._source_frame        = self.get_parameter("source_frame").value
        self._target_frame        = self.get_parameter("target_frame").value
        self._window_sec          = float(self.get_parameter("window_sec").value)
        self._ignore_frac         = float(self.get_parameter("ignore_fraction").value)
        self._min_range_m         = float(self.get_parameter("min_range_m").value)
        self._dbscan_eps          = float(self.get_parameter("dbscan_eps").value)
        self._dbscan_min_pts      = int(self.get_parameter("dbscan_min_samples").value)
        self._ransac_min_inlier   = float(self.get_parameter("ransac_min_inlier_ratio").value)
        self._ransac_resid        = float(self.get_parameter("ransac_residual").value)
        self._ransac_min_pts      = int(self.get_parameter("ransac_min_samples").value)
        self._min_cluster         = int(self.get_parameter("min_cluster_pts").value)
        self._max_range_m         = float(self.get_parameter("max_range_m").value)

        # ── Point accumulation buffer ─────────────────────────────────────────
        # Each entry: (timestamp_sec: float, x: float, y: float)
        self._point_buffer: deque[tuple[float, float, float]] = deque()

        # ── Full-rotation tracking ────────────────────────────────────────────
        # Three end-of-rotation detection mechanisms (priority order):
        #
        #  A) Single-message 360° scan (Gazebo):
        #     If angle_max - angle_min >= 300°, the scan is already complete.
        #     The pipeline is triggered every _min_period_sec seconds.
        #
        #  B) Angular wrap-around (real mechanical Ping360):
        #     Triggered when angle_min jumps from ~+π back to ~-π between
        #     two consecutive messages.
        #
        #  C) Temporal fallback:
        #     If neither A nor B fires and the buffer is large enough, publish
        #     every _min_period_sec seconds to avoid starvation.
        #
        # _min_period_sec : minimum interval between two pipeline triggers
        #                   (prevents flooding the bus with repeated estimates
        #                   when the sonar publishes very rapidly).
        self._last_angle_rad: float | None    = None
        self._tour_start_sec: float | None    = None
        self._tour_duration_sec: float | None = None
        self._full_tour_ready: bool           = False
        self._last_pipeline_sec: float        = 0.0   # timestamp of last pipeline run
        self._min_period_sec: float           = 2.0   # [s] anti-spam between estimates

        # ── TF2 ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS: Best Effort to match the Gazebo bridge ───────────────────────
        sonar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Sonar subscription ─────────────────────────────────────────────────
        self._sonar_sub = self.create_subscription(
            LaserScan,
            self._sonar_topic,
            self._scan_callback,
            sonar_qos,
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self._orient_pub = self.create_publisher(
            PoseStamped,
            self._output_topic,
            10,
        )
        # Boolean signal: True = fresh full-rotation estimate is ready
        self._ready_pub = self.create_publisher(
            Bool,
            self._ready_topic,
            10,
        )

        # ── Debug visualisation publishers (RViz / Foxglove) ──────────────────
        self._debug_cluster_pub = self.create_publisher(
            Marker,
            "~/debug/net_cluster",
            10,
        )
        self._debug_curve_pub = self.create_publisher(
            Marker,
            "~/debug/ransac_curve",
            10,
        )
        self._debug_raw_pub = self.create_publisher(
            Marker,
            "~/debug/raw_points",
            10,
        )

        # ── Diagnostic counters ────────────────────────────────────────────────
        self._n_scans_received = 0
        self._n_points_added   = 0
        self._n_tf_failures    = 0
        self._n_estimates_pub  = 0

        self.get_logger().info(
            f"\n[ping360_nearest] Node started (v3 — RANSAC inlier-ratio selection)\n"
            f"  Sonar topic       : {self._sonar_topic}\n"
            f"  Output topic      : {self._output_topic}\n"
            f"  Ready topic       : {self._ready_topic}\n"
            f"  Frames            : {self._source_frame} → {self._target_frame}\n"
            f"  Window            : {'auto (1 rotation)' if self._window_sec == 0.0 else f'{self._window_sec} s'}\n"
            f"  Range filter      : [{self._min_range_m:.2f} m — "
            f"{'disabled (ignore_fraction)' if self._max_range_m == 0.0 else f'{self._max_range_m:.2f} m'}]\n"
            f"  DBSCAN            : eps={self._dbscan_eps} m, min_pts={self._dbscan_min_pts}\n"
            f"  RANSAC inlier ratio ≥ {self._ransac_min_inlier:.0%} (net selection threshold)\n"
            f"  RANSAC poly2      : residual={self._ransac_resid} m, min_pts={self._ransac_min_pts}\n"
            f"  Min cluster       : {self._min_cluster} points"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Main scan callback
    # ──────────────────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Processes a LaserScan message from the Ping360.

        End-of-rotation detection (priority order):
          A) The message already covers ≥ 300° → complete scan in a single message
             (typical of Gazebo). Publish if _min_period_sec has elapsed.
          B) Angular wrap-around between consecutive messages → real mechanical
             Ping360 rotating in increments.
          C) Temporal fallback: publish if _min_period_sec has elapsed and the
             buffer is large enough (guard against A and B not triggering).
        """
        self._n_scans_received += 1

        stamp_sec       = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        max_valid_range = msg.range_max * self._ignore_frac

        # ── Debug: log range statistics every 5 scans ─────────────────────────
        if self._n_scans_received % 5 == 1:
            finite_ranges = [r for r in msg.ranges if math.isfinite(r)]
            if finite_ranges:
                self.get_logger().debug(
                    f"[ping360_nearest] [DEBUG RANGES] scan #{self._n_scans_received}: "
                    f"n_rays={len(msg.ranges)}, finite={len(finite_ranges)}, "
                    f"min={min(finite_ranges):.3f} m, max={max(finite_ranges):.3f} m, "
                    f"range_max={msg.range_max:.1f} m, valid_threshold={max_valid_range:.3f} m"
                )
            else:
                self.get_logger().debug(
                    f"[ping360_nearest] [DEBUG RANGES] scan #{self._n_scans_received}: "
                    f"NO finite ranges among {len(msg.ranges)} rays! "
                    f"(all inf/NaN)"
                )

        # ── A) Full 360° scan in a single message (Gazebo / simulation) ───────
        scan_angular_range = abs(msg.angle_max - msg.angle_min)
        is_full_scan_msg   = scan_angular_range >= math.radians(300.0)

        # ── B) Angular wrap-around (real mechanical Ping360) ──────────────────
        wrap_detected = False
        current_angle = msg.angle_min
        if self._last_angle_rad is not None and not is_full_scan_msg:
            angle_delta = current_angle - self._last_angle_rad
            if angle_delta < -math.pi:      # jump from ~+π to ~-π = new rotation
                wrap_detected = True
                if self._tour_start_sec is not None:
                    self._tour_duration_sec = stamp_sec - self._tour_start_sec
                    self.get_logger().debug(
                        f"[ping360_nearest] Full rotation (wrap-around) — "
                        f"duration: {self._tour_duration_sec:.2f} s"
                    )
                self._tour_start_sec = stamp_sec
                self._full_tour_ready = True
        else:
            if self._tour_start_sec is None:
                self._tour_start_sec = stamp_sec
        self._last_angle_rad = current_angle

        # ── TF2 transform: looked up once per message for efficiency ──────────
        frame_id = msg.header.frame_id if msg.header.frame_id else self._source_frame
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as exc:
            self._n_tf_failures += 1
            if self._n_tf_failures % 5 == 1:
                self.get_logger().warn(
                    f"[ping360_nearest] TF2 failure #{self._n_tf_failures} "
                    f"({type(exc).__name__}): {exc}  "
                    f"[frames: {frame_id} → {self._target_frame}]"
                )
            return

        # ── Process each range ray ─────────────────────────────────────────────
        # Upper distance threshold: use max_range_m if set, otherwise ignore_fraction * range_max.
        if self._max_range_m > 0.0:
            effective_max = self._max_range_m
        else:
            effective_max = max_valid_range

        n_pts_added_this_scan = 0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < self._min_range_m or r >= effective_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            x_local   = r * math.cos(angle_rad)
            y_local   = r * math.sin(angle_rad)

            pt_in = PointStamped()
            pt_in.point.x = x_local
            pt_in.point.y = y_local
            pt_in.point.z = 0.0
            pt_out = tf2_geometry_msgs.do_transform_point(pt_in, transform)

            self._point_buffer.append((stamp_sec, pt_out.point.x, pt_out.point.y))
            self._n_points_added   += 1
            n_pts_added_this_scan  += 1

        # Log filter efficiency to help tune max_range_m
        if self._n_scans_received % 5 == 1:
            self.get_logger().debug(
                f"[ping360_nearest] [FILTER] scan #{self._n_scans_received}: "
                f"{n_pts_added_this_scan}/{len(msg.ranges)} rays accepted "
                f"(threshold: {self._min_range_m:.2f}—{effective_max:.2f} m), "
                f"buffer={len(self._point_buffer)} pts"
            )

        # ── Rolling window pruning ─────────────────────────────────────────────
        if self._window_sec > 0.0:
            window = self._window_sec
        elif self._tour_duration_sec is not None:
            window = self._tour_duration_sec
        else:
            window = 10.0   # conservative fallback until duration is calibrated

        cutoff = stamp_sec - window
        while self._point_buffer and self._point_buffer[0][0] < cutoff:
            self._point_buffer.popleft()

        # ── Pipeline trigger decision ──────────────────────────────────────────
        time_since_last = stamp_sec - self._last_pipeline_sec
        throttled_ok    = time_since_last >= self._min_period_sec
        buffer_ok       = len(self._point_buffer) >= self._min_cluster

        trigger = False
        trigger_reason = ""

        if is_full_scan_msg and throttled_ok and buffer_ok:
            # Case A: Gazebo full-scan message
            trigger = True
            trigger_reason = f"full_scan_360° ({math.degrees(scan_angular_range):.0f}°)"
        elif wrap_detected and buffer_ok:
            # Case B: real mechanical Ping360
            trigger = True
            trigger_reason = "wrap-around detected"
        elif throttled_ok and buffer_ok and self._n_scans_received > 5:
            # Case C: temporal fallback (neither A nor B triggered)
            trigger = True
            trigger_reason = f"temporal fallback ({time_since_last:.1f} s elapsed)"

        if trigger:
            self.get_logger().debug(
                f"[ping360_nearest] ▶ Pipeline triggered — reason: {trigger_reason}  "
                f"buffer={len(self._point_buffer)} pts"
            )
            self._last_pipeline_sec = stamp_sec
            self._detect_and_publish(stamp_sec, msg.header.stamp)
        elif is_full_scan_msg and buffer_ok and not throttled_ok:
            pass   # valid scan but too frequent → silent (no spam log)
        elif not buffer_ok and time_since_last >= self._min_period_sec:
            self.get_logger().warn(
                f"[ping360_nearest] Buffer too small for {time_since_last:.1f} s — "
                f"{len(self._point_buffer)}/{self._min_cluster} pts required. "
                f"scans_received={self._n_scans_received}, pts_added_total={self._n_points_added}, "
                f"tf_failures={self._n_tf_failures}. "
                "Check sonar range, TF tree, and filtering parameters."
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Detection pipeline: DBSCAN → RANSAC selection → publish
    # ──────────────────────────────────────────────────────────────────────────

    def _detect_and_publish(self, stamp_sec: float, ros_stamp) -> None:
        """
        Runs the detection pipeline on the current buffer and publishes the result.

        Internal steps:
          1. Convert buffer to a NumPy (N, 2) matrix.
          2. DBSCAN → candidate clusters.
          3. RANSAC inlier-ratio selection: the cluster whose points best fit a
             degree-2 parabola is identified as the net. Returns coefficients and
             inlier points directly.
          4. Closest point on the fitted curve to the AUV → normal → target yaw.
          5. Build and publish PoseStamped + Bool "ready" signal.
        """
        # ── 1. Extract XY matrix ───────────────────────────────────────────────
        pts = np.array([(x, y) for _, x, y in self._point_buffer], dtype=np.float64)

        # ── 2. DBSCAN clustering ───────────────────────────────────────────────
        labels = self._run_dbscan(pts)
        if labels is None:
            return

        # ── 3. Net cluster selection + RANSAC poly2 ────────────────────────────
        # _select_net_cluster_ransac iterates over clusters, runs RANSAC on each,
        # and returns the best one (highest inlier ratio).
        selection = self._select_net_cluster_ransac(pts, labels)
        if selection is None:
            return   # error logs already emitted inside the method

        net_pts, coeffs, inlier_pts, swap_axes = selection
        # coeffs = [a, b, c] in regression space (possibly axis-swapped)
        # inlier_pts is in the same space (for _poly2_closest_point)

        # ── Debug markers (non-blocking, best-effort) ──────────────────────────
        self._publish_debug_markers(pts, inlier_pts, coeffs, swap_axes, ros_stamp)

        # ── 4. Closest point + normal ──────────────────────────────────────────
        # Find the point on the fitted curve closest to the cluster centroid
        # (best approximation of "closest to AUV" in the current odom frame).
        x_close_fit, y_close_fit = _poly2_closest_point(coeffs, inlier_pts)

        # Vector from AUV (approx at odom origin = [0,0]) toward the cluster,
        # used to orient the normal toward the AUV.
        net_centroid = inlier_pts.mean(axis=0)
        toward_net   = net_centroid - np.array([0.0, 0.0])   # AUV ≈ at odom origin

        yaw_fit = _poly2_normal_yaw(coeffs, x_close_fit, toward_net)

        if swap_axes:
            # Axes were swapped during regression; restore original frame.
            x_close = y_close_fit
            y_close = x_close_fit

            # The normal vector (cos, sin) in swapped space (Y, X) must be
            # mapped back to (X, Y) via axis swap.
            vec_fit = np.array([math.cos(yaw_fit), math.sin(yaw_fit)])
            yaw_target = math.atan2(vec_fit[0], vec_fit[1])  # atan2(X_fit, Y_fit)
        else:
            x_close = x_close_fit
            y_close = y_close_fit
            yaw_target = yaw_fit

        # ── 5. Build and publish PoseStamped ──────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = ros_stamp
        pose_msg.header.frame_id = self._target_frame

        # Position = closest point on the fitted curve to the AUV
        pose_msg.pose.position.x = x_close
        pose_msg.pose.position.y = y_close
        pose_msg.pose.position.z = 0.0

        qx, qy, qz, qw = _quaternion_from_yaw(yaw_target)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self._orient_pub.publish(pose_msg)
        self._n_estimates_pub += 1

        # Signal that a fresh full-rotation estimate is available
        ready_msg = Bool()
        ready_msg.data = True
        self._ready_pub.publish(ready_msg)

        self.get_logger().debug(
            f"[ping360_nearest] ✔ Estimate #{self._n_estimates_pub} (full rotation): "
            f"yaw={math.degrees(yaw_target):.1f}°  "
            f"closest_pt=({x_close:.2f}, {y_close:.2f})  "
            f"cluster={len(net_pts)} pts → inliers={len(inlier_pts)} "
            f"({len(inlier_pts)/len(net_pts):.0%})  "
            f"buffer={len(pts)} pts"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Debug visualisation
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_debug_markers(
        self,
        raw_pts: np.ndarray,
        inlier_pts: np.ndarray,
        coeffs: np.ndarray,
        swap_axes: bool,
        stamp,
    ) -> None:
        """
        Publish three RViz / Foxglove visualisation markers:
          - Marker 0 (POINTS, green)     : RANSAC inlier points in the odom frame.
          - Marker 1 (LINE_STRIP, red)   : Sampled polynomial curve over the inliers.
          - Marker 2 (POINTS, blue-grey) : Full pre-DBSCAN point cloud (raw buffer).

        These markers are purely cosmetic and do not affect any algorithmic output.

        Args:
            raw_pts    : (N, 2) array of ALL accumulated points before DBSCAN,
                         already in the odom frame — no axis swap needed.
            inlier_pts : (M, 2) array of RANSAC inlier points in *regression space*
                         (axes may be swapped vs. odom frame — see swap_axes).
            coeffs     : [a, b, c] polynomial coefficients in regression space.
            swap_axes  : True if X/Y were swapped before regression.
                         When True the regression X axis corresponds to odom Y,
                         and vice-versa; we restore the physical frame here.
            stamp      : ROS builtin_interfaces/Time stamp for the marker headers.
        """
        # ── Marker 1 — RANSAC inlier points (green POINTS) ────────────────────
        m_pts = Marker()
        m_pts.header.frame_id  = self._target_frame
        m_pts.header.stamp     = stamp
        m_pts.ns               = "ransac_inliers"
        m_pts.id               = 0
        m_pts.type             = Marker.POINTS
        m_pts.action           = Marker.ADD
        m_pts.scale.x          = 0.05   # point width  [m]
        m_pts.scale.y          = 0.05   # point height [m]
        m_pts.scale.z          = 0.05
        m_pts.color.r          = 0.0
        m_pts.color.g          = 1.0
        m_pts.color.b          = 0.0
        m_pts.color.a          = 1.0    # fully opaque

        for row in inlier_pts:
            p = Point()
            if swap_axes:
                # Regression was done with axes swapped: col-0 = odom-Y, col-1 = odom-X
                p.x = float(row[1])
                p.y = float(row[0])
            else:
                p.x = float(row[0])
                p.y = float(row[1])
            p.z = 0.0
            m_pts.points.append(p)

        self._debug_cluster_pub.publish(m_pts)

        # ── Marker 2 — Polynomial curve (red LINE_STRIP) ──────────────────────
        m_curve = Marker()
        m_curve.header.frame_id = self._target_frame
        m_curve.header.stamp    = stamp
        m_curve.ns              = "ransac_curve"
        m_curve.id              = 1
        m_curve.type            = Marker.LINE_STRIP
        m_curve.action          = Marker.ADD
        m_curve.scale.x         = 0.02   # line width [m]
        m_curve.scale.y         = 0.0
        m_curve.scale.z         = 0.0
        m_curve.color.r         = 1.0
        m_curve.color.g         = 0.0
        m_curve.color.b         = 0.0
        m_curve.color.a         = 1.0    # fully opaque

        # Sample 50 points along the regression-space X axis
        x_min = float(inlier_pts[:, 0].min())
        x_max = float(inlier_pts[:, 0].max())
        x_samples = np.linspace(x_min, x_max, 50)
        y_samples  = np.polyval(coeffs, x_samples)

        for xi, yi in zip(x_samples, y_samples):
            p = Point()
            if swap_axes:
                # Same axis restoration as above
                p.x = float(yi)
                p.y = float(xi)
            else:
                p.x = float(xi)
                p.y = float(yi)
            p.z = 0.0
            m_curve.points.append(p)

        self._debug_curve_pub.publish(m_curve)

        # ── Marker 2 — Raw pre-DBSCAN point cloud (blue-grey POINTS) ──────────────
        m_raw = Marker()
        m_raw.header.frame_id = self._target_frame
        m_raw.header.stamp    = stamp
        m_raw.ns              = "raw_points"
        m_raw.id              = 2
        m_raw.type            = Marker.POINTS
        m_raw.action          = Marker.ADD
        m_raw.scale.x         = 0.02   # smaller than inliers to stay in background
        m_raw.scale.y         = 0.02
        m_raw.scale.z         = 0.02
        m_raw.color.r         = 0.5
        m_raw.color.g         = 0.5
        m_raw.color.b         = 0.8
        m_raw.color.a         = 0.6    # semi-transparent

        # raw_pts is already in the odom frame (no swap_axes needed)
        for i in range(len(raw_pts)):
            p = Point()
            p.x = float(raw_pts[i, 0])
            p.y = float(raw_pts[i, 1])
            p.z = 0.0
            m_raw.points.append(p)

        self._debug_raw_pub.publish(m_raw)

    # ──────────────────────────────────────────────────────────────────────────
    # DBSCAN
    # ──────────────────────────────────────────────────────────────────────────

    def _run_dbscan(self, pts: np.ndarray) -> np.ndarray | None:
        """
        Run DBSCAN on the (N, 2) point matrix.

        Returns the label array (−1 = noise / fish) or None on failure.
        """
        try:
            db = DBSCAN(
                eps=self._dbscan_eps,
                min_samples=self._dbscan_min_pts,
                metric="euclidean",
                n_jobs=1,  # deterministic, no parallelism
            )
            labels = db.fit_predict(pts)
            return labels
        except Exception as exc:
            self.get_logger().warn(f"[ping360_nearest] DBSCAN failed: {exc}")
            return None

    # ──────────────────────────────────────────────────────────────────────────
    # Net cluster selection via RANSAC inlier ratio
    # ──────────────────────────────────────────────────────────────────────────

    def _select_net_cluster_ransac(
        self,
        pts: np.ndarray,
        labels: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """
        Select the cluster most likely to be the aquaculture net by testing the
        RANSAC inlier ratio of a degree-2 polynomial fit on each candidate cluster.

        Rationale:
        - An aquaculture net, even heavily curved by currents, is a quasi-2D
          surface structure → points fit well onto a parabola → high inlier ratio
          (≥ ransac_min_inlier_ratio).
        - A fish school is volumetric → points are scattered in 2D sonar space →
          very low inlier ratio.

        This heuristic is robust where global PCA fails: a deeply bowed net has
        near-isotropic variance (PCA ratio ≈ 1) but still yields a high RANSAC
        inlier ratio.

        Args:
            pts    : (N, 2) matrix of all buffer points in the odom frame.
            labels : DBSCAN label array (−1 = noise).

        Returns:
            (cluster_pts, coeffs, inlier_pts_for_helpers, swap_axes) or None.
            - cluster_pts            : raw points of the selected cluster (M, 2)
            - coeffs                 : [a, b, c] RANSAC polynomial coefficients
            - inlier_pts_for_helpers : inlier points in regression space
                                       (for _poly2_closest_point and _poly2_normal_yaw)
            - swap_axes              : True if X/Y axes were swapped during regression
        """
        unique_labels = set(labels)
        unique_labels.discard(-1)   # ignore DBSCAN noise

        if not unique_labels:
            self.get_logger().warn(
                f"[ping360_nearest] ✗ DBSCAN: 0 clusters found (all noise). "
                f"n_points={len(pts)}, eps={self._dbscan_eps}, "
                f"min_samples={self._dbscan_min_pts}"
            )
            return None

        best_label        = None
        best_inlier_ratio = -1.0
        best_result       = None   # (coeffs, inlier_pts_for_helpers, swap_axes)
        best_cluster_pts  = None
        cluster_info      = []     # for diagnostic log
        too_small_count   = 0

        for lbl in unique_labels:
            cluster_pts = pts[labels == lbl]
            n = len(cluster_pts)

            if n < self._min_cluster:
                too_small_count += 1
                continue

            # Run degree-2 polynomial RANSAC on this cluster
            result = self._run_ransac_poly2(cluster_pts)
            if result is None:
                # RANSAC did not converge → cluster rejected (log emitted inside)
                cluster_info.append((lbl, n, 0.0))
                continue

            coeffs, inlier_pts_helpers, swap_axes = result
            inlier_ratio = len(inlier_pts_helpers) / n
            cluster_info.append((lbl, n, inlier_ratio))

            if inlier_ratio > best_inlier_ratio:
                best_inlier_ratio = inlier_ratio
                best_label        = lbl
                best_result       = (coeffs, inlier_pts_helpers, swap_axes)
                best_cluster_pts  = cluster_pts

        # ── Diagnostic log ─────────────────────────────────────────────────────
        if cluster_info:
            info_str = "  ".join(
                f"[lbl={l}, n={n}, inliers={r:.0%}]" for l, n, r in cluster_info
            )
            self.get_logger().debug(
                f"[ping360_nearest] RANSAC clusters: {info_str}  "
                f"→ Best label={best_label} "
                f"(ratio={best_inlier_ratio:.0%}, threshold={self._ransac_min_inlier:.0%})"
            )
        else:
            # All clusters were below min_cluster_pts
            n_labels  = len(unique_labels)
            noise_pts = int(np.sum(labels == -1))
            self.get_logger().debug(
                f"[ping360_nearest] ✗ DBSCAN found {n_labels} cluster(s) but ALL "
                f"have < {self._min_cluster} pts (min_cluster_pts). "
                f"DBSCAN noise: {noise_pts}/{len(pts)} pts."
            )
            return None

        # ── Validate the best candidate ────────────────────────────────────────
        if best_result is None or best_inlier_ratio < self._ransac_min_inlier:
            self.get_logger().warn(
                f"[ping360_nearest] ✗ RANSAC: no cluster exceeds the inlier "
                f"threshold ({self._ransac_min_inlier:.0%}). "
                f"Best ratio achieved: {best_inlier_ratio:.0%}. "
                "Possibly only volumetric fish schools in view."
            )
            return None

        coeffs, inlier_pts_helpers, swap_axes = best_result
        return best_cluster_pts, coeffs, inlier_pts_helpers, swap_axes

    # ──────────────────────────────────────────────────────────────────────────
    # Degree-2 polynomial RANSAC (net curve fitting)
    # ──────────────────────────────────────────────────────────────────────────

    def _run_ransac_poly2(
        self,
        pts: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Fit a degree-2 polynomial (parabola) to cluster points using manual RANSAC
        to be robust against residual outliers remaining after DBSCAN.

        Strategy:
        - Choose the regression axis via variance (PCA-lite): the axis with higher
          variance becomes the independent X axis, avoiding degeneracy for near-
          vertical nets.
        - RANSAC: randomly sample 3 points (minimum for a quadratic), fit a degree-2
          polynomial, count inliers (residual < _ransac_resid).
        - Return the best model coefficients and the corresponding inlier points.

        Returns:
            (coeffs, inlier_pts, swap_axes) where coeffs = [a, b, c] (degree-2
            polynomial in the possibly-swapped regression space), or None on failure.
        """
        if len(pts) < self._ransac_min_pts:
            self.get_logger().debug(
                f"[ping360_nearest] RANSAC poly2 skipped: {len(pts)} pts "
                f"< min={self._ransac_min_pts}."
            )
            return None

        # ── Choose regression axis via variance ───────────────────────────────
        # The axis with the highest variance becomes the independent X axis.
        # This prevents degeneracy when the net runs roughly vertical on screen.
        var_x = float(np.var(pts[:, 0]))
        var_y = float(np.var(pts[:, 1]))
        swap_axes = var_y > var_x   # swap if the net is near-vertical

        if swap_axes:
            X_fit = pts[:, 1].copy()  # y becomes the independent axis
            Y_fit = pts[:, 0].copy()  # x becomes the dependent variable
        else:
            X_fit = pts[:, 0].copy()
            Y_fit = pts[:, 1].copy()

        # ── Manual RANSAC on degree-2 polynomial ───────────────────────────────
        n_pts = len(pts)
        n_trials = 300
        best_inlier_mask = None
        best_n_inliers   = 0
        best_coeffs      = None

        rng = np.random.default_rng(seed=42)

        for _ in range(n_trials):
            # Sample 3 points (minimum for a quadratic fit)
            idx = rng.choice(n_pts, size=3, replace=False)
            X_s, Y_s = X_fit[idx], Y_fit[idx]

            try:
                coeffs = np.polyfit(X_s, Y_s, deg=2)
            except (np.linalg.LinAlgError, ValueError):
                continue

            # Compute residuals over all cluster points
            Y_pred   = np.polyval(coeffs, X_fit)
            residuals = np.abs(Y_fit - Y_pred)
            inlier_mask = residuals < self._ransac_resid
            n_inliers   = int(inlier_mask.sum())

            if n_inliers > best_n_inliers:
                best_n_inliers   = n_inliers
                best_inlier_mask = inlier_mask
                best_coeffs      = coeffs

        if best_coeffs is None or best_n_inliers < self._ransac_min_pts:
            self.get_logger().warn(
                f"[ping360_nearest] RANSAC poly2 did not converge "
                f"(best inliers={best_n_inliers}, required={self._ransac_min_pts})."
            )
            return None

        # ── Re-fit on all inliers for a more stable final model ───────────────
        X_in = X_fit[best_inlier_mask]
        Y_in = Y_fit[best_inlier_mask]
        try:
            final_coeffs = np.polyfit(X_in, Y_in, deg=2)
        except (np.linalg.LinAlgError, ValueError) as exc:
            self.get_logger().warn(
                f"[ping360_nearest] RANSAC poly2 re-fit failed: {exc}"
            )
            return None

        # Rebuild inlier points in the original (non-swapped) frame
        if swap_axes:
            # X_fit = pts[:,1], Y_fit = pts[:,0]
            inlier_pts = pts[best_inlier_mask]   # original points, filtered
        else:
            inlier_pts = pts[best_inlier_mask]

        self.get_logger().debug(
            f"[ping360_nearest] RANSAC poly2: "
            f"inliers={best_n_inliers}/{n_pts}  "
            f"a={final_coeffs[0]:.4f}  b={final_coeffs[1]:.4f}  "
            f"swap_axes={swap_axes}"
        )

        # Return coefficients in regression space + swap flag.
        # The callers (_poly2_closest_point / _poly2_normal_yaw) operate in this
        # space and expect [X_indep, Y_dep] ordering for the inlier points.
        if swap_axes:
            inlier_pts_for_helpers = np.column_stack([X_in, Y_in])
        else:
            inlier_pts_for_helpers = inlier_pts

        return final_coeffs, inlier_pts_for_helpers, swap_axes


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Standard ROS 2 entry point."""
    rclpy.init(args=args)
    node = Ping360NearestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[ping360_nearest] Keyboard interrupt — shutting down.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
