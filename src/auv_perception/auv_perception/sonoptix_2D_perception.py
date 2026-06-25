#!/usr/bin/env python3
"""
sonoptix_2D_perception.py
==========================
ROS 2 perception node for the Sonoptix ECHO multi-beam 2D sonar.

Purpose
-------
Process a live 25 Hz LaserScan stream from the Sonoptix ECHO sonar to
servo an AUV facing a deformed aquaculture net.

The Sonoptix ECHO is bridged from Gazebo as a sensor_msgs/LaserScan (the
same message type as the Ping360), NOT as a PointCloud2.  This node
converts each ray (range + angle) into a 2-D (x, y) Cartesian point
internally before running the RANSAC polynomial fit.

Processing Pipeline
-------------------
  A) Filtering     : Decode LaserScan rays → NumPy (N, 2) Cartesian array.
                     Reject NaN/Inf, out-of-range, and saturated echoes.
  B) RANSAC Poly2  : Fit a degree-2 polynomial (y = ax² + bx + c) via RANSAC.
                     Axis swap: the axis with the highest variance is chosen
                     as the independent variable to avoid near-vertical singularity.
                     Validate only when the inlier ratio ≥ min_inlier_ratio.
  C) Geometry      : Sample the fitted curve (500-point linspace) to find the
                     point closest to the sensor origin (≡ the AUV).
                     Compute the curve tangent at that point, derive the inward-
                     pointing normal, and convert to a Yaw angle (atan2).

Functional Publishers (control loop)
--------------------------------------
  /perception/net_distance     (std_msgs/Float32) — orthogonal distance [m]
  /perception/net_yaw_target   (std_msgs/Float32) — target yaw angle [rad]
  /perception/perception_valid (std_msgs/Bool)    — True if detection is valid

Debug / Visualisation Publishers (Foxglove 3D panel)
------------------------------------------------------
  ~/debug/raw_cloud    (visualization_msgs/Marker POINTS, grey)
    → All range-filtered Cartesian points fed into RANSAC.
  ~/debug/inlier_cloud (visualization_msgs/Marker POINTS, green)
    → Only the RANSAC inlier points (the net echo).
  ~/debug/ransac_curve (visualization_msgs/Marker LINE_STRIP, red)
    → 80-point sampling of the fitted degree-2 parabola.
  ~/debug/normal_arrow (visualization_msgs/Marker ARROW, cyan)
    → Arrow from the closest curve point toward the AUV (normal direction).

Subscriber
----------
  /sonoptix/points (sensor_msgs/LaserScan) — raw sonar scan from ros_gz_bridge

ROS 2 Parameters
----------------
  min_range                 (float, 0.3)  — near dead-zone [m]
  max_range                 (float, 7.0)  — far cut-off [m]
  ransac_residual_threshold (float, 0.05) — RANSAC inlier distance threshold [m]
  ransac_min_inliers_ratio  (float, 0.30) — minimum inlier fraction to validate
  min_points                (int,   10)   — minimum points to attempt RANSAC

Author  : titou
Package : auv_perception
"""

# ── Standard library ──────────────────────────────────────────────────────────
import math
import time

# ── Scientific computing ──────────────────────────────────────────────────────
import numpy as np

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Messages ──────────────────────────────────────────────────────────────────
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker


# ── Default constants (overridden by ROS 2 parameters) ───────────────────────

_DEFAULT_MIN_RANGE             = 0.3     # [m]
_DEFAULT_MAX_RANGE             = 7.0     # [m]
_DEFAULT_RANSAC_RESIDUAL       = 0.05    # [m]  inlier distance threshold
_DEFAULT_RANSAC_MIN_INLIERS    = 0.30    # fraction [0–1]
_DEFAULT_MIN_POINTS            = 10      # min points to attempt RANSAC
_RANSAC_MAX_TRIALS             = 200     # maximum RANSAC iterations
_RANSAC_MIN_SAMPLE             = 3       # minimum points per hypothesis
_CURVE_SAMPLE_N                = 500     # linspace resolution for closest-point
_CURVE_VIZ_N                   = 80      # linspace resolution for debug curve marker


# ── Pure helpers (no ROS dependency) ─────────────────────────────────────────

def _laserscan_to_cartesian(
    msg: LaserScan,
    min_range: float,
    max_range: float,
) -> np.ndarray:
    """
    Convert a sensor_msgs/LaserScan into a (N, 2) float64 NumPy array of
    Cartesian points [x, y] in the sensor frame.

    Filtering applied:
      - NaN / Inf ranges → discarded
      - range < min_range → discarded (dead zone)
      - range > max_range → discarded (far cut-off)
      - range >= range_max * 0.99 → discarded (saturated / max-range echo)

    The sensor frame follows the ROS convention: X forward, Y left.

    Returns a (M, 2) array; M may be 0 if no valid rays exist.
    """
    ranges   = np.asarray(msg.ranges, dtype=np.float64)
    n_rays   = len(ranges)
    angles   = (msg.angle_min
                + np.arange(n_rays, dtype=np.float64) * msg.angle_increment)

    # Build mask: finite, within range, not saturated
    saturate_threshold = msg.range_max * 0.99
    mask = (
        np.isfinite(ranges)
        & (ranges >= min_range)
        & (ranges <= max_range)
        & (ranges < saturate_threshold)
    )

    valid_r = ranges[mask]
    valid_a = angles[mask]

    if len(valid_r) == 0:
        return np.empty((0, 2), dtype=np.float64)

    x = valid_r * np.cos(valid_a)
    y = valid_r * np.sin(valid_a)
    return np.stack([x, y], axis=1)


def _ransac_poly2(
    pts: np.ndarray,
    residual_threshold: float,
    min_inlier_ratio: float,
    max_trials: int = _RANSAC_MAX_TRIALS,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, np.ndarray, bool, float] | None:
    """
    Fit a degree-2 polynomial to a 2-D point cloud using RANSAC.

    Axis-swap heuristic
    -------------------
    If var(Y) > var(X), axes are swapped so the independent axis always has
    the most spread.  This avoids the near-vertical singularity.

    Returns
    -------
    (coeffs, inlier_pts, swap_axes, inlier_ratio)   or   None on failure.
      coeffs      : [a, b, c] polynomial coefficients (regression space).
      inlier_pts  : (M, 2) inlier points in regression space.
      swap_axes   : bool — True if X/Y were swapped before regression.
      inlier_ratio: float — fraction of pts classified as inliers.
    """
    if rng is None:
        rng = np.random.default_rng()

    n = len(pts)
    min_inlier_count = max(_RANSAC_MIN_SAMPLE, int(math.ceil(min_inlier_ratio * n)))

    # ── Axis-swap heuristic ────────────────────────────────────────────────────
    swap_axes = float(np.var(pts[:, 1])) > float(np.var(pts[:, 0]))
    if swap_axes:
        X_reg = pts[:, 1].copy()   # physical Y → regression X
        Y_reg = pts[:, 0].copy()   # physical X → regression Y
    else:
        X_reg = pts[:, 0].copy()
        Y_reg = pts[:, 1].copy()

    # ── RANSAC loop ────────────────────────────────────────────────────────────
    best_inlier_mask  = None
    best_inlier_count = 0
    best_coeffs       = None

    for _ in range(max_trials):
        idx   = rng.choice(n, size=_RANSAC_MIN_SAMPLE, replace=False)
        xs, ys = X_reg[idx], Y_reg[idx]

        try:
            coeffs = np.polyfit(xs, ys, deg=2)
        except (np.linalg.LinAlgError, ValueError):
            continue

        residuals    = np.abs(Y_reg - np.polyval(coeffs, X_reg))
        inlier_mask  = residuals < residual_threshold
        inlier_count = int(inlier_mask.sum())

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inlier_mask  = inlier_mask
            best_coeffs       = coeffs

        if best_inlier_count == n:
            break

    if best_coeffs is None or best_inlier_count < min_inlier_count:
        return None

    # Re-fit on ALL inliers for a more stable estimate
    xs_in = X_reg[best_inlier_mask]
    ys_in = Y_reg[best_inlier_mask]
    try:
        coeffs_final = np.polyfit(xs_in, ys_in, deg=2)
    except (np.linalg.LinAlgError, ValueError):
        coeffs_final = best_coeffs

    inlier_pts   = np.stack([xs_in, ys_in], axis=1)
    inlier_ratio = best_inlier_count / n
    return coeffs_final, inlier_pts, swap_axes, inlier_ratio


def _closest_point_on_curve(
    coeffs: np.ndarray,
    inlier_pts: np.ndarray,
) -> tuple[float, float]:
    """
    Find the point on  y_reg = a·x_reg² + b·x_reg + c  closest to the
    sensor origin (0, 0) in regression space, via 500-point linspace sampling.
    """
    x_min = float(inlier_pts[:, 0].min())
    x_max = float(inlier_pts[:, 0].max())
    xs    = np.linspace(x_min, x_max, _CURVE_SAMPLE_N)
    ys    = np.polyval(coeffs, xs)
    idx   = int(np.argmin(xs ** 2 + ys ** 2))
    return float(xs[idx]), float(ys[idx])


def _curve_normal_yaw(
    coeffs: np.ndarray,
    x0: float,
    swap_axes: bool,
    x_close_phys: float,
    y_close_phys: float,
) -> float:
    """
    Inward-pointing normal of the parabola at x0 → Yaw angle (rad) in the
    physical sensor frame.

    The orientation disambiguation (which of the two perpendicular directions
    is "toward the AUV") is done in **physical space** rather than in
    regression space.  This avoids the numerical instability that arises when
    the regression-space "toward origin" vector has near-zero components (e.g.
    when the closest curve point is nearly on the regression X-axis).

    Physical convention: the AUV sensor sits at (0, 0), the net is always in
    the positive-X half-space of the sensor.  Therefore the inward normal
    (pointing from net toward sensor) must have a negative dot product with
    the vector from the sensor to the closest curve point, i.e. its dot
    product with (x_close_phys, y_close_phys) must be negative.
    """
    a, b  = float(coeffs[0]), float(coeffs[1])
    slope = 2.0 * a * x0 + b          # dy_reg / dx_reg at x0

    # Normal in regression space (one of two anti-parallel directions)
    normal_reg = np.array([-slope, 1.0])
    n_mag = np.linalg.norm(normal_reg)
    normal_reg = normal_reg / n_mag if n_mag > 1e-9 else np.array([0.0, 1.0])

    # Restore to physical (x, y) frame
    if swap_axes:
        # regression col-0 ≡ Y_phys, col-1 ≡ X_phys
        normal_phys = np.array([normal_reg[1], normal_reg[0]])
    else:
        normal_phys = normal_reg.copy()

    # ── Orient toward the AUV in physical space (stable) ──────────────────────
    # The vector from the sensor origin to the closest point on the net is
    # (x_close_phys, y_close_phys).  The inward normal must point in the
    # OPPOSITE direction, so dot(normal_phys, toward_net) < 0.
    toward_net = np.array([x_close_phys, y_close_phys])
    if np.dot(normal_phys, toward_net) > 0.0:
        normal_phys = -normal_phys

    return math.atan2(normal_phys[1], normal_phys[0])


def _reg_to_phys(x_reg: float, y_reg: float, swap_axes: bool) -> tuple[float, float]:
    """Convert a regression-space point back to physical (x, y)."""
    return (y_reg, x_reg) if swap_axes else (x_reg, y_reg)


def _make_marker_base(frame_id: str, stamp, ns: str, marker_id: int,
                      marker_type: int) -> Marker:
    """Return a Marker with all mandatory fields pre-filled."""
    m = Marker()
    m.header.frame_id    = frame_id
    if stamp is not None:
        m.header.stamp   = stamp
    m.ns                 = ns
    m.id                 = marker_id
    m.type               = marker_type
    m.action             = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


# ── Main ROS 2 node ───────────────────────────────────────────────────────────

class Sonoptix2DPerceptionNode(Node):
    """
    ROS 2 node – Sonoptix ECHO 2-D polynomial RANSAC perception.

    Subscribes to /sonoptix/points (sensor_msgs/LaserScan), converts rays to
    Cartesian points, fits a degree-2 RANSAC parabola to the net echo, and
    publishes distance / yaw / validity plus four Foxglove debug markers.
    """

    def __init__(self) -> None:
        super().__init__('sonoptix_2D_perception')

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter('min_range',                 _DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range',                 _DEFAULT_MAX_RANGE)
        self.declare_parameter('ransac_residual_threshold', _DEFAULT_RANSAC_RESIDUAL)
        self.declare_parameter('ransac_min_inliers_ratio',  _DEFAULT_RANSAC_MIN_INLIERS)
        self.declare_parameter('min_points',                _DEFAULT_MIN_POINTS)

        self._min_range    = float(self.get_parameter('min_range').value)
        self._max_range    = float(self.get_parameter('max_range').value)
        self._ransac_resid = float(self.get_parameter('ransac_residual_threshold').value)
        self._ransac_ratio = float(self.get_parameter('ransac_min_inliers_ratio').value)
        self._min_points   = int(self.get_parameter('min_points').value)

        self._rng = np.random.default_rng(seed=42)

        # ── QoS: Best-Effort to match ros_gz_bridge ───────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber — LaserScan (NOT PointCloud2!) ─────────────────────────
        self.create_subscription(
            LaserScan,
            '/sonoptix/points',
            self._scan_cb,
            best_effort_qos,
        )

        # ── Functional publishers ──────────────────────────────────────────────
        self._pub_distance = self.create_publisher(Float32, '/perception/net_distance', 10)
        self._pub_yaw      = self.create_publisher(Float32, '/perception/net_yaw_target', 10)
        self._pub_valid    = self.create_publisher(Bool,    '/perception/perception_valid', 10)

        # ── Debug / Foxglove markers (under node namespace ~/) ─────────────────
        self._pub_raw_cloud    = self.create_publisher(Marker, '~/debug/raw_cloud',    10)
        self._pub_inlier_cloud = self.create_publisher(Marker, '~/debug/inlier_cloud', 10)
        self._pub_ransac_curve = self.create_publisher(Marker, '~/debug/ransac_curve', 10)
        self._pub_normal_arrow = self.create_publisher(Marker, '~/debug/normal_arrow', 10)

        # ── Counters ───────────────────────────────────────────────────────────
        self._n_received = 0
        self._n_valid    = 0
        self._n_invalid  = 0

        # ── Temporal yaw filter (EMA) ──────────────────────────────────────────
        # Smooths residual sign-flip noise on the yaw output.
        # alpha=0 → infinite smoothing (no response), alpha=1 → no smoothing.
        self._yaw_ema: float | None = None
        self._yaw_ema_alpha: float  = 0.25    # tune: smaller = smoother

        self.get_logger().info(
            '\n[Sonoptix2DPerception] Node started\n'
            f'  Subscriber              : /sonoptix/points (sensor_msgs/LaserScan)\n'
            f'  Functional outputs      : /perception/net_distance\n'
            f'                            /perception/net_yaw_target\n'
            f'                            /perception/perception_valid\n'
            f'  Foxglove viz topics     : ~/debug/raw_cloud    (grey  POINTS)\n'
            f'                            ~/debug/inlier_cloud (green POINTS)\n'
            f'                            ~/debug/ransac_curve (red   LINE_STRIP)\n'
            f'                            ~/debug/normal_arrow (cyan  ARROW)\n'
            f'  Range filter            : [{self._min_range:.2f} m — {self._max_range:.2f} m]\n'
            f'  RANSAC residual         : {self._ransac_resid:.3f} m\n'
            f'  RANSAC min inlier ratio : {self._ransac_ratio:.0%}\n'
            f'  Min points for RANSAC   : {self._min_points}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Main callback
    # ──────────────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        """
        Process one Sonoptix LaserScan and publish results + debug markers.

        A) LaserScan → Cartesian (N, 2) with range filter
        B) RANSAC degree-2 polynomial with axis-swap heuristic
        C) Closest point on curve → distance + normal → yaw
        D) Publish functional topics + Foxglove markers
        """
        t0 = time.perf_counter()
        self._n_received += 1

        # Determine the frame for all markers (use the scan's own frame_id)
        frame_id = msg.header.frame_id if msg.header.frame_id else 'sonoptix_link'
        stamp    = msg.header.stamp

        # ── A) LaserScan → filtered Cartesian points ───────────────────────────
        pts = _laserscan_to_cartesian(msg, self._min_range, self._max_range)

        # Always publish the raw filtered cloud so the user can debug even when
        # RANSAC fails
        self._publish_raw_cloud(pts, frame_id, stamp)

        if len(pts) < self._min_points:
            if self._n_received % 25 == 1:
                self.get_logger().debug(
                    f'[Sonoptix2D] Too few points after filter: '
                    f'{len(pts)} (need ≥ {self._min_points}). '
                    f'total_rays={len(msg.ranges)}'
                )
            self._publish_invalid()
            return

        # ── B) RANSAC polynomial fit ───────────────────────────────────────────
        result = _ransac_poly2(
            pts,
            residual_threshold=self._ransac_resid,
            min_inlier_ratio=self._ransac_ratio,
            rng=self._rng,
        )

        if result is None:
            self._n_invalid += 1
            if self._n_invalid % 10 == 1:
                self.get_logger().warn(
                    f'[Sonoptix2D] RANSAC failed (#{self._n_invalid}, pts={len(pts)}). '
                    f'Try lowering ransac_min_inliers_ratio or ransac_residual_threshold.'
                )
            self._publish_invalid()
            return

        coeffs, inlier_pts, swap_axes, inlier_ratio = result

        # ── C) Geometry ────────────────────────────────────────────────────────
        x_close_reg, y_close_reg = _closest_point_on_curve(coeffs, inlier_pts)
        x_close_phys, y_close_phys = _reg_to_phys(x_close_reg, y_close_reg, swap_axes)
        net_distance = float(math.sqrt(x_close_phys ** 2 + y_close_phys ** 2))

        # Normal computed in physical space → stable sign
        yaw_raw = _curve_normal_yaw(
            coeffs, x_close_reg, swap_axes, x_close_phys, y_close_phys)

        # EMA temporal filter — handles residual angle noise using
        # angular arithmetic to avoid wrap-around artefacts.
        if self._yaw_ema is None:
            self._yaw_ema = yaw_raw
        else:
            # Angular difference in [-π, π]
            diff = (yaw_raw - self._yaw_ema + math.pi) % (2 * math.pi) - math.pi
            self._yaw_ema += self._yaw_ema_alpha * diff
            self._yaw_ema = (self._yaw_ema + math.pi) % (2 * math.pi) - math.pi

        yaw_target = self._yaw_ema

        # ── D) Publish functional topics ───────────────────────────────────────
        dist_msg  = Float32(); dist_msg.data  = net_distance
        yaw_msg   = Float32(); yaw_msg.data   = float(yaw_target)
        valid_msg = Bool();    valid_msg.data  = True

        self._pub_distance.publish(dist_msg)
        self._pub_yaw.publish(yaw_msg)
        self._pub_valid.publish(valid_msg)

        # ── D) Publish Foxglove markers ────────────────────────────────────────
        self._publish_inlier_cloud(inlier_pts, swap_axes, frame_id, stamp)
        self._publish_ransac_curve(coeffs, inlier_pts, swap_axes, frame_id, stamp)
        self._publish_normal_arrow(x_close_phys, y_close_phys, yaw_target, frame_id, stamp)

        self._n_valid   += 1
        self._n_invalid  = 0

        elapsed_ms = (time.perf_counter() - t0) * 1e3
        if self._n_valid % 25 == 1:
            self.get_logger().debug(
                f'[Sonoptix2D] ✔ #{self._n_valid:5d} | '
                f'pts={len(pts):3d}  inliers={inlier_ratio:.0%}  '
                f'dist={net_distance:.3f} m  '
                f'yaw_raw={math.degrees(yaw_raw):+.1f}°  '
                f'yaw_filt={math.degrees(yaw_target):+.1f}°  '
                f'swap={swap_axes}  dt={elapsed_ms:.1f} ms'
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Functional helper
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_invalid(self) -> None:
        msg = Bool(); msg.data = False
        self._pub_valid.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # Debug marker publishers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_raw_cloud(self, pts: np.ndarray, frame_id: str, stamp) -> None:
        """Grey POINTS — all points that passed the range filter."""
        m = _make_marker_base(frame_id, stamp, 'raw_cloud', 0, Marker.POINTS)
        m.scale.x = 0.04; m.scale.y = 0.04; m.scale.z = 0.04
        m.color.r = 0.6;  m.color.g = 0.6;  m.color.b = 0.6; m.color.a = 0.7

        for row in pts:
            p = Point(); p.x = float(row[0]); p.y = float(row[1]); p.z = 0.0
            m.points.append(p)

        self._pub_raw_cloud.publish(m)

    def _publish_inlier_cloud(self, inlier_pts: np.ndarray, swap_axes: bool,
                               frame_id: str, stamp) -> None:
        """Green POINTS — RANSAC inlier points (= the detected net)."""
        m = _make_marker_base(frame_id, stamp, 'inlier_cloud', 1, Marker.POINTS)
        m.scale.x = 0.06; m.scale.y = 0.06; m.scale.z = 0.06
        m.color.r = 0.0;  m.color.g = 1.0;  m.color.b = 0.2; m.color.a = 1.0

        for row in inlier_pts:
            xp, yp = _reg_to_phys(float(row[0]), float(row[1]), swap_axes)
            p = Point(); p.x = xp; p.y = yp; p.z = 0.0
            m.points.append(p)

        self._pub_inlier_cloud.publish(m)

    def _publish_ransac_curve(self, coeffs: np.ndarray, inlier_pts: np.ndarray,
                               swap_axes: bool, frame_id: str, stamp) -> None:
        """Red LINE_STRIP — the fitted parabola sampled at 80 points."""
        m = _make_marker_base(frame_id, stamp, 'ransac_curve', 2, Marker.LINE_STRIP)
        m.scale.x = 0.03   # line width [m]
        m.color.r = 1.0; m.color.g = 0.15; m.color.b = 0.0; m.color.a = 1.0

        x_min = float(inlier_pts[:, 0].min())
        x_max = float(inlier_pts[:, 0].max())
        for xi in np.linspace(x_min, x_max, _CURVE_VIZ_N):
            yi = float(np.polyval(coeffs, xi))
            xp, yp = _reg_to_phys(xi, yi, swap_axes)
            p = Point(); p.x = xp; p.y = yp; p.z = 0.0
            m.points.append(p)

        self._pub_ransac_curve.publish(m)

    def _publish_normal_arrow(self, x_close: float, y_close: float,
                               yaw: float, frame_id: str, stamp) -> None:
        """
        Cyan ARROW — from the closest point on the curve to the AUV origin.

        tail = (x_close, y_close) on the parabola
        tip  = (0, 0) = sensor origin = AUV position in sensor frame
        """
        m = _make_marker_base(frame_id, stamp, 'normal_arrow', 3, Marker.ARROW)
        m.scale.x = 0.05   # shaft diameter
        m.scale.y = 0.10   # head diameter
        m.scale.z = 0.12   # head length
        m.color.r = 0.0; m.color.g = 0.85; m.color.b = 1.0; m.color.a = 1.0

        tail = Point(); tail.x = x_close; tail.y = y_close; tail.z = 0.0
        tip  = Point(); tip.x  = 0.0;     tip.y  = 0.0;     tip.z  = 0.0
        m.points = [tail, tip]

        self._pub_normal_arrow.publish(m)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Sonoptix2DPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[Sonoptix2D] Keyboard interrupt — shutting down.')
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
