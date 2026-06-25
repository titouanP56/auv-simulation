#!/usr/bin/env python3
"""
ping360_circle_fitting.py
=========================
ROS 2 perception node — Circle fitting on Ping360 PointCloud2 data.

Processing pipeline
-------------------
  1. Subscribe to /ping360/points  (sensor_msgs/PointCloud2 — full 360° scan, Z≈0).
  2. Filter out NaN values and points too close to the robot (dead zone).
  3. Run a RANSAC circle-fitting loop (pure NumPy — no heavy external library):
       a. Randomly draw 3 points.
       b. Compute circumscribed circle centre (Xc, Yc) and radius R.
       c. Reject if R ∉ [min_radius, max_radius].
       d. Count inliers: |dist(pt, centre) − R| < ransac_distance_threshold.
       e. Keep the model with the most inliers.
  4. If the best inlier ratio ≥ min_inlier_ratio → refine with least-squares on
     inliers and publish the result on three topics.

Published topics
----------------
  /perception/cage_radius   (std_msgs/Float32)          — estimated radius [m]
  /perception/cage_center   (geometry_msgs/PointStamped) — (Xc, Yc) in sensor frame
  /perception/circle_valid  (std_msgs/Bool)              — True if result is valid

ROS 2 parameters
----------------
  min_radius               float   default 5.0   [m]
  max_radius               float   default 25.0  [m]
  min_dead_zone            float   default 0.5   [m]  — robot body exclusion radius
  ransac_iterations        int     default 1000
  ransac_distance_threshold float  default 0.3   [m]
  min_inlier_ratio         float   default 0.4   (0–1)

Notes
-----
- Callback frequency is intentionally low (~0.5–1 Hz) because it waits for a
  complete 360° sonar revolution.
- All maths in the sensor frame (Z ignored, pure 2-D).

Author  : titou
Package : auv_perception
"""

# ── Standard library ──────────────────────────────────────────────────────────
import struct
import math

# ── Scientific computing (pure NumPy — no sklearn / scipy required) ───────────
import numpy as np

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Messages ──────────────────────────────────────────────────────────────────
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

# ── Default parameter values ──────────────────────────────────────────────────
_DEFAULT_MIN_RADIUS     = 5.0    # [m]  — smallest plausible cage radius
_DEFAULT_MAX_RADIUS     = 25.0   # [m]  — largest  plausible cage radius
_DEFAULT_DEAD_ZONE      = 0.5    # [m]  — robot body exclusion radius
_DEFAULT_RANSAC_ITER    = 1000   # number of RANSAC iterations
_DEFAULT_RANSAC_DIST    = 0.3    # [m]  — inlier distance threshold
_DEFAULT_MIN_INLIER     = 0.4    # fraction (0–1)


# ─────────────────────────────────────────────────────────────────────────────
# Pure-NumPy helper — circumscribed circle through three 2-D points
# ─────────────────────────────────────────────────────────────────────────────

def _circumscribed_circle(
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
) -> tuple[float, float, float] | None:
    """
    Compute the circumscribed circle (centre + radius) of the triangle
    formed by three 2-D points.

    Uses the algebraic formulation of the circumcentre via the perpendicular
    bisectors — numerically stable for well-separated, non-collinear points.

    Args:
        p1, p2, p3 : (2,) NumPy arrays — [x, y] coordinates.

    Returns:
        (cx, cy, r) or None if the three points are nearly collinear (denominator
        below a numerical tolerance).
    """
    ax, ay = float(p1[0]), float(p1[1])
    bx, by = float(p2[0]), float(p2[1])
    cx, cy = float(p3[0]), float(p3[1])

    # Denominador del sistema lineal (2 × área del triángulo × 2)
    denom = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

    if abs(denom) < 1e-10:
        return None   # degenerate / collinear points

    ux = (
        (ax**2 + ay**2) * (by - cy)
        + (bx**2 + by**2) * (cy - ay)
        + (cx**2 + cy**2) * (ay - by)
    ) / denom

    uy = (
        (ax**2 + ay**2) * (cx - bx)
        + (bx**2 + by**2) * (ax - cx)
        + (cx**2 + cy**2) * (bx - ax)
    ) / denom

    r = math.hypot(ax - ux, ay - uy)
    return ux, uy, r


# ─────────────────────────────────────────────────────────────────────────────
# Least-squares circle fitting on a set of inlier points (Coope / algebraic)
# ─────────────────────────────────────────────────────────────────────────────

def _least_squares_circle(pts: np.ndarray) -> tuple[float, float, float] | None:
    """
    Fit the best circle to a set of 2-D points using the algebraic
    least-squares method (Coope 1993 — linear system, no iterative solver).

    Minimises  Σ |(xi − cx)² + (yi − cy)² − r²|².

    The system is:   A · [cx, cy, k]ᵀ = b
    where  k = cx² + cy² − r²,  A_i = [2xi, 2yi, 1],  b_i = xi² + yi².

    Args:
        pts : (N, 2) float array.

    Returns:
        (cx, cy, r)  or  None if the system is rank-deficient (N < 3).
    """
    if len(pts) < 3:
        return None

    x = pts[:, 0]
    y = pts[:, 1]

    # Build A (N×3) and b (N,)
    A = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(pts))])
    b = x**2 + y**2

    # Solve via least-squares
    result, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)

    if rank < 3:
        return None

    cx, cy, k = result
    r_sq = cx**2 + cy**2 - k
    if r_sq < 0.0:
        return None

    return float(cx), float(cy), float(math.sqrt(r_sq))


# ─────────────────────────────────────────────────────────────────────────────
# PointCloud2 fast XY extraction (pure Python struct unpacking — no ros2_numpy)
# ─────────────────────────────────────────────────────────────────────────────

def _extract_xy_from_pointcloud2(msg: PointCloud2) -> np.ndarray:
    """
    Extract the X and Y coordinates from a PointCloud2 message.

    This function uses Python's ``struct`` module to unpack the binary data
    directly, avoiding any external dependencies (ros2_numpy / sensor_msgs_py).
    It handles both big-endian and little-endian messages and arbitrary field
    offsets.

    Args:
        msg : sensor_msgs/PointCloud2 message.

    Returns:
        (N, 2) float64 NumPy array of [x, y] coordinates.
        Returns an empty (0, 2) array if the message contains no valid data.
    """
    # Locate field offsets for x and y
    field_offsets: dict[str, int] = {}
    for field in msg.fields:
        if field.name in ("x", "y"):
            field_offsets[field.name] = field.offset

    if "x" not in field_offsets or "y" not in field_offsets:
        return np.empty((0, 2), dtype=np.float64)

    off_x = field_offsets["x"]
    off_y = field_offsets["y"]
    point_step = msg.point_step
    n_points = msg.width * msg.height
    endian_char = "<" if not msg.is_bigendian else ">"

    data = bytes(msg.data)
    xy = np.empty((n_points, 2), dtype=np.float64)
    fmt_x = endian_char + "f"
    fmt_y = endian_char + "f"

    for i in range(n_points):
        base = i * point_step
        xy[i, 0] = struct.unpack_from(fmt_x, data, base + off_x)[0]
        xy[i, 1] = struct.unpack_from(fmt_y, data, base + off_y)[0]

    return xy


# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class Ping360CircleFittingNode(Node):
    """
    ROS 2 perception node — RANSAC circle fitting on Ping360 360° PointCloud2.

    Estimates the radius and centre of the aquaculture cage (assumed cylindrical
    or conical) observed by the Ping360 sonar mounted on the AUV.

    The callback runs at low frequency (~0.5–1 Hz) because it waits for a full
    360° sonar revolution to accumulate before processing.
    """

    def __init__(self) -> None:
        super().__init__("ping360_circle_fitting")

        # ── Parameter declarations ─────────────────────────────────────────────
        self.declare_parameter("min_radius",               _DEFAULT_MIN_RADIUS)
        self.declare_parameter("max_radius",               _DEFAULT_MAX_RADIUS)
        self.declare_parameter("min_dead_zone",            _DEFAULT_DEAD_ZONE)
        self.declare_parameter("ransac_iterations",        _DEFAULT_RANSAC_ITER)
        self.declare_parameter("ransac_distance_threshold", _DEFAULT_RANSAC_DIST)
        self.declare_parameter("min_inlier_ratio",         _DEFAULT_MIN_INLIER)

        # ── Read parameters ────────────────────────────────────────────────────
        self._min_radius  = float(self.get_parameter("min_radius").value)
        self._max_radius  = float(self.get_parameter("max_radius").value)
        self._dead_zone   = float(self.get_parameter("min_dead_zone").value)
        self._ransac_iter = int(self.get_parameter("ransac_iterations").value)
        self._ransac_dist = float(self.get_parameter("ransac_distance_threshold").value)
        self._min_inlier  = float(self.get_parameter("min_inlier_ratio").value)

        # ── Diagnostic counters ────────────────────────────────────────────────
        self._n_msgs_received  = 0
        self._n_valid_pub      = 0
        self._n_invalid_pub    = 0

        # ── QoS — Best Effort to match the Ping360 bridge ─────────────────────
        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscription ───────────────────────────────────────────────────────
        self._pc_sub = self.create_subscription(
            PointCloud2,
            "/ping360/points",
            self._pointcloud_callback,
            pc_qos,
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self._radius_pub = self.create_publisher(
            Float32,
            "/perception/cage_radius",
            10,
        )
        self._center_pub = self.create_publisher(
            PointStamped,
            "/perception/cage_center",
            10,
        )
        self._valid_pub = self.create_publisher(
            Bool,
            "/perception/circle_valid",
            10,
        )
        self._marker_pub = self.create_publisher(
            Marker,
            "/perception/cage_circle_marker",
            10,
        )
        self._inliers_marker_pub = self.create_publisher(
            Marker,
            "/perception/cage_inliers_marker",
            10,
        )

        self.get_logger().info(
            f"\n[ping360_circle_fitting] Node started — RANSAC circle fitting\n"
            f"  Input  : /ping360/points\n"
            f"  Outputs: /perception/cage_radius\n"
            f"           /perception/cage_center\n"
            f"           /perception/circle_valid\n"
            f"  Radius filter     : [{self._min_radius:.1f} m — {self._max_radius:.1f} m]\n"
            f"  Dead zone         : {self._dead_zone:.2f} m (robot body exclusion)\n"
            f"  RANSAC iterations : {self._ransac_iter}\n"
            f"  RANSAC threshold  : {self._ransac_dist:.3f} m\n"
            f"  Min inlier ratio  : {self._min_inlier:.0%}"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # PointCloud2 callback
    # ──────────────────────────────────────────────────────────────────────────

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Main callback — processes a full 360° PointCloud2 from the Ping360.

        Pipeline:
          A. Extract XY coordinates from the binary PointCloud2 payload.
          B. Filter: remove NaN/Inf values and points within the dead zone.
          C. Run RANSAC circle fitting.
          D. Refine with least-squares if RANSAC succeeded.
          E. Publish results.
        """
        self._n_msgs_received += 1

        # ── A. Extract XY ──────────────────────────────────────────────────────
        pts_raw = _extract_xy_from_pointcloud2(msg)

        if pts_raw.shape[0] == 0:
            self.get_logger().warn(
                "[ping360_circle_fitting] Received empty PointCloud2 — no x/y fields?"
            )
            self._publish_invalid(msg.header)
            return

        # ── B. Filter ──────────────────────────────────────────────────────────
        pts = self._filter_points(pts_raw)

        if len(pts) < 3:
            self.get_logger().warn(
                f"[ping360_circle_fitting] Not enough valid points after filtering: "
                f"{len(pts)} (need ≥ 3). raw={pts_raw.shape[0]}"
            )
            self._publish_invalid(msg.header)
            return

        self.get_logger().debug(
            f"[ping360_circle_fitting] msg #{self._n_msgs_received}: "
            f"raw={pts_raw.shape[0]} pts → after filter={len(pts)} pts"
        )

        # ── C. RANSAC circle fitting ───────────────────────────────────────────
        ransac_result = self._ransac_circle(pts)

        if ransac_result is None:
            self.get_logger().debug(
                f"[ping360_circle_fitting] ✗ RANSAC found no valid circle model "
                f"in {self._ransac_iter} iterations "
                f"(radius range [{self._min_radius:.1f}, {self._max_radius:.1f}] m, "
                f"n_pts={len(pts)})"
            )
            self._publish_invalid(msg.header)
            return

        cx_ransac, cy_ransac, r_ransac, inlier_mask, inlier_ratio = ransac_result
        inlier_pts = pts[inlier_mask]

        self.get_logger().debug(
            f"[ping360_circle_fitting] RANSAC best model: "
            f"cx={cx_ransac:.3f} m, cy={cy_ransac:.3f} m, "
            f"r={r_ransac:.3f} m, inliers={inlier_mask.sum()}/{len(pts)} "
            f"({inlier_ratio:.1%})"
        )

        # ── D. Least-squares refinement on inliers ────────────────────────────
        ls_result = _least_squares_circle(inlier_pts)

        if ls_result is not None:
            cx, cy, radius = ls_result
            self.get_logger().debug(
                f"[ping360_circle_fitting] LS refinement: "
                f"cx={cx:.3f} m, cy={cy:.3f} m, r={radius:.3f} m"
            )
        else:
            # Fall back to RANSAC result if LS fails (shouldn't happen with ≥ 3 pts)
            cx, cy, radius = cx_ransac, cy_ransac, r_ransac
            self.get_logger().debug(
                "[ping360_circle_fitting] LS refinement failed — using RANSAC result"
            )

        # ── E. Publish ─────────────────────────────────────────────────────────
        self._publish_result(
            header=msg.header,
            cx=cx,
            cy=cy,
            radius=radius,
            inlier_ratio=inlier_ratio,
            n_inliers=int(inlier_mask.sum()),
            n_total=len(pts),
            inlier_pts=inlier_pts,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Filtering
    # ──────────────────────────────────────────────────────────────────────────

    def _filter_points(self, pts: np.ndarray) -> np.ndarray:
        """
        Remove invalid and near-robot points from the raw (N, 2) array.

        Filters applied (in order):
          1. Remove rows containing NaN or Inf.
          2. Remove points within the dead-zone radius (robot body echoes).

        Args:
            pts : (N, 2) raw XY array.

        Returns:
            (M, 2) filtered float64 array.
        """
        # 1. Remove NaN / Inf
        finite_mask = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1])
        pts = pts[finite_mask]

        if len(pts) == 0:
            return pts

        # 2. Remove dead-zone (too close to the robot)
        dist2 = pts[:, 0] ** 2 + pts[:, 1] ** 2
        alive_mask = dist2 >= self._dead_zone ** 2
        pts = pts[alive_mask]

        return pts

    # ──────────────────────────────────────────────────────────────────────────
    # RANSAC circle fitting
    # ──────────────────────────────────────────────────────────────────────────

    def _ransac_circle(
        self, pts: np.ndarray
    ) -> tuple[float, float, float, np.ndarray, float] | None:
        """
        RANSAC circle fitting on a (N, 2) point set.

        Algorithm:
          For each of ransac_iterations iterations:
            1. Draw 3 distinct random points.
            2. Compute the circumscribed circle of those 3 points.
            3. Reject if radius ∉ [min_radius, max_radius].
            4. Compute per-point residuals: |dist(pt, centre) − R|.
            5. Count inliers (residual < ransac_distance_threshold).
            6. Keep the iteration with the most inliers.

        Args:
            pts : (N, 2) filtered float64 point array.

        Returns:
            (cx, cy, radius, inlier_mask, inlier_ratio) of the best model, or
            None if no valid model was found.
        """
        n = len(pts)
        if n < 3:
            return None

        best_n_inliers   = 0
        best_inlier_mask: np.ndarray | None = None
        best_cx          = 0.0
        best_cy          = 0.0
        best_r           = 0.0

        rng = np.random.default_rng()  # reproducible within a run via seed if needed

        for _ in range(self._ransac_iter):
            # ── 1. Sample 3 distinct points ───────────────────────────────────
            idx = rng.choice(n, size=3, replace=False)
            p1, p2, p3 = pts[idx[0]], pts[idx[1]], pts[idx[2]]

            # ── 2. Circumscribed circle ───────────────────────────────────────
            circle = _circumscribed_circle(p1, p2, p3)
            if circle is None:
                continue
            cx, cy, r = circle

            # ── 3. Radius filter ──────────────────────────────────────────────
            if not (self._min_radius <= r <= self._max_radius):
                continue

            # ── 4. Residuals ──────────────────────────────────────────────────
            # dist(pt, centre) for all N points
            dists = np.sqrt((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2)
            residuals = np.abs(dists - r)

            # ── 5. Inlier mask ────────────────────────────────────────────────
            inlier_mask = residuals < self._ransac_dist
            n_inliers = int(inlier_mask.sum())

            # ── 6. Track best ─────────────────────────────────────────────────
            if n_inliers > best_n_inliers:
                best_n_inliers   = n_inliers
                best_inlier_mask = inlier_mask
                best_cx          = cx
                best_cy          = cy
                best_r           = r

        if best_inlier_mask is None or best_n_inliers == 0:
            return None

        inlier_ratio = best_n_inliers / n
        return best_cx, best_cy, best_r, best_inlier_mask, inlier_ratio

    # ──────────────────────────────────────────────────────────────────────────
    # Publishing helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_result(
        self,
        header,
        cx: float,
        cy: float,
        radius: float,
        inlier_ratio: float,
        n_inliers: int,
        n_total: int,
        inlier_pts: np.ndarray = None,
    ) -> None:
        """
        Decide validity and publish all three output topics.

        Validity criterion: inlier_ratio ≥ min_inlier_ratio.
        """
        is_valid = inlier_ratio >= self._min_inlier

        # ── /perception/cage_radius ───────────────────────────────────────────
        radius_msg = Float32()
        radius_msg.data = float(radius)
        self._radius_pub.publish(radius_msg)

        # ── /perception/cage_center ───────────────────────────────────────────
        center_msg = PointStamped()
        center_msg.header          = header
        center_msg.point.x         = cx
        center_msg.point.y         = cy
        center_msg.point.z         = 0.0
        self._center_pub.publish(center_msg)

        # ── /perception/circle_valid ──────────────────────────────────────────
        valid_msg = Bool()
        valid_msg.data = is_valid
        self._valid_pub.publish(valid_msg)

        # ── /perception/cage_circle_marker ────────────────────────────────────
        marker = Marker()
        marker.header = header
        marker.ns = "cage_circle"
        marker.id = 0
        if is_valid:
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1 # line width
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            points = []
            for i in range(37):
                angle = i * 2 * math.pi / 36.0
                p = Point()
                p.x = cx + radius * math.cos(angle)
                p.y = cy + radius * math.sin(angle)
                p.z = 0.0
                points.append(p)
            marker.points = points
        else:
            marker.action = Marker.DELETE
            
        self._marker_pub.publish(marker)

        # ── /perception/cage_inliers_marker ───────────────────────────────────
        inliers_marker = Marker()
        inliers_marker.header = header
        inliers_marker.ns = "cage_inliers"
        inliers_marker.id = 1
        if is_valid and inlier_pts is not None:
            inliers_marker.type = Marker.POINTS
            inliers_marker.action = Marker.ADD
            inliers_marker.pose.orientation.w = 1.0
            inliers_marker.scale.x = 0.1 # point width
            inliers_marker.scale.y = 0.1 # point height
            inliers_marker.color.a = 1.0
            inliers_marker.color.r = 0.0
            inliers_marker.color.g = 1.0
            inliers_marker.color.b = 0.0
            
            pts = []
            for i in range(len(inlier_pts)):
                p = Point()
                p.x = float(inlier_pts[i, 0])
                p.y = float(inlier_pts[i, 1])
                p.z = 0.0
                pts.append(p)
            inliers_marker.points = pts
        else:
            inliers_marker.action = Marker.DELETE
            
        self._inliers_marker_pub.publish(inliers_marker)

        if is_valid:
            self._n_valid_pub += 1
            self.get_logger().debug(
                f"[ping360_circle_fitting] ✔ Circle #{self._n_valid_pub} VALID — "
                f"radius={radius:.3f} m  centre=({cx:.3f}, {cy:.3f}) m  "
                f"inliers={n_inliers}/{n_total} ({inlier_ratio:.1%})"
            )
        else:
            self._n_invalid_pub += 1
            self.get_logger().debug(
                f"[ping360_circle_fitting] ✗ Circle INVALID — "
                f"radius={radius:.3f} m  centre=({cx:.3f}, {cy:.3f}) m  "
                f"inlier_ratio={inlier_ratio:.1%} < threshold {self._min_inlier:.1%}  "
                f"(invalid count: {self._n_invalid_pub})"
            )

    def _publish_invalid(self, header) -> None:
        """
        Publish a consistent 'no result' state on all three topics.

        Published values:
          cage_radius  → 0.0
          cage_center  → (0, 0, 0)
          circle_valid → False
        """
        # radius
        radius_msg = Float32()
        radius_msg.data = 0.0
        self._radius_pub.publish(radius_msg)

        # center
        center_msg = PointStamped()
        center_msg.header  = header
        center_msg.point.x = 0.0
        center_msg.point.y = 0.0
        center_msg.point.z = 0.0
        self._center_pub.publish(center_msg)

        # valid flag
        valid_msg = Bool()
        valid_msg.data = False
        self._valid_pub.publish(valid_msg)

        # marker delete
        marker = Marker()
        marker.header = header
        marker.ns = "cage_circle"
        marker.id = 0
        marker.action = Marker.DELETE
        self._marker_pub.publish(marker)

        inliers_marker = Marker()
        inliers_marker.header = header
        inliers_marker.ns = "cage_inliers"
        inliers_marker.id = 1
        inliers_marker.action = Marker.DELETE
        self._inliers_marker_pub.publish(inliers_marker)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ping360CircleFittingNode()
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


if __name__ == "__main__":
    main()
