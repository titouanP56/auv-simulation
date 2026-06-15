#!/usr/bin/env python3
"""
sonoptix_perception.py
======================
ROS 2 perception node for the Sonoptix multi-beam sonar.

Processing pipeline
-------------------
  1. Subscribe to /sonoptix/points (PointCloud2, Best-Effort QoS).
  2. Vectorised NumPy decoding of the binary buffer → (N, 3) float32 array.
  3. Spatial culling: discard points outside [min_range_m, max_range_m]
     and any NaN / inf values.
  4. Robust 3D plane extraction via RANSAC:
       - Primary backend  : Open3D  segment_plane()  (C++ backend, real-time capable)
       - Fallback backend : sklearn RANSACRegressor   (C backend, if Open3D not found)
     The fitted plane (aX + bY + cZ + d = 0) treats fish echoes as outliers
     by design, thanks to the inherent robustness of RANSAC.
  5. Geometry extraction from the normalised normal vector n̂ = [a, b, c]:
       - Orthogonal distance = |d|             (Open3D normalises the normal to 1)
       - yaw_normal          = atan2(ny, nx)   (horizontal orientation of the normal)
       - pitch_normal        = arcsin(nz)      (vertical tilt of the normal)
  6. Encode the normal as a ZYX quaternion (roll=0, pitch, yaw) so that
     euler_from_quaternion in the guidance nodes returns (_, pitch_normal, yaw_normal).
  7. Publish:
       /sonoptix/perception       → geometry_msgs/PoseStamped
         pose.position.x  = orthogonal distance to the net plane [m]
         pose.orientation = normal quaternion (Yaw + Pitch)
       /sonoptix/perception_valid → std_msgs/Bool
         True  if RANSAC converged with an inlier ratio ≥ min_inlier_ratio
         False otherwise (empty cloud, RANSAC diverged, too few points…)

ROS 2 Topics
------------
  Input : /sonoptix/points            (sensor_msgs/PointCloud2)
  Outputs: /sonoptix/perception       (geometry_msgs/PoseStamped)
           /sonoptix/perception_valid  (std_msgs/Bool)

Configurable Parameters
-----------------------
  min_range_m              : minimum valid range [m]             (default: 0.3)
  max_range_m              : maximum valid range [m]             (default: 7.0)
  ransac_distance_threshold: RANSAC inlier distance threshold [m](default: 0.05)
  ransac_n                 : min points for a RANSAC hypothesis  (default: 3)
  min_inlier_ratio         : minimum inlier/total ratio to pass  (default: 0.30)
  min_points               : min points to attempt RANSAC        (default: 10)

Author  : titou
Package : auv_perception
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


# ── Backend detection ──────────────────────────────────────────────────────────

try:
    import open3d as o3d
    _HAVE_OPEN3D = True
except ImportError:
    _HAVE_OPEN3D = False

_HAVE_SKLEARN = False
if not _HAVE_OPEN3D:
    try:
        from sklearn.linear_model import RANSACRegressor
        from sklearn.linear_model import LinearRegression
        _HAVE_SKLEARN = True
    except ImportError:
        pass


# ── Default constants ──────────────────────────────────────────────────────────

_DEFAULT_MIN_RANGE_M             = 0.3
_DEFAULT_MAX_RANGE_M             = 7.0
_DEFAULT_RANSAC_DIST_THRESHOLD   = 0.05   # [m]
_DEFAULT_RANSAC_N                = 3
_DEFAULT_MIN_INLIER_RATIO        = 0.30
_DEFAULT_MIN_POINTS              = 10


# ── Helper functions ───────────────────────────────────────────────────────────

def _decode_pointcloud2(msg: PointCloud2) -> np.ndarray | None:
    """
    Decode a PointCloud2 message into a NumPy (N, 3) float32 array.

    Uses the field offsets declared in the message header to be robust against
    any point layout. Assumes x, y, z coordinates encoded as float32 (4 bytes
    each).

    Returns None if the cloud is empty or if the required fields are missing.
    """
    n_pts = msg.width * msg.height
    if n_pts == 0:
        return None

    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return None

    # Float index of each field within the point stride (point_step in bytes)
    floats_per_point = msg.point_step // 4
    x_idx = field_map['x'].offset // 4
    y_idx = field_map['y'].offset // 4
    z_idx = field_map['z'].offset // 4

    # Vectorised decode: reshape directly to (N, floats_per_point)
    payload = np.frombuffer(msg.data, dtype=np.float32)
    if len(payload) < n_pts * floats_per_point:
        return None

    points_raw = payload[:n_pts * floats_per_point].reshape(n_pts, floats_per_point)

    pts = np.stack(
        [points_raw[:, x_idx],
         points_raw[:, y_idx],
         points_raw[:, z_idx]],
        axis=1,
    )
    return pts


def _cull_points(pts: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
    """
    Apply spatial culling:
      - Remove NaN / inf values.
      - Keep only points within [min_range, max_range] metres.

    Returns the filtered array (M, 3). M may be 0.
    """
    finite_mask = np.isfinite(pts).all(axis=1)
    pts = pts[finite_mask]
    if len(pts) == 0:
        return pts

    dist2 = pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2
    range_mask = (dist2 >= min_range ** 2) & (dist2 <= max_range ** 2)
    return pts[range_mask]


def _orient_normal_toward_origin(normal: np.ndarray) -> np.ndarray:
    """
    Ensure the plane normal points toward the AUV (≈ toward the sensor frame
    origin). If the X component of the normal (robot forward axis) is negative,
    flip the vector.
    """
    if normal[0] < 0.0:
        return -normal
    return normal


def _normal_to_quaternion(normal: np.ndarray) -> tuple[float, float, float, float]:
    """
    Encode the plane normal as a ZYX quaternion (roll=0, pitch, yaw).

    Convention:
      yaw_normal   = atan2(ny, nx)   — horizontal angle of the normal
      pitch_normal = arcsin(nz)      — vertical tilt of the normal

    The resulting quaternion is such that euler_from_quaternion([qx, qy, qz, qw])
    in the guidance nodes returns (≈0, pitch_normal, yaw_normal).

    Returns (qx, qy, qz, qw).
    """
    nx, ny, nz = float(normal[0]), float(normal[1]), float(normal[2])

    # Clamp nz to avoid arcsin domain errors
    nz = max(-1.0, min(1.0, nz))

    yaw   = math.atan2(ny, nx)
    pitch = math.asin(nz)

    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    # roll = 0 → cr = 1, sr = 0
    qw = cy * cp
    qx = -sy * sp
    qy = cy * sp
    qz = sy * cp

    return qx, qy, qz, qw


def _ransac_open3d(
    pts: np.ndarray,
    distance_threshold: float,
    ransac_n: int,
    num_iterations: int = 100,
) -> tuple[np.ndarray, float, float] | None:
    """
    Plane fitting by RANSAC using Open3D (C++ backend).

    Returns (normalised_normal, orthogonal_distance, inlier_ratio) or None.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

    try:
        plane_model, inlier_indices = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
        )
    except Exception:
        return None

    a, b, c, d = plane_model
    normal = np.array([a, b, c], dtype=np.float64)
    norm_mag = np.linalg.norm(normal)
    if norm_mag < 1e-9:
        return None

    # Open3D already normalises coefficients (a²+b²+c²=1), but re-normalise
    # for safety.
    normal /= norm_mag
    distance = abs(d / norm_mag)
    inlier_ratio = len(inlier_indices) / len(pts)

    return normal, distance, inlier_ratio


def _ransac_sklearn(
    pts: np.ndarray,
    distance_threshold: float,
) -> tuple[np.ndarray, float, float] | None:
    """
    Plane fitting by RANSAC using sklearn RANSACRegressor (fallback backend).

    Model: z = a*x + b*y + c  →  plane ax + by - z + c = 0
    Normal = [a, b, -1] (normalised).

    Returns (normalised_normal, orthogonal_distance, inlier_ratio) or None.
    """
    X = pts[:, :2]   # (x, y)
    y = pts[:, 2]    # z

    try:
        ransac = RANSACRegressor(
            estimator=LinearRegression(),
            residual_threshold=distance_threshold,
            min_samples=3,
            max_trials=200,
            random_state=42,
        )
        ransac.fit(X, y)
    except Exception:
        return None

    a, b = ransac.estimator_.coef_
    c_intercept = ransac.estimator_.intercept_
    # Plane: a*x + b*y - z + c_intercept = 0
    normal = np.array([a, b, -1.0], dtype=np.float64)
    norm_mag = np.linalg.norm(normal)
    if norm_mag < 1e-9:
        return None
    normal /= norm_mag

    # Distance from origin to the plane
    distance = abs(c_intercept) / norm_mag

    inlier_ratio = float(ransac.inlier_mask_.sum()) / len(pts)
    return normal, distance, inlier_ratio


# ── Main node ──────────────────────────────────────────────────────────────────

class SonoptixPerceptionNode(Node):
    """
    ROS 2 Sonoptix perception node.

    Receives the raw point cloud, extracts the net plane via 3D RANSAC, and
    publishes the orthogonal distance and orientation (quaternion) of the plane
    as a geometry_msgs/PoseStamped.
    """

    def __init__(self) -> None:
        super().__init__('sonoptix_perception')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('min_range_m',              _DEFAULT_MIN_RANGE_M)
        self.declare_parameter('max_range_m',              _DEFAULT_MAX_RANGE_M)
        self.declare_parameter('ransac_distance_threshold', _DEFAULT_RANSAC_DIST_THRESHOLD)
        self.declare_parameter('ransac_n',                 _DEFAULT_RANSAC_N)
        self.declare_parameter('min_inlier_ratio',         _DEFAULT_MIN_INLIER_RATIO)
        self.declare_parameter('min_points',               _DEFAULT_MIN_POINTS)

        self._min_range   = float(self.get_parameter('min_range_m').value)
        self._max_range   = float(self.get_parameter('max_range_m').value)
        self._ransac_dist = float(self.get_parameter('ransac_distance_threshold').value)
        self._ransac_n    = int(self.get_parameter('ransac_n').value)
        self._min_inlier  = float(self.get_parameter('min_inlier_ratio').value)
        self._min_points  = int(self.get_parameter('min_points').value)

        # ── RANSAC backend selection ───────────────────────────────────────────
        if _HAVE_OPEN3D:
            self._backend = 'open3d'
        elif _HAVE_SKLEARN:
            self._backend = 'sklearn'
        else:
            self._backend = 'none'
            self.get_logger().error(
                '[SonoptixPerception] Neither Open3D nor sklearn is available! '
                'Install at least one: pip install open3d  or  pip install scikit-learn'
            )

        # ── QoS ───────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscription ──────────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2,
            '/sonoptix/points',
            self._pointcloud_cb,
            best_effort_qos,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._perception_pub = self.create_publisher(
            PoseStamped, '/sonoptix/perception', 10
        )
        self._valid_pub = self.create_publisher(
            Bool, '/sonoptix/perception_valid', 10
        )

        # ── Diagnostic counters ───────────────────────────────────────────────
        self._n_received  = 0
        self._n_published = 0
        self._n_failures  = 0

        self.get_logger().info(
            f'\n[SonoptixPerception] Node started\n'
            f'  RANSAC backend         : {self._backend}\n'
            f'  Range filter           : [{self._min_range:.2f} m — {self._max_range:.2f} m]\n'
            f'  RANSAC inlier threshold: {self._ransac_dist:.3f} m\n'
            f'  Min inlier ratio       : {self._min_inlier:.0%}\n'
            f'  Min points for RANSAC  : {self._min_points}\n'
            f'  Input topic            : /sonoptix/points\n'
            f'  Output topics          : /sonoptix/perception  +  /sonoptix/perception_valid'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Main callback
    # ──────────────────────────────────────────────────────────────────────────

    def _pointcloud_cb(self, msg: PointCloud2) -> None:
        """Process a Sonoptix point cloud and publish the perception result."""
        self._n_received += 1

        # ── 1. Decode ──────────────────────────────────────────────────────────
        pts = _decode_pointcloud2(msg)
        if pts is None:
            self._publish_invalid()
            return

        # ── 2. Spatial culling ─────────────────────────────────────────────────
        pts = _cull_points(pts, self._min_range, self._max_range)
        if len(pts) < self._min_points:
            if self._n_received % 20 == 1:
                self.get_logger().debug(
                    f'[SonoptixPerception] Cloud too small after culling: '
                    f'{len(pts)} pts (min={self._min_points})'
                )
            self._publish_invalid()
            return

        # ── 3. 3D RANSAC ───────────────────────────────────────────────────────
        result = self._run_ransac(pts)
        if result is None:
            self._n_failures += 1
            if self._n_failures % 10 == 1:
                self.get_logger().warn(
                    f'[SonoptixPerception] RANSAC failure #{self._n_failures} '
                    f'(backend={self._backend}, pts={len(pts)})'
                )
            self._publish_invalid()
            return

        normal, distance, inlier_ratio = result

        # ── 4. Inlier ratio validation ─────────────────────────────────────────
        if inlier_ratio < self._min_inlier:
            if self._n_received % 20 == 1:
                self.get_logger().debug(
                    f'[SonoptixPerception] Inlier ratio too low: '
                    f'{inlier_ratio:.0%} < {self._min_inlier:.0%}'
                )
            self._publish_invalid()
            return

        # ── 5. Orient the normal toward the AUV ───────────────────────────────
        normal = _orient_normal_toward_origin(normal)

        # ── 6. Encode as quaternion ────────────────────────────────────────────
        qx, qy, qz, qw = _normal_to_quaternion(normal)

        # ── 7. Publish ────────────────────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = msg.header.stamp
        pose_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'sonoptix_link'
        pose_msg.pose.position.x = distance
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self._perception_pub.publish(pose_msg)

        valid_msg = Bool()
        valid_msg.data = True
        self._valid_pub.publish(valid_msg)

        self._n_published += 1
        self._n_failures = 0   # reset consecutive failure counter

        if self._n_published % 50 == 1:
            nx, ny, nz = normal
            self.get_logger().debug(
                f'[SonoptixPerception] #{self._n_published} ✔ '
                f'dist={distance:.3f} m  '
                f'n=({nx:.2f}, {ny:.2f}, {nz:.2f})  '
                f'inliers={inlier_ratio:.0%}  '
                f'pts={len(pts)}'
            )

    # ──────────────────────────────────────────────────────────────────────────
    # RANSAC backend dispatcher
    # ──────────────────────────────────────────────────────────────────────────

    def _run_ransac(
        self, pts: np.ndarray
    ) -> tuple[np.ndarray, float, float] | None:
        """Dispatch to the available RANSAC backend."""
        if self._backend == 'open3d':
            return _ransac_open3d(pts, self._ransac_dist, self._ransac_n)
        elif self._backend == 'sklearn':
            return _ransac_sklearn(pts, self._ransac_dist)
        else:
            return None

    # ──────────────────────────────────────────────────────────────────────────
    # Invalid result publisher
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_invalid(self) -> None:
        """Publish False on /sonoptix/perception_valid without touching the Pose topic."""
        valid_msg = Bool()
        valid_msg.data = False
        self._valid_pub.publish(valid_msg)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SonoptixPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[SonoptixPerception] Keyboard interrupt — shutting down.')
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
