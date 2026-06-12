#!/usr/bin/env python3
"""
bag_to_ply.py
=============
Multi-sensor fusion pipeline: Ping360 sonar + IMU attitude + depth sensor.
Reads a ROS 1 (.bag) file, synchronises the three asynchronous topics via
nearest-neighbour timestamp matching, applies a full 6-DoF rigid-body
transformation, and exports a filtered ASCII PLY point cloud.

Pipeline
--------
  Pass 1 – Cache:  Read /sensor/attitude and /sensor/depth_temperature into
                   sorted timestamp lists (RAM-friendly; messages only).
  Pass 2 – Sonar:  For each /sensor/ping360 measurement, look up the closest
                   attitude and depth sample using bisect (O(log n)).
             → Range-gate filter  (RANGE_MIN … RANGE_MAX)
             → Consecutive-angle median de-noising
             → Polar → local Cartesian
             → Rotation via scipy (roll, pitch, yaw)
             → Translation (sensor offsets + depth)
  Post-processing: Radius Outlier Removal (density filter) before PLY export.

Usage
-----
    python3 bag_to_ply.py          # edit BAG_PATH / OUTPUT_PLY below
    ros2 run auv_perception bag_to_ply
"""

import bisect
import math
import statistics
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from rosbags.highlevel import AnyReader

# =============================================================================
# USER CONFIGURATION  ← edit these variables to match your setup
# =============================================================================
BAG_PATH   = Path("/home/titou/AUV_project/ros2_AUV/2026-02-05_11-08-20_data.bag")
OUTPUT_PLY = Path("/home/titou/AUV_project/ros2_AUV/src/auv_perception/output_bassin_11-08-20.ply")

TOPIC_SONAR = "/sensor/ping360"
TOPIC_ATT   = "/sensor/attitude"
TOPIC_DEPTH = "/sensor/depth_temperature"

# Hard range gate for the Ping360 (metres)
RANGE_MIN = 0.5
RANGE_MAX = 3.5

# Static sensor-to-robot-centre offsets (metres)
OFFSET_Y = 0.08   # lateral offset
OFFSET_Z = 0.10   # vertical mounting height above the reference frame

# Radius Outlier Removal: keep a point only if it has at least MIN_NEIGHBORS
# neighbours within SEARCH_RADIUS metres.
SEARCH_RADIUS  = 0.15   # metres
MIN_NEIGHBORS  = 3
# =============================================================================


# ---------------------------------------------------------------------------
# Pass 1 – Build timestamp-indexed caches for auxiliary sensors
# ---------------------------------------------------------------------------

def build_sensor_caches(bag_path: Path) -> tuple[list, list]:
    """
    First pass through the bag: read attitude and depth messages into two
    sorted lists of (timestamp_ns, ...) tuples.

    Keeping only the scalar data (no raw bytes, no ROS objects) is intentional
    to minimise RAM usage when bags are large.

    Returns
    -------
    att_cache   : list of (ts_ns: int, roll: float, pitch: float, yaw: float)
    depth_cache : list of (ts_ns: int, depth: float)
    Both lists are sorted by ts_ns (guaranteed by bag chronological order).
    """
    att_cache   = []  # (timestamp_ns, roll, pitch, yaw)
    depth_cache = []  # (timestamp_ns, depth)

    topics_needed = {TOPIC_ATT, TOPIC_DEPTH}

    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic in topics_needed]
        if not connections:
            print(f"[bag_to_ply] WARNING: none of {topics_needed} found in bag.")
            return att_cache, depth_cache

        found_topics = {c.topic for c in connections}
        print(f"[bag_to_ply] Pass 1 – caching topics: {found_topics}")

        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == TOPIC_ATT:
                att_cache.append((
                    int(timestamp_ns),
                    float(msg.roll),
                    float(msg.pitch),
                    float(msg.yaw),
                ))
            elif connection.topic == TOPIC_DEPTH:
                depth_cache.append((
                    int(timestamp_ns),
                    float(msg.depth),
                ))

    print(f"[bag_to_ply] Cached {len(att_cache)} attitude samples, "
          f"{len(depth_cache)} depth samples.")
    return att_cache, depth_cache


# ---------------------------------------------------------------------------
# Nearest-neighbour lookup helper
# ---------------------------------------------------------------------------

def nearest_sample(cache: list, query_ts: int) -> tuple | None:
    """
    Return the tuple from *cache* whose first element (timestamp_ns) is
    closest to *query_ts*, using bisect for O(log n) lookup.

    Parameters
    ----------
    cache    : sorted list of (ts_ns, *values)
    query_ts : query timestamp in nanoseconds

    Returns
    -------
    The closest tuple, or None if the cache is empty.
    """
    if not cache:
        return None

    # Extract timestamps once per call; acceptable for moderate cache sizes.
    # For very large caches (>100k), pre-build a separate ts array once.
    timestamps = [entry[0] for entry in cache]

    idx = bisect.bisect_left(timestamps, query_ts)

    # Edge cases
    if idx == 0:
        return cache[0]
    if idx >= len(cache):
        return cache[-1]

    # Pick the nearer of the two surrounding samples
    before = cache[idx - 1]
    after  = cache[idx]
    if (query_ts - before[0]) <= (after[0] - query_ts):
        return before
    return after


# ---------------------------------------------------------------------------
# Pass 2 – Sonar reading with consecutive-angle median filter
# ---------------------------------------------------------------------------

def read_and_filter_sonar(bag_path: Path):
    """
    Second pass through the bag: read /sensor/ping360 messages, apply range
    gating and consecutive-angle median de-noising.

    Yields
    ------
    (timestamp_ns, angle_deg, range_m) : tuple[int, float, float]
    """
    current_angle  = None
    current_group  = []  # list of (timestamp_ns, range_m)

    def flush(angle, group):
        """Yield the median-range entry from the current group."""
        if not group:
            return
        # Keep the median range; use the middle timestamp as representative
        group_sorted = sorted(group, key=lambda e: e[1])
        mid_idx      = len(group_sorted) // 2
        yield (group_sorted[mid_idx][0], angle, group_sorted[mid_idx][1])

    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic == TOPIC_SONAR]
        if not connections:
            raise RuntimeError(
                f"Topic '{TOPIC_SONAR}' not found in bag. "
                f"Available: {[c.topic for c in reader.connections]}"
            )

        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            msg       = reader.deserialize(rawdata, connection.msgtype)
            angle_deg = float(msg.angle_deg)
            range_m   = float(msg.range)

            # Hard range gate
            if range_m < RANGE_MIN or range_m > RANGE_MAX:
                continue

            if angle_deg == current_angle:
                current_group.append((int(timestamp_ns), range_m))
            else:
                yield from flush(current_angle, current_group)
                current_angle = angle_deg
                current_group = [(int(timestamp_ns), range_m)]

    yield from flush(current_angle, current_group)


# ---------------------------------------------------------------------------
# 3-D projection with full rotation + translation
# ---------------------------------------------------------------------------

def project_point(
    angle_deg: float,
    range_m:   float,
    roll:      float,
    pitch:     float,
    yaw:       float,
    depth:     float,
) -> tuple[float, float, float]:
    """
    Convert a Ping360 polar measurement to world-frame 3-D coordinates.

    Steps
    -----
    1. Local Cartesian in the sonar frame (flat plane, Z = 0).
    2. Apply the robot attitude rotation (scipy Rotation, 'xyz' convention).
    3. Apply the static sensor-to-robot offset and global depth translation.

    Parameters
    ----------
    angle_deg   : beam angle in degrees
    range_m     : measured slant range in metres
    roll, pitch : attitude angles in radians
    yaw         : heading in radians
    depth       : water depth of the robot (positive downward), in metres

    Returns
    -------
    (X_final, Y_final, Z_final) in the world frame
    """
    angle_rad = math.radians(angle_deg)

    # Step 1 – local sonar-frame Cartesian (Z=0: sonar sweeps horizontally)
    x_loc = range_m * math.cos(angle_rad)
    y_loc = range_m * math.sin(angle_rad)
    z_loc = 0.0

    # Step 2 – rotate by current robot attitude
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    x_rot, y_rot, z_rot = rot.apply([x_loc, y_loc, z_loc])

    # Step 3 – translate: sensor offsets + vertical depth compensation
    x_final = x_rot
    y_final = y_rot + OFFSET_Y
    z_final = z_rot + OFFSET_Z - depth

    return x_final, y_final, z_final


# ---------------------------------------------------------------------------
# Post-processing: Radius Outlier Removal
# ---------------------------------------------------------------------------

def radius_outlier_removal(
    points: list,
    search_radius: float = SEARCH_RADIUS,
    min_neighbors: int   = MIN_NEIGHBORS,
) -> list:
    """
    Remove isolated points that have fewer than *min_neighbors* neighbours
    within *search_radius* metres.

    Uses a vectorised numpy pairwise-distance approach, which is efficient
    for clouds up to ~10 000 points. For larger clouds, a KD-tree should be
    substituted.

    Parameters
    ----------
    points        : list of (x, y, z) tuples
    search_radius : neighbourhood radius in metres
    min_neighbors : minimum number of neighbours required to keep a point

    Returns
    -------
    Filtered list of (x, y, z) tuples.
    """
    if len(points) < min_neighbors + 1:
        return points  # too few points to filter meaningfully

    pts = np.array(points, dtype=np.float32)   # shape (N, 3)

    # Pairwise squared distances (memory: O(N²); fine for N < 10k)
    diff = pts[:, np.newaxis, :] - pts[np.newaxis, :, :]   # (N, N, 3)
    sq_dist = np.sum(diff ** 2, axis=2)                    # (N, N)

    # Count neighbours within search_radius (exclude self: diag = 0)
    neighbor_count = np.sum(sq_dist < search_radius ** 2, axis=1) - 1  # (N,)

    mask = neighbor_count >= min_neighbors
    filtered = [pt for pt, keep in zip(points, mask) if keep]
    print(f"[bag_to_ply] ROR: {len(points)} → {len(filtered)} points "
          f"(removed {len(points) - len(filtered)} outliers)")
    return filtered


# ---------------------------------------------------------------------------
# PLY export (pure-text, no open3d dependency)
# ---------------------------------------------------------------------------

def write_ply(points: list, output_path: Path) -> None:
    """
    Write a list of (X, Y, Z) tuples as an ASCII PLY point cloud.

    Parameters
    ----------
    points      : list of (float, float, float)
    output_path : destination .ply file path
    """
    num_vertices = len(points)

    with open(output_path, "w", encoding="utf-8") as f:
        # PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment Generated by bag_to_ply.py (auv_perception) – multi-sensor fusion\n")
        f.write(f"element vertex {num_vertices}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        # Vertex data
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

    print(f"[bag_to_ply] Wrote {num_vertices} points → {output_path}")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """Orchestrate the full multi-sensor extraction → fusion → export pipeline."""
    print(f"[bag_to_ply] Bag       : {BAG_PATH}")
    print(f"[bag_to_ply] Output    : {OUTPUT_PLY}")
    print(f"[bag_to_ply] Range gate: [{RANGE_MIN}, {RANGE_MAX}] m")

    # ── Pass 1: cache auxiliary sensor data ──────────────────────────────────
    att_cache, depth_cache = build_sensor_caches(BAG_PATH)

    att_available   = len(att_cache)   > 0
    depth_available = len(depth_cache) > 0

    if not att_available:
        print("[bag_to_ply] WARNING: no attitude data – will use roll=0, pitch=0, yaw=0.")
    if not depth_available:
        print("[bag_to_ply] WARNING: no depth data – will use depth=0.")

    # Pre-extract timestamp arrays once (avoids rebuilding inside the loop)
    att_ts   = [e[0] for e in att_cache]   if att_available   else []
    depth_ts = [e[0] for e in depth_cache] if depth_available else []

    # ── Pass 2: process sonar with synchronised attitude and depth ───────────
    print(f"[bag_to_ply] Pass 2 – processing sonar topic: {TOPIC_SONAR}")

    points = []
    n_sonar_raw = 0

    for ts_ns, angle_deg, range_m in read_and_filter_sonar(BAG_PATH):
        n_sonar_raw += 1

        # Nearest-neighbour lookup for attitude
        if att_available:
            idx_att = bisect.bisect_left(att_ts, ts_ns)
            if idx_att == 0:
                att_entry = att_cache[0]
            elif idx_att >= len(att_cache):
                att_entry = att_cache[-1]
            else:
                before = att_cache[idx_att - 1]
                after  = att_cache[idx_att]
                att_entry = before if (ts_ns - before[0]) <= (after[0] - ts_ns) else after
            _, roll, pitch, yaw = att_entry
        else:
            roll = pitch = yaw = 0.0

        # Nearest-neighbour lookup for depth
        if depth_available:
            idx_dep = bisect.bisect_left(depth_ts, ts_ns)
            if idx_dep == 0:
                dep_entry = depth_cache[0]
            elif idx_dep >= len(depth_cache):
                dep_entry = depth_cache[-1]
            else:
                before = depth_cache[idx_dep - 1]
                after  = depth_cache[idx_dep]
                dep_entry = before if (ts_ns - before[0]) <= (after[0] - ts_ns) else after
            _, depth = dep_entry
        else:
            depth = 0.0

        # 3-D projection
        x, y, z = project_point(angle_deg, range_m, roll, pitch, yaw, depth)
        points.append((x, y, z))

    print(f"[bag_to_ply] {n_sonar_raw} sonar measurements after range gating + median filter")
    print(f"[bag_to_ply] {len(points)} points projected to 3-D world frame")

    if not points:
        print("[bag_to_ply] WARNING: no valid points – PLY will be empty.")
        write_ply([], OUTPUT_PLY)
        return

    # ── Post-processing: density filter ─────────────────────────────────────
    points = radius_outlier_removal(points)

    # ── Export ───────────────────────────────────────────────────────────────
    write_ply(points, OUTPUT_PLY)


if __name__ == "__main__":
    main()
