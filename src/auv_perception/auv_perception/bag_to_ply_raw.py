#!/usr/bin/env python3
"""
bag_to_ply_raw.py
=================
Minimal script: reads raw Ping360 data from a ROS 1 bag and exports
every point directly to an ASCII PLY file, with NO filtering, NO
rotation, NO outlier removal.

Each measurement is converted from polar (range, angle) to Cartesian
(X, Y) coordinates only. Z is set to 0.

Usage
-----
    python3 bag_to_ply_raw.py
"""

import math
from pathlib import Path

from rosbags.highlevel import AnyReader

# =============================================================================
# USER CONFIGURATION  ← edit these two lines
# =============================================================================
BAG_PATH   = Path("/home/titou/AUV_project/ros2_AUV/2026-02-05_11-08-20_data.bag")
OUTPUT_PLY = Path("/home/titou/AUV_project/ros2_AUV/src/auv_perception/output_raw_11-08-20.ply")

TOPIC_SONAR = "/sensor/ping360"
# =============================================================================


def main():
    print(f"[raw] Bag    : {BAG_PATH}")
    print(f"[raw] Output : {OUTPUT_PLY}")

    points = []

    with AnyReader([BAG_PATH]) as reader:
        connections = [c for c in reader.connections if c.topic == TOPIC_SONAR]
        if not connections:
            raise RuntimeError(
                f"Topic '{TOPIC_SONAR}' not found. "
                f"Available: {[c.topic for c in reader.connections]}"
            )

        for connection, _ts, rawdata in reader.messages(connections=connections):
            msg       = reader.deserialize(rawdata, connection.msgtype)
            angle_rad = math.radians(float(msg.angle_deg))
            range_m   = float(msg.range)

            x = range_m * math.cos(angle_rad)
            y = range_m * math.sin(angle_rad)
            z = 0.0

            points.append((x, y, z))

    print(f"[raw] {len(points)} raw points read")

    # Write PLY
    with open(OUTPUT_PLY, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment Raw Ping360 export – no filtering\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

    print(f"[raw] Wrote {len(points)} points → {OUTPUT_PLY}")


if __name__ == "__main__":
    main()
