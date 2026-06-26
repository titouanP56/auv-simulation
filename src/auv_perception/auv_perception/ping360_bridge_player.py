#!/usr/bin/env python3
"""
ping360_bridge_player.py
========================
ROS 2 node — "Bag Reader & Sonar Bridge"

Purpose
-------
Reads a ROS 1 bag file that contains raw Ping360 sonar data (one ray per
message on the topic /sensor/ping360) and republishes the data as a
standard sensor_msgs/msg/LaserScan on /ping360/scan.

This allows the existing ROS 2 perception node (ping360_nearest.py) to
process historical/offline bag recordings as if the sonar were live.

Processing pipeline
-------------------
1. Open the bag with ``rosbags.highlevel.AnyReader`` (no rosbag dependency).
2. Iterate over messages on /sensor/ping360.
3. For each message extract ``msg.angle_deg`` (int, 0–359) and
   ``msg.range`` (float, metres).
4. Place the reading in a 360-slot accumulator: ``ranges[angle_deg] = range``.
5. Detect a full-sweep wrap-around (angle goes from > WRAP_HIGH back to
   < WRAP_LOW between consecutive messages).
6. On wrap-around: build and publish a complete LaserScan.
7. Between messages: sleep proportionally to reproduce real-time playback.

Additionally, a static TF transform (odom → ping360_link) is broadcast
once in __init__ so that the perception node can resolve the sensor frame.

Usage
-----
1. Edit BAG_FILE_PATH below to point to your .bag file.
2. Run the node::

       ros2 run auv_perception ping360_bridge_player

Author  : titou
Package : auv_perception
Topics  : output → /ping360/scan   (sensor_msgs/msg/LaserScan)
          static TF: odom → ping360_link
"""

# ── Standard library ──────────────────────────────────────────────────────────
import math
import time
from pathlib import Path

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node

# ── Messages & TF2 ────────────────────────────────────────────────────────────
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

# ── Bag reader (rosbags library — ROS-distro-agnostic) ────────────────────────
from rosbags.highlevel import AnyReader


# ═════════════════════════════════════════════════════════════════════════════
# ▼▼▼  MODIFY THIS PATH TO POINT TO YOUR ACTUAL BAG FILE  ▼▼▼
# ═════════════════════════════════════════════════════════════════════════════
BAG_FILE_PATH: str = "/home/titou/AUV_project/ros2_AUV/2026-02-05_11-16-19_data.bag"
# ═════════════════════════════════════════════════════════════════════════════


# ── Topic inside the bag that carries the raw Ping360 rays ───────────────────
SONAR_BAG_TOPIC: str = "/sensor/ping360"

# ── Output topic consumed by ping360_nearest.py ──────────────────────────────
OUTPUT_SCAN_TOPIC: str = "/ping360/scan"

# ── Sensor TF frames ─────────────────────────────────────────────────────────
PARENT_FRAME: str = "odom"
CHILD_FRAME: str  = "ping360_link"

# ── Static transform translation (metres) ────────────────────────────────────
TF_TRANSLATION_X: float = 0.0
TF_TRANSLATION_Y: float = 0.08
TF_TRANSLATION_Z: float = 0.10

# ── Wrap-around detection thresholds ─────────────────────────────────────────
# A new sweep is detected when the angle drops from above WRAP_HIGH
# back down to below WRAP_LOW.
WRAP_HIGH: int = 300   # [°] previous angle must be above this …
WRAP_LOW: int  = 50    # [°] … and the current angle must be below this

# ── LaserScan fixed metadata ──────────────────────────────────────────────────
SCAN_FRAME_ID: str     = CHILD_FRAME
SCAN_RANGE_MIN: float  = 0.5    # [m] minimum valid range
SCAN_RANGE_MAX: float  = 10.0   # [m] maximum valid range


class Ping360BridgePlayer(Node):
    """
    ROS 2 node that replays a ROS 1 Ping360 bag as live LaserScan messages.

    The node reads the bag in a tight loop (with real-time pacing via
    time.sleep) and publishes one LaserScan per completed 360° sweep on
    /ping360/scan.  A static TF (odom → ping360_link) is broadcast once
    on startup.
    """

    def __init__(self) -> None:
        super().__init__("ping360_bridge_player")

        # ── Static TF broadcaster ─────────────────────────────────────────────
        self._tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # ── LaserScan publisher ───────────────────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan, OUTPUT_SCAN_TOPIC, 10)

        # ── Internal state ────────────────────────────────────────────────────
        # 360-slot range accumulator; initialised to infinity (no reading)
        self._ranges: list[float] = [math.inf] * 360
        self._prev_angle_deg: int | None = None   # angle of the previous message
        self._n_scans_published: int = 0

        self.get_logger().info(
            f"\n[ping360_bridge_player] Node ready.\n"
            f"  Bag file      : {BAG_FILE_PATH}\n"
            f"  Bag topic     : {SONAR_BAG_TOPIC}\n"
            f"  Output topic  : {OUTPUT_SCAN_TOPIC}\n"
            f"  Static TF     : {PARENT_FRAME} → {CHILD_FRAME}\n"
            f"  Wrap detection: prev > {WRAP_HIGH}° and curr < {WRAP_LOW}°"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Static TF
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_static_tf(self) -> None:
        """
        Publish a static transform from PARENT_FRAME to CHILD_FRAME.

        This makes the sensor position known to the TF tree so that
        ping360_nearest.py can successfully call lookup_transform().
        The quaternion (0, 0, 0, 1) encodes a neutral (zero) rotation.
        """
        t = TransformStamped()

        # Use the current ROS clock time as the stamp
        now = self.get_clock().now().to_msg()
        t.header.stamp    = now
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id  = CHILD_FRAME

        # Translation (sensor position relative to the robot origin)
        t.transform.translation.x = TF_TRANSLATION_X
        t.transform.translation.y = TF_TRANSLATION_Y
        t.transform.translation.z = TF_TRANSLATION_Z

        # Rotation: identity quaternion (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"[ping360_bridge_player] Static TF published: "
            f"{PARENT_FRAME} → {CHILD_FRAME}  "
            f"T=({TF_TRANSLATION_X}, {TF_TRANSLATION_Y}, {TF_TRANSLATION_Z})"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Main playback loop
    # ──────────────────────────────────────────────────────────────────────────

    def play(self) -> None:
        """
        Open the bag file and replay the Ping360 data in real time.

        This method blocks until the bag is fully consumed or the ROS context
        is shut down.  It is meant to be called once after node creation.

        Real-time pacing
        ~~~~~~~~~~~~~~~~
        The timestamp of each message is extracted from the bag.  The wall-clock
        sleep between two consecutive messages equals the original inter-message
        interval (timestamp difference), clamped to [0, 1] second to prevent
        stalls on gaps or clock jumps in the recording.
        """
        bag_path = Path(BAG_FILE_PATH)
        if not bag_path.exists():
            self.get_logger().error(
                f"[ping360_bridge_player] Bag file not found: {BAG_FILE_PATH}\n"
                "Please update the BAG_FILE_PATH variable at the top of the script."
            )
            return

        self.get_logger().info(
            f"[ping360_bridge_player] Opening bag: {BAG_FILE_PATH}"
        )

        prev_bag_timestamp_ns: int | None = None   # nanosecond timestamp of previous msg

        with AnyReader([bag_path]) as reader:
            # Filter connections to the Ping360 raw topic
            connections = [
                c for c in reader.connections
                if c.topic == SONAR_BAG_TOPIC
            ]

            if not connections:
                self.get_logger().error(
                    f"[ping360_bridge_player] Topic '{SONAR_BAG_TOPIC}' not found "
                    f"in bag. Available topics: "
                    f"{sorted(set(c.topic for c in reader.connections))}"
                )
                return

            self.get_logger().info(
                f"[ping360_bridge_player] Found {len(connections)} connection(s) "
                f"on '{SONAR_BAG_TOPIC}'. Starting playback …"
            )

            # Iterate over all messages on the filtered connections
            for connection, bag_timestamp_ns, raw_data in reader.messages(
                connections=connections
            ):
                # Check ROS shutdown between messages
                if not rclpy.ok():
                    self.get_logger().info(
                        "[ping360_bridge_player] ROS shutdown requested — stopping."
                    )
                    break

                # ── Deserialise the raw message ───────────────────────────────
                msg = reader.deserialize(raw_data, connection.msgtype)

                # ── Extract sonar data ────────────────────────────────────────
                angle_deg: int   = int(msg.angle_deg)    # 0 … 359
                range_m: float   = float(msg.range)      # metres

                # ── Real-time pacing ──────────────────────────────────────────
                if prev_bag_timestamp_ns is not None:
                    delta_ns  = bag_timestamp_ns - prev_bag_timestamp_ns
                    delta_sec = delta_ns * 1e-9
                    # Clamp to avoid sleeping too long on recording gaps
                    sleep_sec = max(0.0, min(delta_sec, 1.0))
                    if sleep_sec > 0.0:
                        time.sleep(sleep_sec)

                prev_bag_timestamp_ns = bag_timestamp_ns

                # ── Accumulate ray in the 360° buffer ─────────────────────────
                if 0 <= angle_deg <= 359:
                    self._ranges[angle_deg] = range_m
                else:
                    self.get_logger().warn(
                        f"[ping360_bridge_player] Unexpected angle value: "
                        f"{angle_deg}° — skipping."
                    )
                    continue

                # A new sweep begins when the angle crosses the 0/360 boundary
                # either forward (>300 to <50) or backward (<50 to >300).
                if self._prev_angle_deg is not None:
                    forward_wrap = (self._prev_angle_deg > WRAP_HIGH and angle_deg < WRAP_LOW)
                    reverse_wrap = (self._prev_angle_deg < WRAP_LOW and angle_deg > WRAP_HIGH)
                    
                    if forward_wrap or reverse_wrap:
                        direction = "forward" if forward_wrap else "reverse"
                        self.get_logger().debug(
                            f"[ping360_bridge_player] Wrap-around detected ({direction}): "
                            f"{self._prev_angle_deg}° → {angle_deg}° — "
                            "publishing LaserScan."
                        )
                        self._publish_scan(bag_timestamp_ns)

                self._prev_angle_deg = angle_deg

        self.get_logger().info(
            f"[ping360_bridge_player] Bag playback complete. "
            f"Published {self._n_scans_published} LaserScan(s)."
        )

    # ──────────────────────────────────────────────────────────────────────────
    # LaserScan builder & publisher
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_scan(self, bag_timestamp_ns: int) -> None:
        """
        Build a LaserScan message from the current 360-slot accumulator and
        publish it on OUTPUT_SCAN_TOPIC.

        After publishing, the accumulator is reset to ``math.inf`` so the next
        sweep starts clean.

        Args:
            bag_timestamp_ns: Bag timestamp of the wrap-around message, in
                              nanoseconds.  Used as the scan stamp so that the
                              LaserScan header matches the original recording
                              time.
        """
        scan = LaserScan()

        # ── Header ───────────────────────────────────────────────────────────
        scan.header.frame_id = SCAN_FRAME_ID

        # Convert nanosecond bag timestamp to ROS builtin_interfaces/Time
        stamp = RosTime()
        stamp.sec     = int(bag_timestamp_ns // 1_000_000_000)
        stamp.nanosec = int(bag_timestamp_ns  % 1_000_000_000)
        scan.header.stamp = stamp

        # ── Angular geometry (full circle, 1° increments) ─────────────────
        scan.angle_min       = 0.0               # [rad]
        scan.angle_max       = 2.0 * math.pi     # [rad]
        scan.angle_increment = math.pi / 180.0   # [rad] = 1° per slot

        # scan_time: approximate time for one full rotation.
        # We have 360 rays; leave it at 0.0 if unknown.
        scan.scan_time      = 0.0
        scan.time_increment = 0.0

        # ── Range validity bounds ─────────────────────────────────────────
        scan.range_min = SCAN_RANGE_MIN
        scan.range_max = SCAN_RANGE_MAX

        # ── Ranges: copy the accumulator and reset it ──────────────────────
        scan.ranges = list(self._ranges)         # shallow copy of current sweep
        self._ranges = [math.inf] * 360          # reset for the next sweep

        # ── Intensities: not used — leave empty ───────────────────────────
        scan.intensities = []

        # ── Publish ───────────────────────────────────────────────────────
        self._scan_pub.publish(scan)
        self._n_scans_published += 1

        self.get_logger().info(
            f"[ping360_bridge_player] ✔ LaserScan #{self._n_scans_published} published — "
            f"stamp={stamp.sec}.{stamp.nanosec:09d}  "
            f"finite_rays={sum(math.isfinite(r) for r in scan.ranges)}/360"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Initialise ROS 2, create the node, run the bag playback, then shut down."""
    rclpy.init(args=args)
    node = Ping360BridgePlayer()

    try:
        # play() is blocking: it reads the entire bag sequentially.
        node.play()
    except KeyboardInterrupt:
        node.get_logger().info(
            "[ping360_bridge_player] Interrupted by user (Ctrl-C)."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
