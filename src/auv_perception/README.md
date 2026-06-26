# auv_perception

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This package is the **sensing layer** of the AUV stack. It processes raw sonar data from two sensors and publishes clean, actionable results to the guidance nodes.

The **primary sensor** for net inspection is the **Sonoptix ECHO 2D** (multi-beam sonar → LaserScan 25 Hz), handled by `sonoptix_2D_perception`. The **Ping360** rotating sonar plays a secondary role: initial net orientation during `GLOBAL_SEARCH` (`ping360_nearest`).

---

## Package Contents

| Node / executable | Sensor | Output | Used in |
|---|---|---|---|
| `ping360_nearest` | Ping360 (LaserScan — 360° sweep) | Net orientation yaw | Phase 2 GLOBAL_SEARCH |
| `sonoptix_2D_perception` | Sonoptix ECHO 2D (LaserScan — 25 Hz) | Net distance + yaw target | Phase 2 APPROACHING + Phase 3 orbit |
| `sonoptix_perception` | Sonoptix ECHO 3D (PointCloud2) | Net distance + normal (PoseStamped) | Legacy — 3D pipeline |
| `ping360_bridge_player` | — | Replays Ping360 bags | Offline testing |
| `auto_saver_node` | OctoMap map server | Periodically saves 3D net map as `.bt` | Research / mapping |

---

## Quick Start

These nodes are launched automatically by the main mission file (`net_full_inspection_true_auv.launch.py`). To run them individually for testing:

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select auv_perception
source install/setup.bash

# 2D Sonoptix net detector (requires /sonoptix/points LaserScan to be publishing)
ros2 run auv_perception sonoptix_2D_perception

# Ping360 net orientation finder (requires /ping360/scan to be publishing)
ros2 run auv_perception ping360_nearest

# OctoMap Auto-Saver (requires octomap_server to be running)
ros2 run auv_perception auto_saver_node
```

---

## Node Reference

### `sonoptix_2D_perception` — 2D RANSAC Net Estimator ⭐ (main pipeline)

**Purpose:** Processes the Sonoptix ECHO multi-beam sonar (bridged as a 25 Hz `LaserScan`) to compute the net's distance and orientation. This is the **primary perception output** used by both Phase 2 (approach) and Phase 3 (orbit).

**Algorithm:**
1. Receives `sensor_msgs/LaserScan` from `/sonoptix/points` at 25 Hz.
2. Converts each ray (range + angle) to a 2D Cartesian point `(x, y)` in the sensor frame.
3. Applies range filter: discards NaN, out-of-range, and saturated returns.
4. **Axis-swap heuristic**: chooses the axis with highest variance as the independent variable to avoid near-vertical singularity.
5. **RANSAC polynomial deg-2 fit**: finds the best parabola through the net echo (200 iterations).
6. Samples the fitted curve (500 points) to find the closest point to the sensor origin.
7. Computes the inward-pointing normal → yaw angle → applies **EMA temporal filter** (α = 0.25) to reduce sign-flip noise.
8. Publishes distance, yaw target, and validity.

**Key parameters (settable at launch):**

| Parameter | Default | Description |
|---|---|---|
| `min_range` | `0.3` m | Near dead-zone |
| `max_range` | `7.0` m | Far cut-off |
| `ransac_residual_threshold` | `0.05` m | Inlier distance threshold |
| `ransac_min_inliers_ratio` | `0.30` | Minimum inlier fraction to validate |
| `min_points` | `10` | Minimum points to attempt RANSAC |

**Topics:**

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/sonoptix/points` | `sensor_msgs/LaserScan` |
| Publish | `/perception/net_distance` | `std_msgs/Float32` |
| Publish | `/perception/net_yaw_target` | `std_msgs/Float32` |
| Publish | `/perception/perception_valid` | `std_msgs/Bool` |

**Foxglove debug markers (under `~/debug/`):**

| Topic | Marker type | Colour | Content |
|---|---|---|---|
| `~/debug/raw_cloud` | `POINTS` | Grey | All range-filtered Cartesian points |
| `~/debug/inlier_cloud` | `POINTS` | Green | RANSAC inlier points (= the net echo) |
| `~/debug/ransac_curve` | `LINE_STRIP` | Red | Fitted parabola (80-point sample) |
| `~/debug/normal_arrow` | `ARROW` | Cyan | Inward normal from net to AUV |

---

### `ping360_nearest` — Net Orientation Estimator

**Purpose:** Finds the net's direction relative to the robot using the 360° Ping360 sonar scan. Used **only** during Phase 2 `GLOBAL_SEARCH` to compute the initial yaw toward the net.

**Algorithm (v3 — RANSAC inlier-ratio cluster selection):**
1. Receives `sensor_msgs/LaserScan` from `/ping360/scan`.
2. Transforms each valid beam from `ping360_link` → `odom` frame via TF2 (compensates for robot motion).
3. Accumulates points over **one full 360° rotation** in a rolling buffer.
4. At end of rotation: runs **DBSCAN clustering** (eps=0.25m, min_pts=5).
5. For each cluster: fits a degree-2 RANSAC polynomial and measures inlier ratio.
6. Selects the cluster with ratio ≥ 30% (net echo has high ratio; fish school has low ratio).
7. Finds the closest point on the fitted curve → tangent → normal → target yaw.

**Key parameters:**

| Parameter | Default | Description |
|---|---|---|
| `max_range_m` | `5.0` | Ignore returns beyond this distance |
| `dbscan_eps` | `0.25` | DBSCAN neighbourhood radius [m] |
| `dbscan_min_samples` | `5` | Minimum cluster size |
| `ransac_min_inlier_ratio` | `0.30` | Minimum inlier fraction to validate as net |

**Topics:**

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/ping360/scan` | `sensor_msgs/LaserScan` |
| Publish | `/perception/net_orientation` | `geometry_msgs/PoseStamped` |
| Publish | `/perception/full_scan_ready` | `std_msgs/Bool` |

---

### `sonoptix_perception` — 3D Plane Estimator (legacy)

**Purpose:** Processes a 3D Sonoptix `PointCloud2` stream to estimate the net plane via RANSAC. This is the **older 3D pipeline** — the active mission now uses `sonoptix_2D_perception` instead.

**Topics:**

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/sonoptix/points` | `sensor_msgs/PointCloud2` |
| Publish | `/sonoptix/perception` | `geometry_msgs/PoseStamped` |
| Publish | `/sonoptix/perception_valid` | `std_msgs/Bool` |

---

### `auto_saver_node` — OctoMap Autosave

**Purpose:** Ensures the 3D map of the net (generated by `octomap_server`) is not lost if the simulation crashes.

Runs a 60-second timer. On tick, spawns `octomap_saver_node`. Also saves on `Ctrl+C`.

**Save location:** `~/AUV_project/ros2_AUV/src/auv_perception/net_map_autosave.bt`

---

## Maintenance

### Tuning the 2D Sonoptix RANSAC

If the net is not detected reliably:
- **Increase `ransac_residual_threshold`** (e.g., `0.05` → `0.10`) to tolerate more sonar noise.
- **Decrease `ransac_min_inliers_ratio`** (e.g., `0.30` → `0.20`) for sparser net echoes.
- **Decrease `min_range`** if the robot is very close (< 0.3 m) to the net.

Watch the Foxglove debug markers (`~/debug/*`) to see what RANSAC is doing in real time.

### Tuning the Ping360 clustering

If the robot gets stuck in `GLOBAL_SEARCH`:
- **Decrease `dbscan_eps`** to separate tightly packed clusters more aggressively.
- **Decrease `dbscan_min_samples`** to detect smaller echoes.
- Check that `/ping360/scan` is publishing (verify bridge is running).

### Adding Open3D (for 3D legacy pipeline)

```bash
pip install open3d
```

If Open3D is not installed, `sonoptix_perception` automatically falls back to sklearn.
