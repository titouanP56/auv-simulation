# AUV Perception

> **Tested environment:** ROS 2 **Jazzy** + Gazebo **Harmonic** on Ubuntu 24.04 LTS.

## 1. Introduction for Beginners

Welcome to the **AUV Perception** package! This package acts as the "eyes and memory" of the underwater robot.

When the robot swims around the aquaculture net, it receives thousands of data points every second from its sonar (like an underwater radar). This package is responsible for:
1. **Filtering the noise**: Removing echoes from things that are too far away or irrelevant.
2. **Understanding the shape**: Analyzing the filtered sonar points to figure out exactly how the net is angled relative to the robot.
3. **Saving memories**: Taking the 3D map that the robot builds as it swims (using a tool called OctoMap) and automatically saving it to the computer's hard drive so it doesn't get lost when we turn the robot off.

---

## 2. Quick Start Guide

### Prerequisites
Make sure ROS 2 Jazzy is installed and your workspace is built. See the [root README installation guide](../../README.md#2-system-requirements--installation) if this is your first time.

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select auv_perception
source install/setup.bash
```

### Running the Nodes

These nodes are usually launched automatically by the main mission launch files, but you can run them individually for testing:

**1. Run the Sonar Filter:**
Filters out sonar points beyond 4.0 meters.
```bash
ros2 run auv_perception sonar_filter_node
```

**2. Run the Net Local Estimator:**
Analyzes the filtered points to estimate the net's pose.
```bash
ros2 run auv_perception net_local_estimator
```

**3. Run the Auto Saver:**
Automatically saves the OctoMap every 60 seconds.
```bash
ros2 run auv_perception auto_saver_node
```

---

## 3. Technical Architecture

This package contains lightweight, specialized Python nodes that process `sensor_msgs/PointCloud2` data and manage system-level save operations.

### Core Nodes

1. **`sonar_filter_node`**: 
   - **Role**: Distance-based filtering of the raw Sonoptix point cloud.
   - **Logic**: Converts the `PointCloud2` into a numpy structured array, calculates the Euclidean distance for every point, and drops points where $D > 4.0$ meters.
   - **Output**: Publishes `/sonoptix/points_filtered`.

2. **`net_local_estimator`**:
   - **Role**: Estimates the localized pose of the net surface relative to the robot.
   - **Logic**: Takes the filtered point cloud, restricts points to a $\pm 45^\circ$ horizontal cone, and uses Principal Component Analysis (PCA) to perform a robust 2D line fit. The smallest eigenvector provides the normal vector to the net. It applies a moving-average filter (window=5) to smooth the calculated distance and yaw.
   - **Output**: Publishes a `geometry_msgs/PoseStamped` on `/perception/net_local_frame` representing the closest point on the net and its normal orientation.

3. **`auto_saver_node`**:
   - **Role**: Persistent storage of the OctoMap.
   - **Logic**: Uses a ROS 2 timer to trigger a system `subprocess` every 60 seconds. It calls the `octomap_saver_node` executable to save the current `.bt` file into the `auv_perception` source directory. Captures `KeyboardInterrupt` (Ctrl+C) to guarantee a final save on shutdown.

### Subscribed Topics
- `/sonoptix/points` (`sensor_msgs/PointCloud2`): Raw sonar point cloud.
- `/sonoptix/points_filtered` (`sensor_msgs/PointCloud2`): Filtered sonar data (used by the estimator).

### Published Topics
- `/sonoptix/points_filtered` (`sensor_msgs/PointCloud2`): Output of the filter node.
- `/perception/net_local_frame` (`geometry_msgs/PoseStamped`): Estimated pose of the net.

---

## 4. Maintenance Guide

If you are a developer taking over this project, here is how you can modify or improve the perception code:

- **Adjusting the Sonar Filter Distance**: Open `sonar_filter_node.py` and modify the condition `distances <= 4.0`. If you are testing in a very large environment, you may want to increase this threshold.
- **Tuning the Net Estimator Smoothing**: In `net_local_estimator.py`, the `deque` sizes for `distance_history` and `angle_history` dictate how much the normal vector is smoothed. Increase `maxlen=5` for a smoother but more sluggish response.
- **Changing the Map Save Location**: The `auto_saver_node.py` dynamically resolves the package path to save `net_map_autosave.bt` directly into the `src/auv_perception` folder. If this fails, it defaults to your home directory (`~/net_map_autosave.bt`). You can modify the `save_path` variable to target a dedicated logging directory.
