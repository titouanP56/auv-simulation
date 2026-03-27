# AI Repository Guide: ROS 2 AUV Project

This document is designed for AI agents (Gemini, ChatGPT, etc.) to quickly understand the organization and technical details of this ROS 2 AUV repository.

---

## 🏗️ High-Level Architecture

The repository follows a modular architecture inspired by Fossen's marine robot control system:

1.  **Guidance (`AUV_guidance`)**: High-level trajectory and mission management.
2.  **Controller (`AUV_controller`)**: Low-level execution using MPC or PID.
3.  **Localization (`my_auv_localization`)**: 6-DOF state estimation (EKF).
4.  **Dynamics & Simulation (`AUV_description`)**: Physical models (URDF) and Gazebo environments.
5.  **Perception (`auv_perception`)**: 3D mapping (OctoMap) and sonar filtering.
6.  **Bridges (`auv_dvl_bridge`)**: Interfaces between Gazebo sensors and ROS 2.

---

## 📦 Package Details

### 1. `AUV_guidance`
**Role**: High-level decision maker.
*   **Nodes**:
    *   `net_approach`: Manages Phase 2 mission transitions (approach to net and standoff).
    *   `lawnmower_trajectory_node`: Generates lawnmower scanning patterns for depth-consistent searches.
*   **Main Launch**: `net_inspection.launch.py` (integrates the full mission stack).
*   **Topics**: Often publishes to `/cmd_vel` or `/guidance/target_pose`.

### 2. `AUV_controller`
**Role**: Low-level control and thruster allocation.
*   **Nodes**:
    *   `mpc_controller_realistic`: Advanced Model Predictive Control tuned for the BlueROV2 physics in Gazebo.
    *   `station_keeping`: PID-based node for holding a specific 3D pose.
    *   `move_forward` / `move_down`: Open-loop scripts for hardware/thruster testing.
*   **Key Topics**: Subscribes to `/odometry/filtered`, publishes to `/cmd_vel` or directly to thruster topics.

### 3. `my_auv_localization`
**Role**: Sensor fusion for state estimation.
*   **Implementation**: Relies on `robot_localization` (EKF).
*   **Input Fusion**:
    *   `IMU`: Angular rates (Orientation is relative to avoid heading bias).
    *   `DVL`: Linear velocities (Vx, Vy, Vz) in body frame.
    *   `Depth Sensor`: Absolute Z position.
*   **Output**: `/odometry/filtered` (6-DOF pose).

### 4. `AUV_description`
**Role**: Robot visuals, physical properties, and simulation world.
*   **URDF/Xacro**:
    *   `Bluerov2_realistic.urdf.xml`: Optimized version for MPC (includes correct hydrodynamics parameters).
    *   `BlueROV2captors.urdf.xml`: Full sensor suite (DVL, IMU, Sonar).
*   **Worlds**: `ocean_40m.xml` (realistic ocean environment with net), `Bassin_ntnu.xml` (controlled basin).
*   **Misc**: Contains the `simulated_depth_sensor` node.

### 5. `auv_perception`
**Role**: Mapping and raw data processing.
*   **Nodes**:
    *   `sonar_filter_node`: Cleans point clouds from the Sonoptix sonar (removes out-of-range noise > 4m).
    *   `auto_saver_node`: Automatically saves OctoMaps (`.bt`) every 60s.
*   **Mapping**: Uses `octomap_server` to generate 3D voxel grids from `/sonoptix/points_filtered`.
*   **Main Launch**: `mapping.launch.py`.

### 6. `auv_dvl_bridge`
**Role**: Hardware/Simulation interface.
*   **Function**: Converts Gazebo `gz::msgs::DVLVelocityTracking` to ROS 2 standard `geometry_msgs/TwistWithCovarianceStamped`.
*   **Fixes**: Standardizes `frame_id` to `base_link` and formats the covariance matrix for EKF compatibility.

---

## 🚀 Common Workflows

### Build & Source
```bash
colcon build --symlink-install
source install/setup.bash
```

### Run Full Mission (Net Inspection)
```bash
ros2 launch AUV_guidance net_inspection.launch.py
```

### Start Mapping Independently
```bash
ros2 launch auv_perception mapping.launch.py
```

---

## 📊 Key Topics & Frames

*   **Frames**: `odom` (World fixed), `base_link` (Robot center).
*   **Control**: `/cmd_vel` (`geometry_msgs/Twist`).
*   **Sensors**: `/imu/data`, `/dvl/velocity_ros`, `/sonoptix/points`.
*   **Map**: `/octomap_binary`, `/octomap_point_cloud_centers`.

---

## 🛠️ Configuration Checklist

- **MPC Tuning**: Check `AUV_controller` weights if the robot oscillates.
- **EKF Noise**: Adjust `ekf.yaml` in `my_auv_localization` if position drifts.
- **Sonar Range**: `sonar_filter_node` in `auv_perception` has a hardcoded 4.0m cutoff.
