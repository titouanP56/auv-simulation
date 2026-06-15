# AUV Localization

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This package configures the **Extended Kalman Filter (EKF)** that fuses DVL, IMU, and depth sensor data into a smooth, continuous 6-DOF pose estimate for the guidance nodes.

Underwater robots cannot use GPS. The EKF combines three complementary sensors to estimate position and orientation without it:

| Sensor | What it provides |
|---|---|
| **DVL** (`/dvl/velocity_ros`) | Linear velocity (Vx, Vy, Vz) |
| **IMU** (`/imu/fixed`) | Angular velocity (roll/pitch/yaw rates) |
| **Depth** (`/depth/pose`) | Absolute Z position |

---

## Quick Start

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select my_auv_localization
source install/setup.bash

# Run manually (usually launched automatically with the simulation)
ros2 launch my_auv_localization localization.launch.py

# Verify the EKF output
ros2 topic echo /odometry/filtered
```

---

## Configuration (`config/ekf.yaml`)

| Parameter | Value |
|---|---|
| Filter frequency | 30 Hz |
| 3D mode | Enabled (`two_d_mode: false`) |
| World frame | `odom` |
| Body frame | `base_link` |

**Sensor configuration:**

- **IMU** (`/imu/fixed`): Only angular velocities (roll/pitch/yaw rates) are fused. Absolute orientation is disabled to avoid heading offset issues in simulation.
- **DVL** (`/dvl/velocity_ros`): Linear velocities (Vx, Vy, Vz) fused in the body frame (`twist_body: true`).
- **Depth** (`/depth/pose`): Only Z position is fused, anchoring depth vertically.

---

## Topics

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Subscribe | `/imu/fixed` | `sensor_msgs/Imu` | Cleaned IMU data |
| Subscribe | `/dvl/velocity_ros` | `geometry_msgs/TwistWithCovarianceStamped` | DVL velocity |
| Subscribe | `/depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Depth measurement |
| Publish | `/odometry/filtered` | `nav_msgs/Odometry` | Fused 6-DOF pose estimate |
| Publish | `/tf` | TF2 | `odom` → `base_link` transform |

---

## Maintenance

- **Tune the EKF:** Edit `config/ekf.yaml`. The `process_noise_covariance` matrix controls how much the filter trusts its own motion model vs. the sensors. If the robot "jumps" in position, increase sensor covariance in the bridge nodes (`dvl_bridge_node.cpp`, `simulated_depth_sensor.py`, `imu_republisher.py`).

- **Add a new sensor:** Add a block to `ekf.yaml` (e.g., `pose1: /usbl/pose`) and set the 15-element boolean array to select which state variables that sensor provides (x, y, z, roll, pitch, yaw, Vx, Vy, Vz, …).

- **Real hardware:** The launch file uses `use_sim_time: true` by default. Set it to `false` when running on the real BlueROV2 to use the system clock.
