# AUV Localization

## 1. Introduction for Beginners

Welcome to the **AUV Localization** package! This package answers the most important question for any robot: *"Where am I?"*

Underwater, GPS doesn't work. To figure out where it is, the robot uses an **Extended Kalman Filter (EKF)**. You can think of the EKF as a very smart detective that constantly gathers clues from different sensors:
- **The DVL (Doppler Velocity Log)** tells it how fast it's swimming.
- **The Depth Sensor** tells it how deep it is.
- **The IMU (Inertial Measurement Unit)** acts like its inner ear, telling it how it's tilting and turning.

By combining all these clues, the EKF creates a smooth, reliable estimate of the robot's exact position and orientation in the water, which the controllers use to navigate.

---

## 2. Quick Start Guide

### Prerequisites
Make sure your ROS 2 workspace is sourced and built.

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select my_auv_localization
source install/setup.bash
```

### Running the Localization Filter

This package is usually launched automatically along with the simulation. To run it manually:

```bash
ros2 launch my_auv_localization localization.launch.py
```

You can verify the filter is working by looking at the smoothed odometry:
```bash
ros2 topic echo /odometry/filtered
```

---

## 3. Technical Architecture

This package is a configuration wrapper around the standard `robot_localization` ROS 2 package. It configures a 3D Extended Kalman Filter to fuse our specific sensor setup.

### Sensor Fusion Setup (`config/ekf.yaml`)

- **Filter Frequency**: 30 Hz
- **3D Mode**: Enabled (`two_d_mode: false`)
- **World Frame**: `odom`
- **Base Frame**: `base_link`

**Inputs:**
1. **IMU** (`/imu/fixed`): Only angular velocities (roll/pitch/yaw rates) are fused. Absolute orientation is disabled to avoid initial heading offset issues in the simulator.
2. **DVL** (`/dvl/velocity_ros`): Linear velocities ($V_x, V_y, V_z$) are fused in the robot body frame (`twist_body: true`).
3. **Depth** (`/depth/pose`): Only the absolute $Z$ position is fused, anchoring the robot vertically.

### Subscribed Topics
- `/imu/fixed` (`sensor_msgs/Imu`): Cleaned IMU data.
- `/dvl/velocity_ros` (`geometry_msgs/TwistWithCovarianceStamped`): Cleaned DVL data.
- `/depth/pose` (`geometry_msgs/PoseWithCovarianceStamped`): Simulated depth sensor data.

### Published Topics
- `/odometry/filtered` (`nav_msgs/Odometry`): Smooth, continuous 6-DOF pose estimate.
- `/tf`: Broadcasts the dynamic transform from `odom` → `base_link`.

---

## 4. Maintenance Guide

If you are a developer tuning the robot's localization:

- **Tuning the EKF**: Open `config/ekf.yaml`. The matrices (`process_noise_covariance`, `initial_estimate_covariance`) define how much the filter trusts its own mathematical model vs. the sensors. If the robot "jumps" around, you might need to adjust the sensor covariances in the bridge nodes.
- **Adding a Sensor**: If you add a new sensor (like an underwater USBL or visual odometry), add a new block to `ekf.yaml` (e.g., `pose1: /usbl/pose`) and define which of the 15 variables (x, y, z, roll, pitch, yaw, etc.) the filter should ingest from that sensor by modifying the `[false, false, ...]` boolean array.
- **Simulation Time**: The launch file defaults to `use_sim_time: true`. If you run this on real hardware, ensure this is set to `false`.
