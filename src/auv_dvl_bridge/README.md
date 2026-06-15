# AUV DVL Bridge

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This C++ package translates Gazebo's DVL (Doppler Velocity Log) protobuf messages into standard ROS 2 messages for the EKF.

---

## What is a DVL?

A DVL is an acoustic velocity sensor. It pings the sea floor and measures how fast the robot is moving in X, Y, Z by analysing the Doppler shift of the echoes — essentially an underwater speedometer.

Gazebo Harmonic outputs DVL data using its own binary format (Protobuf via gz-transport). The rest of the stack speaks ROS 2. This node is the translator.

---

## Quick Start

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select auv_dvl_bridge
source install/setup.bash

# Run manually (usually launched automatically with the simulation)
ros2 run auv_dvl_bridge dvl_bridge_node

# Verify output
ros2 topic echo /dvl/velocity_ros
```

---

## How It Works

The node (`dvl_bridge_node.cpp`):

1. Creates a Gazebo Transport node and subscribes to `/dvl/velocity` (`gz::msgs::DVLVelocityTracking`).
2. **Y-axis flip:** Gazebo outputs Y as "right" (starboard), but ROS 2 `base_link` uses Y as "left" (port). The bridge negates Y to prevent the EKF from drifting sideways.
3. Maps the 3×3 Gazebo covariance into the 6×6 `TwistWithCovariance` matrix (linear velocity block only). Falls back to `0.01` if no covariance is provided.
4. Publishes on `/dvl/velocity_ros`.

| Direction | Topic | Type |
|---|---|---|
| Subscribe (Gazebo) | `/dvl/velocity` | `gz::msgs::DVLVelocityTracking` |
| Publish (ROS 2) | `/dvl/velocity_ros` | `geometry_msgs/TwistWithCovarianceStamped` |

---

## Maintenance

- **Build dependencies:** Requires `gz-transport13` and `gz-msgs10` (shipped with Gazebo Harmonic). If CMake fails, verify your Gazebo install and that `ros-jazzy-ros-gz` is installed.
- **Covariance fallback:** If the EKF trusts the DVL too much, increase the fallback value from `0.01` to `0.1` or higher in `src/dvl_bridge_node.cpp`.
- **Frame ID:** The bridge hardcodes `frame_id = "base_link"`. If you add a dedicated `dvl_link` TF in the URDF, update this accordingly so `robot_localization` can handle the coordinate transform.
