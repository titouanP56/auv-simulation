# AUV DVL Bridge

> **Tested environment:** ROS 2 **Jazzy** + Gazebo **Harmonic** on Ubuntu 24.04 LTS.

## 1. Introduction for Beginners

Welcome to the **AUV DVL Bridge** package! 

A DVL (Doppler Velocity Log) is a special acoustic sensor that acts like an underwater speedometer. By pinging the sea floor with sound, it tells the robot exactly how fast it is moving forward, sideways, and up/down. 

Because we use the advanced Gazebo Harmonic simulator, the simulated DVL sensor outputs its data using a communication system called "Protobuf" (Gazebo Transport). However, the rest of our robot's brain expects data in a format called "ROS 2". This package is a dedicated translator that listens to the simulator, instantly translates the speed data into ROS 2 format, and hands it over to the robot's navigation system.

---

## 2. Quick Start Guide

### Prerequisites
Make sure ROS 2 Jazzy is installed and your workspace is built. See the [root README installation guide](../../README.md#2-system-requirements--installation) if this is your first time.

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select auv_dvl_bridge
source install/setup.bash
```

### Running the Bridge

This bridge is usually launched automatically along with the simulation. To run it manually:

```bash
ros2 run auv_dvl_bridge dvl_bridge_node
```

You can verify it is working by echoing the translated ROS 2 topic:
```bash
ros2 topic echo /dvl/velocity_ros
```

---

## 3. Technical Architecture

This package is implemented in C++ for maximum performance and minimum latency, as velocity data is critical for real-time sensor fusion.

### Core Node

**`dvl_bridge_node`**:
- **Role**: Translates Gazebo DVL messages to ROS 2.
- **Logic**: 
  - Initializes a Gazebo Transport node (`gz::transport::Node`).
  - Subscribes to the Gazebo topic `/dvl/velocity` which uses the `gz::msgs::DVLVelocityTracking` protobuf message.
  - Converts the linear velocities ($x, y, z$). **Crucial fix**: Gazebo outputs $Y$ velocity as "Right" (starboard), but ROS standard (`base_link`) expects $Y$ to be "Left" (port). The bridge negates the $Y$ value to ensure the EKF doesn't drift in the wrong direction.
  - Maps the 9-element 3x3 covariance matrix from Gazebo into the 36-element 6x6 covariance matrix expected by the `TwistWithCovariance` message (filling only the linear velocity components). If no covariance is provided, it injects a fallback value (0.01).
- **Output**: Publishes a `geometry_msgs/msg/TwistWithCovarianceStamped` on `/dvl/velocity_ros`.

### Subscribed Topics (Gazebo Transport)
- `/dvl/velocity` (`gz::msgs::DVLVelocityTracking`): Raw simulator output.

### Published Topics (ROS 2)
- `/dvl/velocity_ros` (`geometry_msgs/msg/TwistWithCovarianceStamped`): Processed velocity data for the EKF.

---

## 4. Maintenance Guide

If you are a developer modifying this package:

- **Compilation**: This package depends on `gz-transport13` and `gz-msgs10` (Gazebo Harmonic versions). If CMake fails, verify your Gazebo Harmonic installation and that `GZ_VERSION=harmonic` is set in your environment. The `ros-jazzy-ros-gz` apt package should install all required Gazebo libraries.
- **Modifying Covariance Fallback**: If you find the EKF is trusting the DVL too much when the simulation doesn't provide covariance, open `src/dvl_bridge_node.cpp` and increase the fallback values (`0.01` to `0.1` or higher).
- **Frame ID**: The bridge hardcodes the frame ID to `"base_link"` because Gazebo often attaches complex, unmapped names (like `BlueROV2::base_link::dvl_sensor`). If you add a proper TF link for the DVL in the URDF, you can update this to `"dvl_link"` and let `robot_localization` handle the coordinate transform.
