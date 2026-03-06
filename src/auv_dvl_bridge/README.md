# auv_dvl_bridge

ROS 2 package providing a bridge between Gazebo Harmonic's DVL (Doppler Velocity Log) sensor output and ROS 2. 

It converts Gazebo's native `gz::msgs::DVLVelocityTracking` messages into standard ROS 2 `geometry_msgs/msg/TwistWithCovarianceStamped` messages, making them ready to be consumed by state estimation nodes like the Extended Kalman Filter (`robot_localization`).

## Why is this bridge needed?

While `ros_gz_bridge` handles many standard message types, Gazebo's DVL messages require specific handling:
1. **Message Translation**: Direct conversion from Gazebo's protobuf format to ROS 2 standard messages.
2. **Frame ID Correction**: Gazebo often outputs fully qualified scoped names (e.g., `BlueROV2::base_link::dvl_sensor`). This node forces the `frame_id` to `base_link` so the EKF readily accepts the measurements.
3. **Covariance Formatting**: It explicitly maps Gazebo's covariance array (sometimes 9 elements) into the 36-element `TwistWithCovariance` ROS 2 array. It also provides a fallback covariance if Gazebo does not provide one.

## Topics

| Name | Type | Direction | Description |
|---|---|---|---|
| `/dvl/velocity` | `gz::msgs::DVLVelocityTracking` | **Subscribed** (Gazebo) | Raw velocity data from the Gazebo DVL plugin. |
| `/dvl/velocity_ros` | `geometry_msgs/msg/TwistWithCovarianceStamped` | **Published** (ROS 2) | Filtered and formatted linear velocities (Vx, Vy, Vz). |

## Usage

This node is typically launched alongside the main simulation.

### Standalone
```bash
ros2 run auv_dvl_bridge dvl_bridge_node
```

The node automatically uses the `/clock` topic for simulation time synchronization when `use_sim_time` is set to true.

## Dependencies

- `rclcpp`
- `geometry_msgs`
- `gz-transport` (Gazebo Transport)
- `gz-msgs` (Gazebo Messages)
