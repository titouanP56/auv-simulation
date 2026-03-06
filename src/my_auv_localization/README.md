# my_auv_localization

ROS2 package for AUV state estimation using an Extended Kalman Filter (EKF). It fuses data from the DVL, IMU and depth sensor to produce a smooth, reliable 6-DOF pose estimate.

## Contents

```
my_auv_localization/
├── config/
│   └── ekf.yaml            # EKF configuration (sensors, noise covariance)
└── launch/
    └── localization.launch.py  # Launches the EKF node
```

## How it works

The package relies on the standard `robot_localization` EKF node. Three sensor sources are fused:

| Source | Topic | Data used |
|---|---|---|
| IMU | `/imu/fixed` | Angular velocity (roll/pitch/yaw rates) |
| DVL | `/dvl/velocity_ros` | Linear velocity (Vx, Vy, Vz) in `base_link` frame |
| Depth sensor | `/depth/pose` | Absolute Z position |

The filter outputs the estimated pose on `/odometry/filtered`, which is consumed by the MPC controller.

## EKF Configuration (`ekf.yaml`)

| Parameter | Value |
|---|---|
| Filter frequency | 30 Hz |
| Sensor timeout | 0.1 s |
| 3D mode | Yes (`two_d_mode: false`) |
| World frame | `odom` |
| Base frame | `base_link` |

### Sensor fusion details

- **IMU** (`/imu/fixed`): only angular velocities are fused. Orientation is disabled to avoid initial heading offset issues. `imu0_relative: true`.
- **DVL** (`/dvl/velocity_ros`): linear velocities (Vx, Vy, Vz) are fused in the robot body frame (`twist_body: true`). Provided by the `auv_dvl_bridge` package.
- **Depth** (`/depth/pose`): only the Z position is fused, giving an absolute depth reference.

## Topics

| Topic | Type | Role |
|---|---|---|
| `/imu/fixed` | `sensor_msgs/Imu` | Input — IMU angular velocities |
| `/dvl/velocity_ros` | `geometry_msgs/TwistWithCovarianceStamped` | Input — DVL linear velocities |
| `/depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Input — absolute depth |
| `/odometry/filtered` | `nav_msgs/Odometry` | Output — filtered 6-DOF pose |
| `/tf` | TF tree | Output — `odom` → `base_link` transform |

## Launch

```bash
# Standalone
ros2 launch my_auv_localization localization.launch.py

# Typically included in the main simulation launch
ros2 launch AUV_description bluerov2_bassin_captors.launch.py
```

> **Note:** `use_sim_time: true` is set by default. The node synchronises with Gazebo's simulation clock via the `/clock` topic.

## Dependencies

- [`robot_localization`](https://docs.ros.org/en/humble/p/robot_localization/) — EKF/UKF state estimation
- `auv_dvl_bridge` — DVL bridge (same workspace)
- `AUV_description` — provides the `simulated_depth_sensor` node
