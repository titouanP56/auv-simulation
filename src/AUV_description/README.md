# AUV_description

This package contains the robot description (URDF/Xacro), Gazebo simulation worlds, and launch files for the AUV within the `ros2_AUV` workspace.

## Main Features
- **Robot Descriptions**: URDF and Xacro files for the BlueROV2, including modular configurations with various sensors (DVL, IMU, Camera, Ping360 Sonar, Sonoptix Echo, depth sensor) and a a **realistic version**.
- **Gazebo Simulation**: Simulation environment (`Bassin_ntnu.xml`).
- **Launch Files**: Comprehensive launch files that spawn the robot in Gazebo, start the `robot_state_publisher`, and set up `ros_gz_bridge` for communication between Gazebo and ROS 2.
- **Utility Nodes (Python)**: Includes nodes to correct or process simulation data for ROS 2 navigation (`simulated_depth_sensor`, `imu_republisher`).

## Package Structure
- `urdf/`: URDF and Xacro description files for the AUVs.
- `world/`: Gazebo simulation environments (in SDF/XML format).
- `launch/`: ROS 2 launch scripts.
- `AUV_description/`: Python scripts (ROS 2 nodes) for sensor data processing.

## Main Launch Files

- `bluerov2_bassin.launch.py`: Launches the BlueROV2 in the NTNU basin environment.
- `bluerov2_bassin_captors.launch.py`: Launches the BlueROV2 **fully equipped with sensors** in the NTNU basin. Also starts localization nodes and communication bridges.
- `test_bluerov2_realistic.launch.py`: Launches the **realistic BlueROV2** in the NTNU basin with full sensor suite (DVL, IMU, Sonars).
- `bluerov2_ocean_realistic.launch.py`: Launches the **realistic BlueROV2** in a **40m deep ocean world** containing a large net model.
- `bluerov2_bassin_waves.launch.py`: Launches the **realistic BlueROV2** in the NTNU basin with waves.

## Usage

To launch the BlueROV2 in the NTNU basin:

```bash
colcon build
source install/setup.bash
ros2 launch AUV_description bluerov2_bassin_captors.launch.py
```

This will launch Gazebo and spawn the robot.
It will also set up parameter bridges for sensors and thrusters, publish the TF tree, and launch utility nodes (e.g., IMU and depth sensor).

## Dependencies
- `ros_gz_sim`
- `ros_gz_bridge`
- `robot_state_publisher`
- `xacro`
- `tf2_ros`
- Workspace packages: `auv_dvl_bridge`, `my_auv_localization`
