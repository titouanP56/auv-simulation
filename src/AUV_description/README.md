# AUV_description

This package contains the robot description (URDF/Xacro), Gazebo simulation worlds, and launch files for the AUV within the `ros2_AUV` workspace.

## Main Features
- **Robot Descriptions**: URDF and Xacro files for the BlueROV2, including modular configurations with various sensors (DVL, IMU, Camera, Ping360 Sonar, Sonoptix Echo, depth sensor) and a a **realistic version**.
- **Gazebo Simulation**: Simulation environments (`Bassin_ntnu.xml`, `small_net.xml`, `small_net_deforme.xml`).
- **Optimized Net Models**: `small_net.xml` utilizes a small version of the net for faster testing. `small_net_deforme.xml` additionally includes a static cylinder (5m diameter, 6m depth) slightly deformed to test the robustness of the algorithms.
- **Launch Files**: Comprehensive launch files that spawn the robot in Gazebo, start the `robot_state_publisher`, and set up `ros_gz_bridge` for communication between Gazebo and ROS 2.
- **Utility Nodes (Python)**: Includes nodes to correct or process simulation data for ROS 2 navigation (`simulated_depth_sensor`, `imu_republisher`).

## Package Structure
- `urdf/`: URDF and Xacro description files for the AUVs.
- `world/`: Gazebo simulation environments (in SDF/XML format).
- `models/`: 3D models and assets (e.g., `fish_net` FBX meshes) loaded dynamically via ROS 2 `package://` URIs.
- `launch/`: ROS 2 launch scripts.
- `AUV_description/`: Python scripts (ROS 2 nodes) for sensor data processing.

## Main Launch Files

- `bluerov2_bassin.launch.py`: Launches the BlueROV2 in the NTNU basin environment.
- `bluerov2_bassin_captors.launch.py`: Launches the BlueROV2 **fully equipped with sensors** in the NTNU basin. Also starts localization nodes and communication bridges.
- `bluerov2_ocean_realistic.launch.py`: Launches the **realistic BlueROV2** in a **40m deep ocean world** containing a large net model.
- `bluerov2_bassin_waves.launch.py`: Launches the **realistic BlueROV2** in the NTNU basin with waves. (not working) 
- `bluerov2_realist_bassin.launch.py`: Launches the **realistic BlueROV2** in the NTNU basin without waves.
- `small_net.xml`: Optimized 10m diameter cylindrical net world for reactive Phase 3 inspection.
- `small_net_deforme.xml`: Deformed 10m diameter cylindrical net.

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
