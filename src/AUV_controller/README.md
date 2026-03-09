# AUV_controller

This package contains the control and navigation algorithms for the AUV developed in the ROS 2 environment.

It implements controllers ranging from Station Keeping using PID to Model Predictive Control (MPC) to reach a waypoint.

## Main Features

- **MPC Control (Model Predictive Control)**: Uses `do_mpc` and `CASADI` to optimize the submarine's trajectory while respecting physical constraints (maximum thruster force, maximum moments) and the robot's hydrodynamic model.
- **Station Keeping (PID)**: A classic and robust PID algorithm allowing the AUV to maintain a fixed position and depth against perturbations and buoyancy mismatches.
- **Open-loop tests**: Various basic scripts to individually test the motors or validate the physical model and hardware integration in Gazebo.

## Main Nodes and Scripts

The package is structured to offer several levels of control:

### 1. Advanced Controllers (Navigation)
- **`mpc_controller_realistic.py`**: Similar to the sensors version but tuned specifically for the realistic BlueROV2 model.
- **`mpc_controller_sensors.py`**: Main MPC controller designed to work with the robot's sensors. Subscribes to `/odometry/filtered` and calculates optimal thruster commands.
- **`station_keeping.py`**: Robust and fast PID controller to maintain the AUV at a stable `(x, y, z, yaw)` position.
- **`mpc_controller_blueROV.py`**: Theoretical version of the MPC, subscribing directly to exact ground truth `/odom`.

### 2. Utilities & Tests (Open Loop)
Moved or structured in specific sub-folders (`bluerov/` or `tools/`), we find:
- **`move_forward.py`**, **`move_down.py`**: Basic scripts applying a constant or progressive force. Ideal for testing the direction of rotation of the thrusters or validating that communication with the controller is done correctly, in a unitary way.

## Dependencies

This ROS 2 Python package has strict mathematical dependencies related to MPC control:

- `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`
- **`do_mpc`**: Python framework for Model Predictive Control.
- **`casadi`**: Mathematical framework for non-linear optimization (default backend for do_mpc).
- `numpy`

## Usage

Make sure to have instantiated the simulator or the real robot before linking the controllers.

1. Build and source the workspace:
```bash
colcon build --packages-select AUV_controller
source install/setup.bash
```

2. Launch the Station Keeping controller:
```bash
ros2 run AUV_controller station_keeping
```

3. Launch the realistic MPC controller:
```bash
ros2 run AUV_controller mpc_controller_realistic
```

4. Launch the standard sensors MPC:
```bash
ros2 run AUV_controller mpc_controller_sensors
```
