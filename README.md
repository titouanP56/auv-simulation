# ROS 2 AUV Project

Welcome to the ROS 2 AUV (Autonomous Underwater Vehicle) project! This workspace contains everything you need to simulate and control an underwater robot (specifically based on the BlueROV2) in a 3D environment.


## Project Overview

The project is divided into differents packages:

1. **`AUV_guidance`** (Guidance): The high-level decision maker. Generates trajectories (e.g., Lawnmower patterns) and manages mission state transitions (e.g., Phase 2 approach).
2. **`AUV_controller`** (Control): The low-level execution layer. Calculates the physical forces needed to follow the guidance instructions using Model Predictive Control (MPC) or PID Station Keeping.
3. **`AUV_description`** (Dynamics): Contains the physical 3D model of the submarine (URDF/Xacro), its sensors, and the virtual simulation environments (Gazebo).
4. **`my_auv_localization`** (Navigation): Fuses sensor data (IMU, DVL) using an Extended Kalman Filter (EKF) to estimate the robot's precise 3D position and orientation.
5. **`auv_dvl_bridge`**: Hardware/Simulation interface that translates raw DVL sensor data into standard ROS 2 formats.

## Prerequisites

Before starting, ensure you have:
- A Linux computer (Ubuntu is recommended).
- **ROS 2** installed (e.g., Jazzy).
- **Gazebo** installed (the 3D simulation software).

## How to Use the Project

Follow these steps to launch the simulation and make the robot move or execute missions.

### 1. Build the Workspace

First, compile the code so the system can run it. Open a terminal and run the following commands:

```bash
# Go to the project folder
cd ~/AUV_project/ros2_AUV

# Compile the code
colcon build

# Source the newly compiled programs
source install/setup.bash
```

*Note: You must run `source install/setup.bash` every time you open a new terminal.*

### 2. Launch the 3D Simulation

Start the virtual environment and spawn the robot. In the same terminal, run:

- **Phase 4 Net Inspection (Full Integration)**:
  Launches the realistic ocean environment, the robot with all sensors, and delays the start of the Guidance and Control modules to inspect a net.
  ```bash
  ros2 launch AUV_guidance net_inspection.launch.py
  ```

Alternatively, you can launch standalone simulation environments (useful for manual testing):
- **Equipped with sensors in a basin**: `ros2 launch AUV_description bluerov2_bassin_captors.launch.py`
- **Realistic AUV in the bassin**: `ros2 launch AUV_description bluerov2_realist_bassin.launch.py`

### 3. Start the Guidance or Controllers manually

If you didn't launch the full `phase4` integrated mission, you can start the components manually in a **new, sourced terminal**.

**1. Guidance (Missions & Trajectories) (Not recommanded):**
- **Phase 2 Mission (Approach & Standoff)**:
  ```bash
  ros2 run AUV_guidance net_approach
  ```
- **Lawnmower Trajectory**:
  ```bash
  ros2 run AUV_guidance lawnmower_trajectory_node
  ```

**2. Control Algorithms:**
- **MPC with Sensors**: Standard MPC using real sensor data (EKF odometry).
  ```bash
  ros2 run AUV_controller mpc_controller_sensors
  ```
- **Station Keeping**: Makes the robot hold its current position.
  ```bash
  ros2 run AUV_controller station_keeping
  ```

**3. Basic Open-Loop Movement (Hardware Testing):**
These scripts apply a constant force to test engines:
- **Move Forward**:
  ```bash
  ros2 run AUV_controller move_forward
  ```
- **Move Down**:
  ```bash
  ros2 run AUV_controller move_down
  ```

## Troubleshooting

- **"Command not found: colcon"** or **"ros2: command not found"**: You probably forgot to source your main ROS 2 installation. Run `source /opt/ros/humble/setup.bash` (replace `humble` with your ROS 2 version).
- **The robot spins or flies out of the water**: Physics simulations can sometimes glitch. Close the terminals (using `Ctrl+C`) and start the simulation again.
- **Gazebo is very slow**: 3D simulations require a decent graphics card. If it's too slow, make sure your computer is plugged in and using its dedicated GPU if it has one.

Enjoy experimenting with your autonomous underwater vehicle!

## Visualization with Foxglove

To visualize the robot's sensors and telemetry (cameras, sonar, 3D pose) in real-time using [Foxglove](https://foxglove.dev/), we use the **Foxglove Bridge**.

1. **Install Foxglove Bridge** (if not already installed):
   ```bash
   sudo apt install ros-jazzy-foxglove-bridge
   ```

2. **Launch the Foxglove Bridge Node**:
   Open a new terminal, source your workspace, and run:
   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```
   *(By default, this opens a WebSocket connection on port `8765`)*

3. **Connect from Foxglove Studio**:
   - Open the **Foxglove Studio** app (desktop or web version).
   - Click "**Open connection**".
   - Select "**Foxglove WebSocket**".
   - Enter the URL: `ws://172.19.68.228:8765` (or replace `localhost` with your robot's IP if running on a different machine).
   - You can now visualize topics like `/odom`, `/camera/image_raw`, `/ping360/scan`, and 3D models!
