# ROS 2 AUV Project

Welcome to the ROS 2 AUV (Autonomous Underwater Vehicle) project! This workspace contains everything you need to simulate and control an underwater robot (specifically based on the BlueROV2) in a 3D environment.


## Project Overview

The project is divided into 4 main parts (called "packages"):

1. **`AUV_description`**: Contains the 3D model of the submarine, its sensors (cameras, sonar, etc.), and the virtual 3D pool where it swims (using the Gazebo simulator).
2. **`AUV_controller`**: The "brain" of the robot. It contains algorithms to make the robot move and stay in place (using methods like Station Keeping with PID control and Model Predictive Control - MPC).
3. **`my_auv_localization`**: Helps the robot figure out exactly where it is in the water by combining data from its sensors (like the IMU for rotation and DVL for speed).
4. **`auv_dvl_bridge`**: A translator that takes raw sensor data (from the DVL) and converts it into a format the robot's brain can understand.

## Prerequisites

Before starting, ensure you have:
- A Linux computer (Ubuntu is recommended).
- **ROS 2** installed (e.g., Foxy).
- **Gazebo** installed (the 3D simulation software).

## How to Use the Project

Follow these steps to launch the simulation and make the robot move.

### 1. Build the Workspace

First, we need to compile the code so the computer can run it. Open a terminal and run the following commands:

```bash
# Go to the project folder
cd ~/AUV_project/ros2_AUV

# Compile the code
colcon build

# Tell your terminal where to find the newly compiled programs
source install/setup.bash
```

*Note: You only need to run `colcon build` if you have changed the code. But you **always** need to run `source install/setup.bash` every time you open a new terminal.*

### 2. Launch the 3D Simulation

Now, let's start the virtual pool and put the robot inside it. In the same terminal, run:

- **Realistic Simulation** :
  ```bash
  ros2 launch AUV_description test_bluerov2_realistic.launch.py
  ```
- **Equipped with sensors** (classic):
  ```bash
  ros2 launch AUV_description bluerov2_bassin_captors.launch.py
  ```
- **Basic model** (no sensors):
  ```bash
  ros2 launch AUV_description bluerov2_bassin.launch.py
  ```
- **Realistic Simulation with waves** :
  ```bash
  ros2 launch AUV_description bluerov2_bassin_waves.launch.py
  ```
- **Deep Ocean Simulation** (40m depth + Net):
  ```bash
  ros2 launch AUV_description bluerov2_ocean_realistic.launch.py
  ```

Wait a few moments. A new window (Gazebo) will open showing a 3D pool with the BlueROV2 submarine floating inside. The robot's sensors are now active and gathering data.

### 3. Start the Controller (Make the robot move)

Right now, the robot is just floating. We need to give it a brain so it can maintain its position or move to a specific point.

Open a **new, second terminal** and run:

```bash
# Go to the project folder again
cd ~/AUV_project/ros2_AUV

# Source the configuration again (required for every new terminal)
source install/setup.bash
```

Here is the list of scripts you can launch to move the robot:

**1. Advanced Navigation Controllers:**
- **Station Keeping**: Makes the robot hold its current position.
  ```bash
  ros2 run AUV_controller station_keeping
  ```
- **MPC Realistic**: **[Optimized]** Advanced Model Predictive Control tuned for the realistic BlueROV2 model.
  ```bash
  ros2 run AUV_controller mpc_controller_realistic
  ```
- **MPC with Sensors**: Standard MPC using real sensor data (EKF odometry) to navigate.
  ```bash
  ros2 run AUV_controller mpc_controller_sensors
  ```
- **MPC with Exact Simulation Data**: Theoretical MPC subscribing directly to Gazebo's perfect odometry.
  ```bash
  ros2 run AUV_controller mpc_controller_bluerov
  ```

**2. Basic Open-Loop Movement (Testing):**
These scripts apply a constant force to test engines or validate physics:
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
