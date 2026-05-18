# ROS 2 Autonomous Underwater Vehicle (AUV) Project

## 1. Introduction for Beginners

Welcome to the ROS 2 AUV project! This workspace contains the complete software stack needed to simulate and control an intelligent underwater robot (specifically based on the BlueROV2).

Imagine this project as a complete digital brain and training ground for a submarine:
- It provides a **virtual ocean** (simulation) where the robot can safely swim.
- It gives the robot **eyes and ears** (perception) to see structures like aquaculture nets.
- It tells the robot **where it is** (localization) without needing GPS.
- It provides the **intelligence** (guidance) to make autonomous decisions, like finding a net, approaching it, and performing a 360° cyclic inspection.

Whether you are running the mission purely on your computer or deploying the code to a real-world BlueROV2, this workspace provides all the necessary tools.

---

## 2. Quick Start Guide

### Prerequisites
Before starting, ensure your system has:
- A Linux operating system (Ubuntu is highly recommended).
- **ROS 2** installed (e.g., Jazzy or Humble).
- **Gazebo Harmonic** installed (for the 3D physics simulation).

### Building the Workspace
First, compile the code so the system can run it. Open a terminal and run:

```bash
cd ~/AUV_project/ros2_AUV
colcon build
source install/setup.bash
```
*(Note: You must run `source install/setup.bash` every time you open a new terminal.)*

### Launching the Autonomous Mission
The main showcase of this project is the fully autonomous net inspection mission. This single command starts the ocean simulation, the robot, the sensors, and the autonomous intelligence:

```bash
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False
```

> **Hardware Deployment:** To run this exact same mission on the real BlueROV2 instead of the simulator, simply append `use_hardware:=True` to the launch command.

### Visualizing with Foxglove
To see what the robot sees (cameras, sonar, 3D position) in real-time using [Foxglove Studio](https://foxglove.dev/):
1. Install the bridge if needed: `sudo apt install ros-jazzy-foxglove-bridge` (replace `jazzy` with your version).
2. Run the bridge in a sourced terminal: `ros2 run foxglove_bridge foxglove_bridge`
3. Open Foxglove Studio, click "Open connection", select "Foxglove WebSocket", and connect to `ws://localhost:8765` (or your robot's IP).

---

## 3. Technical Architecture

This workspace is highly modular, separated into specialized ROS 2 packages. **For detailed information on any of these components, please read the specific `README.md` located inside each package folder.**

1. **`AUV_guidance`** (The Brain): Handles the high-level mission state machine (Approach, Standoff, Inspect). It uses reactive PID controllers to orbit the net and maintain distance, depth, and orientation.
2. **`AUV_description`** (The Body & Environment): Contains the URDF 3D models, sensor plugins, and the Gazebo underwater world files (`.xml`).
3. **`auv_perception`** (The Eyes & Memory): Filters raw Sonoptix point clouds, estimates the net's orientation via PCA line-fitting, and handles saving 3D OctoMaps.
4. **`my_auv_localization`** (The Inner Ear): Fuses DVL, IMU, and Depth data through an Extended Kalman Filter (EKF) to provide a smooth, reliable 6-DOF odometry.
5. **`auv_dvl_bridge`** (The Translator): Converts raw Gazebo DVL protobuf messages into standard ROS 2 Twist formats with covariance for the EKF.
6. **`AUV_controller`** *(Archived/Research)*: Contains historical Model Predictive Control (MPC) algorithms. Currently bypassed in favor of the reactive PIDs in `AUV_guidance`.

---

## 4. Maintenance & Troubleshooting

- **"Command not found: colcon"** or **"ros2: command not found"**: You forgot to source your main ROS 2 installation. Run `source /opt/ros/jazzy/setup.bash` (replace `jazzy` with your version).
- **Gazebo is very slow**: 3D simulations require a decent graphics card. If it's too slow, make sure your computer is plugged in and using its dedicated GPU. You can also run the mission without the graphical window by adding `headless:=True` to your launch command.
- **The robot spins or flies out of the water**: Physics simulations can sometimes glitch upon spawning the robot. If this happens, close the terminals (using `Ctrl+C`) and start the launch again.
