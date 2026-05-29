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

> **Performance Mode:** On a low-end machine or in a headless CI environment, add `optimize:=True` to reduce the simulation load. This coarsens the physics timestep (1 ms → 6 ms), lowers sensor update rates, and drops the control loops to 5 Hz — making the simulation run much faster at the cost of some physical fidelity. See [Section 5](#5-performance--optimize-mode) for details.

```bash
# Example: headless + optimized (ideal for CI or low-end hardware)
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True
```

### Visualizing with Foxglove
To see what the robot sees (cameras, sonar, 3D position) in real-time using [Foxglove Studio](https://foxglove.dev/):
1. Install the bridge if needed: `sudo apt install ros-jazzy-foxglove-bridge` (replace `jazzy` with your version).
2. Run the bridge in a sourced terminal: `ros2 run foxglove_bridge foxglove_bridge`
3. Open Foxglove Studio, click "Open connection", select "Foxglove WebSocket", and connect to `ws://localhost:8765` (or your robot's IP).

---

## 3. Docker Deployment

A `Dockerfile` and a `.dockerignore` are provided at the root of this workspace to package the entire simulation environment into a portable container. This removes the need to install ROS 2 or Gazebo locally and makes it easy to share the setup with the research team.

### Prerequisites
- [Docker](https://docs.docker.com/engine/install/) installed and running (`docker --version`).
- For GUI (Gazebo window): an X11 server must be available on the host (standard on Ubuntu desktops, built-in via WSLg on Windows).

### Building the image
From the root of the workspace, run:
```bash
cd ~/AUV_project/ros2_AUV
docker build -t ros2_auv:latest .
```
The build will:
1. Pull `osrf/ros:jazzy-desktop` as base.
2. Install system packages (`python3-numpy`, `ros-jazzy-ros-gz`, `ros-jazzy-robot-localization`, `libcgal-dev`, `libfftw3-dev` …).
3. Run `rosdep install` for all ROS dependencies declared in `package.xml` files.
4. Compile the workspace with `colcon build --symlink-install`.

Build time is typically **5–15 minutes** on first run (network-dependent). Subsequent builds use Docker layer cache and are much faster.

---

### 🐧 Running on Linux (Ubuntu desktop)

**Headless simulation (no GUI — ideal for CI/servers or low-end machines):**
```bash
docker run --rm -it \
  --net=host \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**With Gazebo GUI (3D window):**
```bash
# 1. Allow Docker to open windows on your screen (run once per session)
xhost +local:docker

# 2. Launch with the graphical interface
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False"
```

---

### 🪟 Running on Windows (via WSL 2)

> **Important:** Windows itself cannot run Docker or ROS 2 natively. You must use **WSL 2** (Windows Subsystem for Linux) with Ubuntu. [Docker Desktop](https://www.docker.com/products/docker-desktop/) must be installed and WSL 2 integration must be enabled in its settings.

**Step 1 — Open a fresh WSL 2 Ubuntu terminal** from Windows Terminal or the Start menu.

Verify Docker works:
```bash
docker --version
# If you see "permission denied", run the following and reopen your terminal:
sudo usermod -aG docker $USER
# Then close ALL WSL terminals and run in PowerShell: wsl --shutdown
```

**Step 2 — Clone the repo and build the image (once)**
```bash
git clone https://github.com/your-account/ros2_AUV.git
cd ros2_AUV
docker build -t ros2_auv:latest .
```

**Step 3a — Headless simulation (recommended to test first):**
```bash
docker run --rm -it \
  --net=host \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**Step 3b — With Gazebo GUI on Windows (via WSLg):**

WSL 2 uses **WSLg** to display Linux GUI applications directly in Windows. You must start from a **fresh WSL terminal** (not via `su -`) so that the `$DISPLAY` variable is correctly inherited.

```bash
# Verify this returns ":0" before continuing
echo $DISPLAY

# Launch with GUI (use sudo -E to preserve the DISPLAY variable)
sudo -E docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False"
```

> **Why `sudo -E`?** On WSL 2, your user may need `sudo` to access Docker. The `-E` flag tells `sudo` to keep your environment variables (including `$DISPLAY`) so the GUI can work.

---

### 📡 Visualizing with Foxglove Studio

[Foxglove Studio](https://foxglove.dev/) lets you visualize robot sensors in real-time (cameras, sonar, 3D position, etc.) from any browser or desktop app, without needing ROS installed.

**Step 1 — Launch the simulation with a name:**
```bash
# Works on both Linux and Windows/WSL2 — add --name to identify the container
docker run --rm -it \
  --net=host \
  --name ros2_auv_sim \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**Step 2 — In a second terminal, start the Foxglove bridge inside the container:**
```bash
docker exec -it ros2_auv_sim \
  bash -c "apt-get install -y -q ros-jazzy-foxglove-bridge && \
  source /opt/ros/jazzy/setup.bash && \
  ros2 run foxglove_bridge foxglove_bridge"
```

**Step 3 — Connect Foxglove Studio (Windows, macOS, or Linux):**
1. Download and open [Foxglove Studio](https://foxglove.dev/download).
2. Click **"Open connection"**.
3. Select **"Foxglove WebSocket"**.
4. Enter: `ws://localhost:8765`
5. Click **"Open"** — all robot topics will appear in the panel.

---

### 🔧 Interactive shell (development / debugging)
```bash
docker run --rm -it ros2_auv:latest bash
# source /opt/ros/jazzy/setup.bash  ← done automatically via ~/.bashrc
# source /ros2_ws/install/setup.bash ← done automatically via ~/.bashrc
```

### What `.dockerignore` excludes
The `build/`, `install/` and `log/` directories created by `colcon` locally are excluded from the Docker build context. This keeps the image clean and avoids conflicts between your host build artifacts and the in-container build.

---

## 4. Technical Architecture

This workspace is highly modular, separated into specialized ROS 2 packages. **For detailed information on any of these components, please read the specific `README.md` located inside each package folder.**

1. **`AUV_guidance`** (The Brain): Handles the high-level mission state machine (Approach, Standoff, Inspect). It uses reactive PID controllers to orbit the net and maintain distance, depth, and orientation.
2. **`AUV_description`** (The Body & Environment): Contains the URDF 3D models, sensor plugins, and the Gazebo underwater world files (`.xml`).
3. **`auv_perception`** (The Eyes & Memory): Filters raw Sonoptix point clouds, estimates the net's orientation via PCA line-fitting, and handles saving 3D OctoMaps.
4. **`my_auv_localization`** (The Inner Ear): Fuses DVL, IMU, and Depth data through an Extended Kalman Filter (EKF) to provide a smooth, reliable 6-DOF odometry.
5. **`auv_dvl_bridge`** (The Translator): Converts raw Gazebo DVL protobuf messages into standard ROS 2 Twist formats with covariance for the EKF.
6. **`AUV_controller`** *(Archived/Research)*: Contains historical Model Predictive Control (MPC) algorithms. Currently bypassed in favor of the reactive PIDs in `AUV_guidance`.

---

## 5. Performance / Optimize Mode

Both mission launch files (`net_full_inspection.launch.py` and `net_inspection_big_net.launch.py`) expose an `optimize` argument that significantly reduces simulation load **without modifying any source file**.

| Parameter | Normal (`optimize:=False`) | Optimized (`optimize:=True`) |
|---|---|---|
| Gazebo physics step | 1 ms | 6 ms |
| URDF sensor rates | Full | Reduced (via Xacro flag) |
| Control loop rate | 20 Hz | 5 Hz |
| Yaw EMA filter | α = 1.0 (no smoothing) | α = 0.15 (smoothed) |

The world file patch is applied **in memory only** (written to a temp file) — the original `.xml` files in `AUV_description/world/` are never touched.

> Refer to the individual package READMEs for full details:
> - [`AUV_guidance/README.md` → Section 5](src/AUV_guidance/README.md)
> - [`AUV_description/README.md` → Section 5](src/AUV_description/README.md)

---

## 6. Maintenance & Troubleshooting

- **"Command not found: colcon"** or **"ros2: command not found"**: You forgot to source your main ROS 2 installation. Run `source /opt/ros/jazzy/setup.bash` (replace `jazzy` with your version).
- **Gazebo is very slow**: 3D simulations require a decent graphics card. If it's too slow, make sure your computer is plugged in and using its dedicated GPU. You can also run in performance mode with `optimize:=True`, or hide the graphical window entirely with `headless:=True`.
- **The robot spins or flies out of the water**: Physics simulations can sometimes glitch upon spawning the robot. If this happens, close the terminals (using `Ctrl+C`) and start the launch again.
- **Docker build fails on `rosdep install`**: Make sure the `src/` folder contains all required packages and their `package.xml` files before building the image. Run `rosdep update` on your host first if needed.
- **No display in Docker**: When running the container with a GUI, ensure you have run `xhost +local:docker` beforehand and that `$DISPLAY` is set correctly on the host.
