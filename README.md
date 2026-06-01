# ROS 2 Autonomous Underwater Vehicle (AUV) Project

> **Tested environment:** ROS 2 **Jazzy** + Gazebo **Harmonic** on Ubuntu 24.04 LTS.

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Requirements & Installation](#2-system-requirements--installation)
3. [Quick Start Guide](#3-quick-start-guide)
4. [Docker Deployment](#4-docker-deployment)
5. [Architecture Overview](#5-architecture-overview)
6. [Launch Arguments Reference](#6-launch-arguments-reference)
7. [Performance / Optimize Mode](#7-performance--optimize-mode)
8. [Visualizing with Foxglove Studio](#8-visualizing-with-foxglove-studio)
9. [Maintenance & Troubleshooting](#9-maintenance--troubleshooting)

---

## 1. Introduction

This workspace contains the complete software stack to **simulate and autonomously control a BlueROV2 underwater robot** performing aquaculture net inspection missions.

Think of this project as a complete digital brain and training ground for a submarine:
- It provides a **virtual ocean** (Gazebo Harmonic simulation) where the robot can safely swim.
- It gives the robot **eyes and ears** (Ping360 + Sonoptix sonar sensors) to detect aquaculture net structures.
- It tells the robot **where it is** (EKF localization from DVL + IMU + depth) without GPS.
- It provides the **intelligence** (reactive PID guidance) to autonomously find a net, approach it, and perform a 360° cyclic inspection.

The same code can be deployed on a **real BlueROV2** by switching a single launch argument (`use_hardware:=True`).

---

## 2. System Requirements & Installation

> **If you just want to run the project without installing ROS 2 locally, skip to [Section 4 — Docker](#4-docker-deployment).** Docker is the easiest path.

### 2.1 Supported Platform

| Component | Version |
|---|---|
| Operating System | Ubuntu **24.04** LTS |
| ROS 2 | **Jazzy** Jalisco |
| Gazebo | **Harmonic** (8.x) |
| Python | 3.12 (bundled with Ubuntu 24.04) |

### 2.2 Install ROS 2 Jazzy

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), or use the commands below:

```bash
# Set up the ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-rosdep python3-colcon-common-extensions
```

### 2.3 Install Gazebo Harmonic

```bash
# Add the Gazebo apt repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic
```

### 2.4 Install Project Dependencies

```bash
# ROS 2 packages required by this project
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-localization \
  ros-jazzy-foxglove-bridge \
  python3-numpy \
  libcgal-dev \
  libfftw3-dev

# Initialize rosdep (first-time only)
sudo rosdep init
rosdep update

# Install all remaining ROS dependencies declared in package.xml files
cd ~/AUV_project/ros2_AUV
rosdep install --from-paths src --ignore-src -r -y
```

### 2.5 Source ROS 2 automatically (recommended)

Add the following line to your `~/.bashrc` so every new terminal is ready:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Quick Start Guide

### Step 1 — Clone and build

```bash
git clone https://github.com/titouanP56/auv-simulation.git ~/AUV_project/ros2_AUV
cd ~/AUV_project/ros2_AUV
colcon build
source install/setup.bash
```

> **Important:** You must run `source install/setup.bash` in **every new terminal** you open, or add it to `~/.bashrc`.

### Step 2 — Launch the autonomous mission

This single command starts the ocean simulation, spawns the BlueROV2, activates all sensors, and runs the full autonomous net-inspection mission:

```bash
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False
```

Wait ~10 seconds for Gazebo to initialize, then watch the robot dive and approach the net.

**Hardware deployment:** To run the same mission on the real BlueROV2 instead of the simulator:
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py use_hardware:=True
```

**Low-end machine / CI (no GUI, faster physics):**
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True
```

---

## 4. Docker Deployment

A `Dockerfile` is provided at the root of this workspace. It packages the **entire simulation environment** into a portable container — no need to install ROS 2 or Gazebo locally.

### Prerequisites

- [Docker](https://docs.docker.com/engine/install/) installed (`docker --version`)
- For GUI (Gazebo window): an X11 server on the host (standard on Ubuntu, built-in via WSLg on Windows)

### Build the image

```bash
cd ~/AUV_project/ros2_AUV
docker build -t ros2_auv:latest .
```

The build will:
1. Pull `osrf/ros:jazzy-desktop` as base image.
2. Install system packages (`python3-numpy`, `ros-jazzy-ros-gz`, `ros-jazzy-robot-localization`, `libcgal-dev`, `libfftw3-dev` …).
3. Run `rosdep install` for all ROS dependencies.
4. Compile the workspace with `colcon build --symlink-install`.

Build time is typically **5–15 minutes** on first run. Subsequent builds use Docker layer cache.

---

### 🐧 Linux (Ubuntu desktop)

**Headless — recommended to test first:**
```bash
docker run --rm -it \
  --net=host \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**With Gazebo GUI:**
```bash
# Allow Docker to open windows on your screen (once per session)
xhost +local:docker

docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False"
```

---

### 🪟 Windows (via WSL 2)

> **Important:** Windows cannot run Docker or ROS 2 natively. You must use **WSL 2** (Windows Subsystem for Linux) with Ubuntu. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and enable WSL 2 integration in its settings.

**Step 1 — Open a WSL 2 Ubuntu terminal** and verify Docker works:
```bash
docker --version
# If you see "permission denied":
sudo usermod -aG docker $USER
# Then close ALL WSL terminals and run in PowerShell: wsl --shutdown
```

**Step 2 — Clone the repo and build:**
```bash
git clone <YOUR_REPO_URL> ros2_AUV
cd ros2_AUV
docker build -t ros2_auv:latest .
```

**Step 3a — Headless simulation (recommended first):**
```bash
docker run --rm -it \
  --net=host \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**Step 3b — With Gazebo GUI (via WSLg):**

WSL 2 uses **WSLg** to display Linux GUI applications in Windows. Start from a **fresh WSL terminal** so `$DISPLAY` is correctly set.

```bash
# Verify this returns ":0" before continuing
echo $DISPLAY

# Launch with GUI (sudo -E preserves the DISPLAY variable)
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

---

### 📡 Foxglove Studio in Docker

**Step 1 — Launch simulation with a named container:**
```bash
docker run --rm -it \
  --net=host \
  --name ros2_auv_sim \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**Step 2 — Start the Foxglove bridge (second terminal):**
```bash
docker exec -it ros2_auv_sim \
  bash -c "source /opt/ros/jazzy/setup.bash && \
  ros2 run foxglove_bridge foxglove_bridge"
```

**Step 3 — Connect:** Open [Foxglove Studio](https://foxglove.dev/download) → **Open connection** → **Foxglove WebSocket** → `ws://localhost:8765`

---

### 🔧 Interactive shell (development / debugging)
```bash
docker run --rm -it ros2_auv:latest bash
# Both ROS 2 and the workspace are sourced automatically via ~/.bashrc
```

### What `.dockerignore` excludes
The `build/`, `install/`, and `log/` directories (created by `colcon` locally) are excluded from the Docker build context to keep the image clean.

---

## 5. Architecture Overview

This workspace is modular, with 6 specialized ROS 2 packages. **For full details, read the `README.md` inside each package folder.**

```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSORS (Gazebo)                         │
│  Ping360 Sonar   Sonoptix 3D Sonar   DVL   IMU   Depth Sensor  │
└────┬──────────────────┬───────────────┬─────┬──────────┬────────┘
     │                  │               │     │          │
     ▼                  ▼               ▼     ▼          ▼
┌──────────┐   ┌─────────────────┐  ┌──────────────────────────┐
│AUV_desc  │   │  auv_perception │  │    my_auv_localization   │
│(bridges, │   │  sonar filter   │  │  EKF (DVL + IMU + Depth) │
│ depth,   │   │  net estimator  │  │  → /odometry/filtered    │
│ IMU fix) │   │  /net_local_    │  └──────────────┬───────────┘
└────┬─────┘   │  frame          │                 │
     │         └────────┬────────┘                 │
     │                  │                          │
     └──────────────────┴──────────────────────────┘
                        │ /odometry/filtered
                        │ /ping360/scan
                        │ /sonoptix/points_filtered
                        │ /perception/net_local_frame
                        ▼
              ┌──────────────────┐
              │   AUV_guidance   │  ← The Mission Brain
              │  phase2: approach│
              │  phase3: inspect │
              │  → /auv/command_ │
              │    wrench        │
              └────────┬─────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
  ┌──────────────┐         ┌─────────────────┐
  │ sim_thruster │         │ bluerov2_bridge  │
  │ _bridge      │         │ (real hardware)  │
  │ /cmd_vel_1-8 │         │ MAVROS RC PWM    │
  └──────────────┘         └─────────────────┘
```

### Package Summary

| Package | Role | Key Tech |
|---|---|---|
| [`AUV_guidance`](src/AUV_guidance/README.md) | Mission brain (approach + inspection state machine, thruster bridges) | Python, PID controllers |
| [`AUV_description`](src/AUV_description/README.md) | Robot body, sensors, simulation worlds | URDF/Xacro, Gazebo SDF |
| [`auv_perception`](src/auv_perception/README.md) | Sonar filtering, net pose estimation, OctoMap autosave | Python, NumPy, PCA |
| [`my_auv_localization`](src/my_auv_localization/README.md) | EKF sensor fusion (DVL + IMU + Depth) | `robot_localization`, YAML |
| [`auv_dvl_bridge`](src/auv_dvl_bridge/README.md) | Gazebo DVL protobuf → ROS 2 TwistWithCovariance | C++, gz-transport |
| [`AUV_controller`](src/AUV_controller/README.md) | *(Archived)* MPC and station-keeping research | Python, CasADi |

> **Note on `asv_wave_sim`:** This third-party package (gz-waves) provides ocean wave simulation. It is **not required** to run the main net inspection missions (`small_net.xml`, `ocean_40m.xml`). It is only used by the optional `Bassin_ntnu_waves.xml` world. The `gz-waves` sub-package is excluded from `colcon build` via a `COLCON_IGNORE` file to avoid compilation failures on machines that do not have all its optional dependencies. If you need wave simulation, see `src/asv_wave_sim/README.md`.

---

## 6. Launch Arguments Reference

Both main launch files (`net_full_inspection.launch.py` and `net_inspection_big_net.launch.py`) accept the following arguments:

| Argument | Default | Description |
|---|---|---|
| `headless` | `False` | Run Gazebo without the 3D GUI window. Saves GPU/CPU resources. |
| `use_hardware` | `False` | Deploy on the **real BlueROV2**. Disables Gazebo and starts MAVROS + `bluerov2_bridge`. |
| `rviz` | `False` | Launch RViz2 to visualize TF trees, sensor data, and point clouds. |
| `world_file` | `small_net.xml` | Gazebo world to load (files located in `AUV_description/world/`). |
| `gz_delay` | `8.0` | Seconds to wait for Gazebo to finish loading before spawning the robot. Increase on slow machines. |
| `optimize` | `False` | **Performance mode**: reduces physics fidelity for faster simulation. See [Section 7](#7-performance--optimize-mode). |

### Available world files

| World file | Description |
|---|---|
| `small_net.xml` | Small aquaculture net (radius ≈ 3.4 m) — **default** |
| `ocean_40m.xml` | Large net (radius ≈ 20 m), use with `net_inspection_big_net.launch.py` |
| `bluerov2_bassin.xml` | Empty pool, useful for basic PID tuning |
| `Bassin_ntnu_waves.xml` | Pool with waves (requires `gz-waves` compiled separately) |

### Example commands

```bash
# Default: simulation with GUI
ros2 launch AUV_guidance net_full_inspection.launch.py

# Headless + optimized (CI, low-end laptop)
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True

# With RViz2 visualization
ros2 launch AUV_guidance net_full_inspection.launch.py rviz:=True

# Large net world
ros2 launch AUV_guidance net_inspection_big_net.launch.py headless:=False

# Real hardware
ros2 launch AUV_guidance net_full_inspection.launch.py use_hardware:=True
```

---

## 7. Performance / Optimize Mode

Both mission launch files expose an `optimize` argument that **significantly reduces simulation load without modifying any source file**.

| Parameter | Normal (`optimize:=False`) | Optimized (`optimize:=True`) |
|---|---|---|
| Gazebo physics step | 1 ms | 6 ms |
| URDF sensor rates | Full | Reduced (via Xacro flag) |
| Control loop rate | 20 Hz | 5 Hz |
| Yaw EMA filter α | 1.0 (no smoothing) | 0.15 (smoothed) |

The world file physics patch is applied **in memory only** (written to a `/tmp` file) — the original `.xml` files in `AUV_description/world/` are never modified.

> Refer to the individual package READMEs for full details:
> - [`AUV_guidance/README.md` → Section 5](src/AUV_guidance/README.md)
> - [`AUV_description/README.md` → Section 5](src/AUV_description/README.md)

---

## 8. Visualizing with Foxglove Studio

[Foxglove Studio](https://foxglove.dev/) lets you visualize robot sensors in real-time (cameras, sonar, 3D position) from any browser or desktop app, without needing ROS installed on the viewing machine.

**Step 1 — Install the bridge (if not already installed):**
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

**Step 2 — Run the bridge in a sourced terminal (while the simulation is running):**
```bash
ros2 run foxglove_bridge foxglove_bridge
```

**Step 3 — Connect Foxglove Studio:**
1. Download and open [Foxglove Studio](https://foxglove.dev/download).
2. Click **"Open connection"**.
3. Select **"Foxglove WebSocket"**.
4. Enter: `ws://localhost:8765` (or your robot's IP if connecting remotely).
5. Click **"Open"** — all robot topics will appear in the panel.

---

## 9. Maintenance & Troubleshooting

### Common errors

| Error | Solution |
|---|---|
| `ros2: command not found` | Run `source /opt/ros/jazzy/setup.bash` or add it to `~/.bashrc` |
| `colcon: command not found` | Run `sudo apt install python3-colcon-common-extensions` |
| `Package 'AUV_guidance' not found` | You forgot to `source install/setup.bash` after building |
| Gazebo is very slow | Use `optimize:=True` or `headless:=True`; ensure your GPU drivers are active |
| Robot spins / flies out of water | Physics glitch at spawn — press `Ctrl+C` and relaunch |
| `rosdep install` fails | Make sure all `package.xml` files are present in `src/`; run `rosdep update` first |
| No display in Docker | Run `xhost +local:docker` before launching; verify `$DISPLAY` is set |
| `gz-waves` build failure | The `gz-waves` package is excluded from `colcon build` via `COLCON_IGNORE`. Wave simulation is not needed for the main missions. If you need it, see `src/asv_wave_sim/README.md` and build it manually. |

### Partial builds (useful for development)

```bash
# Build only one package and its dependencies
colcon build --packages-up-to AUV_guidance

# Build a single package (no dependencies)
colcon build --packages-select AUV_description

# Build with verbose output to debug errors
colcon build --packages-select auv_dvl_bridge --event-handlers console_direct+
```

### Useful ROS 2 debug commands

```bash
# List all active topics
ros2 topic list

# Check the robot's estimated position (EKF output)
ros2 topic echo /odometry/filtered

# Check the current mission phase
ros2 topic echo /mission/phase

# Inspect the TF tree
ros2 run tf2_tools view_frames

# Check a node's active parameters
ros2 param list /net_approach
```
