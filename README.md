# ROS 2 Autonomous Underwater Vehicle — BlueROV2 Net Inspection

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

---

## Table of Contents

1. [What this project does](#1-what-this-project-does)
2. [System Requirements & Installation](#2-system-requirements--installation)
3. [Quick Start — run in 3 commands](#3-quick-start--run-in-3-commands)
4. [Docker Deployment](#4-docker-deployment)
5. [Architecture Overview](#5-architecture-overview)
6. [Launch Arguments Reference](#6-launch-arguments-reference)
7. [Performance / Optimize Mode](#7-performance--optimize-mode)
8. [Visualizing with Foxglove Studio](#8-visualizing-with-foxglove-studio)
9. [Maintenance & Troubleshooting](#9-maintenance--troubleshooting)

---

## 1. What this project does

This workspace is the complete software stack for an autonomous **BlueROV2** underwater robot that inspects aquaculture fish-farm nets.

The robot:
1. **Dives** to a target depth.
2. **Searches** for the net using a 360° Ping360 sonar scan.
3. **Aligns and approaches** the net, stopping at a 1.5 m standoff.
4. **Orbits** the entire net perimeter, descending lap by lap to inspect the full depth of the structure.

Everything runs in **Gazebo Harmonic simulation** out of the box. Switching to the real BlueROV2 requires a single launch argument (`use_hardware:=True`).

### Sensor pipeline at a glance

```
Ping360 (2D sonar)  →  ping360_nearest      →  net orientation & initial yaw
Sonoptix (3D sonar) →  sonoptix_perception  →  net distance + normal vector (RANSAC)
DVL + IMU + Depth   →  EKF (robot_loc.)     →  /odometry/filtered
                                                      │
                             net_approach (Phase 2)  ◄┘
                             phase3_inspection (Phase 3)
                                      │
                             /auv/command_wrench
                                      │
                    sim_thruster_bridge ──► /cmd_vel_1…8 (Gazebo)
                    bluerov2_bridge    ──► MAVROS RC PWM (real hardware)
```

---

## 2. System Requirements & Installation

> **No ROS 2 installed?** Skip to [Section 4 — Docker](#4-docker-deployment). It is the easiest path.

### 2.1 Supported Platform

| Component | Version |
|---|---|
| OS | Ubuntu **24.04** LTS |
| ROS 2 | **Jazzy** Jalisco |
| Gazebo | **Harmonic** (8.x) |
| Python | 3.12 |

### 2.2 Install ROS 2 Jazzy

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-rosdep python3-colcon-common-extensions
```

### 2.3 Install Gazebo Harmonic

```bash
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
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-localization \
  ros-jazzy-foxglove-bridge \
  python3-numpy \
  libcgal-dev \
  libfftw3-dev

# First-time rosdep setup
sudo rosdep init
rosdep update

# Install all remaining ROS dependencies
cd ~/AUV_project/ros2_AUV
rosdep install --from-paths src --ignore-src -r -y
```

### 2.5 Auto-source ROS 2 in every terminal

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Quick Start — run in 3 commands

```bash
# 1. Clone and build
git clone https://github.com/titouanP56/auv-simulation.git ~/AUV_project/ros2_AUV
cd ~/AUV_project/ros2_AUV
colcon build

# 2. Source the workspace  ← do this in every new terminal
source install/setup.bash

# 3. Launch the full autonomous mission (small net)
ros2 launch AUV_guidance net_full_inspection.launch.py
```

Wait ~10 seconds for Gazebo to initialize. The robot will dive, find the net, and start orbiting automatically.

**Large net variant:**
```bash
ros2 launch AUV_guidance net_inspection_big_net.launch.py
```

**Faster physics — recommended on laptops:**
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py optimize:=True
```

**Real BlueROV2 hardware:**
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py use_hardware:=True
```

---

## 4. Docker Deployment

A `Dockerfile` is included at the workspace root. It ships the entire simulation stack as a portable container — no local ROS 2 or Gazebo install needed.

### Build the image

```bash
cd ~/AUV_project/ros2_AUV
docker build -t ros2_auv:latest .
```

First build takes 5–15 minutes. Subsequent builds use Docker's layer cache.

---

### 🐧 Linux (Ubuntu)

**Headless (test first):**
```bash
docker run --rm -it --net=host ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"
```

**With Gazebo window:**
```bash
xhost +local:docker
docker run --rm -it --net=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False"
```

---

### 🪟 Windows (WSL 2)

> Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) with WSL 2 integration enabled.

```bash
# Step 1 — verify Docker works from WSL
docker --version
# If you see "permission denied": sudo usermod -aG docker $USER, then wsl --shutdown

# Step 2 — build
cd ~/AUV_project/ros2_AUV && docker build -t ros2_auv:latest .

# Step 3a — headless
docker run --rm -it --net=host ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"

# Step 3b — with Gazebo GUI (via WSLg)
echo $DISPLAY   # should print ":0"
sudo -E docker run --rm -it --net=host \
  -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False"
```

---

### 📡 Foxglove Studio in Docker

```bash
# Terminal 1 — simulation
docker run --rm -it --net=host --name ros2_auv_sim ros2_auv:latest \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True"

# Terminal 2 — Foxglove bridge
docker exec -it ros2_auv_sim \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge"
```

Then open [Foxglove Studio](https://foxglove.dev/download) → **Open connection** → **Foxglove WebSocket** → `ws://localhost:8765`

---

## 5. Architecture Overview

The workspace is split into 6 ROS 2 packages. Each has its own `README.md` with full details.

```
┌──────────────────────────────────────────────────────┐
│                    SENSORS (Gazebo)                   │
│  Ping360 (2D)   Sonoptix (3D)   DVL   IMU   Depth   │
└──┬──────────────────┬─────────────────────┬───────────┘
   │                  │                     │
   ▼                  ▼                     ▼
ping360_nearest  sonoptix_perception   my_auv_localization
  (DBSCAN +       (RANSAC plane fit)   (EKF: DVL+IMU+Depth)
   full-scan)      PoseStamped out      /odometry/filtered
   net yaw out    net dist + normal
       │                  │                     │
       └──────────────────┴──────────────────── ┘
                          │
                   AUV_guidance
              ┌─────────────────────┐
              │ Phase 2: net_approach│  DESCENDING → GLOBAL_SEARCH
              │                     │  → ALIGNING → APPROACHING
              │                     │  → STABILIZING → STANDOFF
              │ Phase 3: inspection  │  WAITING → WALKING_THE_NET
              │                     │  → (LOST_WALL) → LAP_COMPLETED
              └─────────┬───────────┘
                        │ /auv/command_wrench
             ┌──────────┴──────────┐
             ▼                     ▼
    sim_thruster_bridge     bluerov2_bridge
    
    (Gazebo)                (real hardware)
```

### Package summary

| Package | Role |
|---|---|
| [`AUV_guidance`](src/AUV_guidance/README.md) | Mission state machines (Phase 2 & 3), thruster bridges |
| [`AUV_description`](src/AUV_description/README.md) | Robot URDF, Gazebo worlds, IMU/depth helper nodes |
| [`auv_perception`](src/auv_perception/README.md) | Ping360 net finder, Sonoptix RANSAC plane estimator |
| [`my_auv_localization`](src/my_auv_localization/README.md) | EKF configuration (DVL + IMU + Depth → odometry) |
| [`auv_dvl_bridge`](src/auv_dvl_bridge/README.md) | Gazebo DVL protobuf → ROS 2 TwistWithCovariance (C++) |
| [`AUV_controller`](src/AUV_controller/README.md) | ⚠️ Archived — MPC research, not used in current mission |

> **`asv_wave_sim`**: Third-party ocean wave plugin. Excluded from `colcon build` via `COLCON_IGNORE`. Not required for the main missions. See `src/asv_wave_sim/README.md` if you need wave simulation.

---

## 6. Launch Arguments Reference

Both `net_full_inspection.launch.py` and `net_inspection_big_net.launch.py` accept the same arguments:

| Argument | Default | Description |
|---|---|---|
| `headless` | `False` | Run Gazebo without the 3D GUI (saves GPU/CPU) |
| `use_hardware` | `False` | Deploy on the real BlueROV2 (disables Gazebo, starts MAVROS) |
| `rviz` | `False` | Open RViz2 for TF/sensor visualization |
| `world_file` | see below | Gazebo world file (in `AUV_description/world/`) |
| `gz_delay` | `8.0` | Seconds to wait for Gazebo before spawning nodes. Increase on slow machines. |
| `optimize` | `False` | Performance mode — coarser physics, slower loops. See [Section 7](#7-performance--optimize-mode). |

### Available world files

| World file | Net size | Use with |
|---|---|---|
| `small_net.xml` | Small (radius ≈ 3.4 m) | `net_full_inspection.launch.py` (default) |
| `ocean_40m.xml` | Large (radius ≈ 20 m, depth 40 m) | `net_inspection_big_net.launch.py` (default) |
| `bluerov2_bassin.xml` | No net — empty pool | Basic PID tuning |
| `Bassin_ntnu_waves.xml` | Pool with waves | Requires `gz-waves` built manually |

### Example commands

```bash
# Default — small net with GUI
ros2 launch AUV_guidance net_full_inspection.launch.py

# Headless + fast physics (CI / low-end laptop)
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True

# With RViz2
ros2 launch AUV_guidance net_full_inspection.launch.py rviz:=True

# Large net
ros2 launch AUV_guidance net_inspection_big_net.launch.py headless:=False

# Real hardware
ros2 launch AUV_guidance net_full_inspection.launch.py use_hardware:=True
```

---

## 7. Performance / Optimize Mode

The `optimize` argument lets you trade physics fidelity for simulation speed — no source file is ever modified.

| Parameter | Normal (`False`) | Optimized (`True`) |
|---|---|---|
| Gazebo physics step | 1 ms | 6 ms |
| URDF sensor rates | Full | Reduced (via Xacro flag) |
| Control loop rate | 20 Hz | 5 Hz |
| Yaw EMA filter α | 1.0 (raw) | 0.15 (smoothed) |

The world file patch is applied **in memory only** (written to `/tmp`) — the `.xml` files in `AUV_description/world/` are never touched.

> Use `optimize:=True` for headless CI runs or slow laptops.
> Use `optimize:=False` (default) for demos, final validation, or whenever sensor timing matters.

---

## 8. Visualizing with Foxglove Studio

[Foxglove Studio](https://foxglove.dev/) gives you real-time plots, 3D views, and topic inspection from any browser — no ROS install needed on the viewer machine.

**Step 1 — Install the bridge (if not already):**
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

**Step 2 — Run the bridge while the simulation is running:**
```bash
ros2 run foxglove_bridge foxglove_bridge
```

**Step 3 — Connect:**
1. Open [Foxglove Studio](https://foxglove.dev/download)
2. **Open connection** → **Foxglove WebSocket** → `ws://localhost:8765`

**Useful topics to monitor:**

| Topic | Content |
|---|---|
| `/mission/phase` | Current mission state (DESCENDING, ALIGNING…) |
| `/odometry/filtered` | Robot position & velocity (EKF output) |
| `/sonoptix/perception` | Distance to net + normal vector |
| `/phase3/wall_distance` | Raw sonar distance |
| `/phase3/wall_distance_smoothed` | EMA-filtered distance |
| `/phase3/yaw_error` | Angular alignment error |
| `/phase3/yaw_accumulated` | Total yaw turned (lap tracking) |
| `/phase3/real_time_factor` | Simulation RTF |

---

## 9. Maintenance & Troubleshooting

### Common errors

| Error | Solution |
|---|---|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` (or add to `~/.bashrc`) |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
| `Package 'AUV_guidance' not found` | Run `source install/setup.bash` after building |
| Gazebo is very slow | Use `optimize:=True` or `headless:=True`; check GPU drivers |
| Robot shoots out of water at spawn | Physics glitch — `Ctrl+C` and relaunch |
| `rosdep install` fails | Run `rosdep update` first; check all `package.xml` files are present |
| No display in Docker | Run `xhost +local:docker`; verify `$DISPLAY` is set |
| `gz-waves` build failure | `gz-waves` is excluded via `COLCON_IGNORE`. Not needed for main missions. |
| MAVROS won't connect | Check `fcu_url` in the launch file matches your BlueROV2 IP and port |

### Partial builds (useful during development)

```bash
# Rebuild only one package and its dependencies
colcon build --packages-up-to AUV_guidance

# Rebuild a single package
colcon build --packages-select AUV_description

# Verbose output (debug build errors)
colcon build --packages-select auv_dvl_bridge --event-handlers console_direct+
```

### Key ROS 2 debug commands

```bash
# List all active topics
ros2 topic list

# Watch the mission state
ros2 topic echo /mission/phase

# Watch robot position
ros2 topic echo /odometry/filtered --once

# Watch net distance from the sonar
ros2 topic echo /sonoptix/perception

# Inspect the TF tree
ros2 run tf2_tools view_frames

# List parameters of a node
ros2 param list /net_approach
ros2 param list /phase3_inspection
```

### Key constants to tune

| File | Constant | Effect |
|---|---|---|
| `phase3_inspection.py` | `STANDOFF_DIST` | Distance from net surface (default 1.5 m) |
| `phase3_inspection.py` | `ORBIT_DIRECTION` | +1 = counter-clockwise, -1 = clockwise |
| `phase3_inspection.py` | `DEPTH_STEP` | Depth decrement per lap (default 0.5 m) |
| `phase3_inspection.py` | `FINAL_DEPTH_LIMIT` | Stop depth (small net: −6 m, big net: −29.5 m) |
| `net_approach.py` | `STANDOFF_DIST` | Same — standoff for approach phase |
| `net_approach.py` | `TARGET_DEPTH` | Initial dive target (default −2 m) |
| `sonoptix_perception.py` | `ransac_distance_threshold` | RANSAC inlier tolerance [m] |
