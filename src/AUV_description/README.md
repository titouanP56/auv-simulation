# AUV Description

## 1. Introduction for Beginners

Welcome to the **AUV Description** package! This package defines the "body" and "senses" of our robot inside the virtual world (Gazebo). 

Imagine building a video game character: you need to define its shape, how heavy it is, where its eyes (cameras) and ears (sonars) are located, and how it interacts with water. That is exactly what this package does. It holds the 3D models (meshes), the physical properties (weight, buoyancy), and the sensors (IMU, Sonar, Depth) for the BlueROV2. It also contains the simulated "worlds" (like a pool or the ocean with a fishing net) where the robot will dive.

Additionally, this package provides small helper scripts that take raw, perfect data from the simulation and make it "messy" and realistic, so our robot's brain has to work just as hard in simulation as it would in the real ocean!

---

## 2. Quick Start Guide

### Prerequisites
Make sure your ROS 2 workspace is sourced and built.

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_description
source install/setup.bash
```

### Launching the Simulated Environments

This package primarily contains launch files to spawn the robot in different simulated worlds.

**1. Basic Pool Environment:**
A simple, empty pool useful for testing basic movements and PID tuning.
```bash
ros2 launch AUV_description bluerov2_bassin.launch.py
```

**2. Realistic Pool with Sensors:**
Spawns the robot with all its sensors active (Sonar, IMU, Depth) in the pool.
```bash
ros2 launch AUV_description bluerov2_bassin_captors.launch.py
```

**3. Pool with Waves:**
Adds surface waves to the pool to test stability and station keeping.
```bash
ros2 launch AUV_description bluerov2_bassin_waves.launch.py
```

**4. Ocean Environment (Net Inspection):**
Spawns the robot in an open ocean environment featuring a large aquaculture net.
```bash
ros2 launch AUV_description bluerov2_ocean_realistic.launch.py
```

> **Note:** The launch files in `AUV_guidance` also accept an `optimize:=True` flag that dynamically patches the physics step size of the world file before loading it in Gazebo. See the [Performance / Optimize Mode](#5-performance--optimize-mode) section below for details.

---

## 3. Technical Architecture

This package uses standard URDF/Xacro and SDF formats to define the robot and environments for Gazebo Harmonic. It also includes two critical Python nodes to bridge the gap between ideal simulation data and realistic sensor noise.

### Core Nodes

1. **`simulated_depth_sensor`**: 
   - **Role**: Simulates a realistic pressure/depth sensor.
   - **Logic**: Subscribes to the perfect Gazebo odometry, extracts the exact Z coordinate, injects Gaussian noise ($\sigma = 2$ cm), and publishes it.
   - **Output**: Publishes a `PoseWithCovarianceStamped` where only the Z-axis has a valid covariance matrix entry.

2. **`imu_republisher`**:
   - **Role**: Fixes Gazebo Harmonic IMU messages.
   - **Logic**: Gazebo publishes IMU data with zero-filled covariance matrices. An Extended Kalman Filter (EKF) will interpret a zero covariance as "absolute perfection" and fail. This node intercepts the message and injects realistic variance values (derived from URDF noise parameters) into the covariance matrices before republishing.

### Subscribed Topics
- `/odom` (`nav_msgs/Odometry`): Exact simulation odometry (used by depth sensor).
- `/imu` (`sensor_msgs/Imu`): Raw, zero-covariance IMU data from Gazebo.

### Published Topics
- `/depth/pose` (`geometry_msgs/PoseWithCovarianceStamped`): Noisy Z-axis measurement.
- `/imu/fixed` (`sensor_msgs/Imu`): IMU data with populated covariance matrices for EKF fusion.

### Important Directories
- `urdf/`: Contains the Xacro/URDF definitions of the BlueROV2.
- `world/`: Contains the SDF files defining the simulation environments (e.g., `small_net.xml`).
- `meshes/`: 3D visual and collision models (.stl or .dae).

---

## 4. Maintenance Guide

If you are a developer taking over this project, here is how you can modify or improve the simulation:

- **Modifying the Robot's Mass/Buoyancy**: Open the URDF/Xacro files in the `urdf/` folder. You can adjust the `<mass>` tags or the buoyancy plugin parameters to make the robot float or sink differently.
- **Adding New Sensors**: If you need a new camera or DVL, add the corresponding Gazebo sensor plugin block into the URDF. Make sure to bind it to a specific physical link on the robot.
- **Tuning Sensor Noise**: If the EKF (localization) is struggling, you can adjust the noise variances in `imu_republisher.py` (`ORIENT_VAR`, `ANGVEL_VAR`, `LINACC_VAR`) or the standard deviation in `simulated_depth_sensor.py` to match the specs of your real-world hardware.
- **Changing the Environment**: Modify the `.xml` files in the `world/` directory. You can add static objects (like rocks or cylinders) to test obstacle avoidance algorithms.

---

## 5. Performance / Optimize Mode

The `AUV_guidance` launch files expose an `optimize` argument that acts on this package's world files at launch time. **No world file is modified on disk** — the patch is applied to a temporary copy.

### What changes with `optimize:=True`

| Parameter | Normal mode | Optimize mode |
|---|---|---|
| Gazebo `max_step_size` | `0.001` s (1 ms) | `0.006` s (6 ms) |
| URDF sensor update rates | Full rate (via `xacro optimize:=false`) | Reduced rate (via `xacro optimize:=true`) |
| Yaw EMA filter alpha (`yaw_ema_alpha`) | `1.0` (no smoothing) | `0.15` (smoothed) |

### How it works (world patch)

When `optimize:=True` is passed to a guidance launch file, the launch script:
1. Reads the selected `.xml` world file from `AUV_description/world/`.
2. Replaces the `<max_step_size>` value in memory using a regex substitution.
3. Writes the modified content to a **temporary file** (`/tmp/gz_world_*.xml`).
4. Passes that temporary file to Gazebo instead of the original.

The original world files in `world/` are **never touched**.

### When to use it

- Use `optimize:=True` on **low-end machines** or when running **headless batch simulations** where real-time physics fidelity is less important than throughput.
- Keep `optimize:=False` (default) for **final validation runs** or whenever sensor timing accuracy matters.
