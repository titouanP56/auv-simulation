# AUV Description

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This package defines the **robot body, sensors, and simulation worlds**. It holds the URDF/Xacro model of the BlueROV2, all Gazebo world files, and two helper nodes that make simulation sensor data realistic.

The **active robot model** is `Bluerov2_realistic_2D.urdf.xml`, which features a **Sonoptix ECHO 2D** multi-beam sonar (256 rays, ±60°, 25 Hz) as the main sensor for net detection.

---

## Quick Start

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_description
source install/setup.bash
```

The main mission launch file (`AUV_guidance net_full_inspection_true_auv.launch.py`) already includes everything from this package. The standalone launch files below are mainly useful for development and debugging:

```bash
# Realistic robot (Sonoptix 2D + Ping360 + DVL + IMU) in NTNU basin — no mission
ros2 launch AUV_description bluerov2_realist_bassin.launch.py

# Realistic robot in basin with net — no mission
ros2 launch AUV_description bluerov2_realist_net.launch.py

# Empty pool — basic movement tests
ros2 launch AUV_description bluerov2_bassin.launch.py

# Pool with all sensors active
ros2 launch AUV_description bluerov2_bassin_captors.launch.py

# Open ocean with large aquaculture net
ros2 launch AUV_description bluerov2_ocean_realistic.launch.py
```

> **Wave simulation** (`bluerov2_bassin_waves.launch.py`) requires `gz-waves` from `asv_wave_sim`, which is excluded from the default `colcon build` via `COLCON_IGNORE`. The main net inspection missions do **not** need it.

---

## Key Directories

| Directory | Contents |
|---|---|
| `urdf/` | Xacro/URDF models — robot shape, mass, buoyancy, sensors |
| `world/` | SDF world files (`small_net.xml`, `ocean_40m.xml`, …) |
| `meshes/` | 3D visual and collision models (.stl / .dae) |
| `launch/` | Standalone launch files for individual environments |
| `config/` | Sensor configuration files |

---

## URDF Models

| File | Description |
|---|---|
| `BlueROV2.urdf.xml` | Bare-bones robot (hull + thrusters only) |
| `BlueROV2captors.urdf.xml` | Adds basic sensors (camera, IMU, depth) |
| `Bluerov2_realistic.urdf.xml` | Realistic model with 3D Sonoptix (PointCloud2) |
| `Bluerov2_realistic_2D.urdf.xml` | ⭐ **Active model** — Sonoptix ECHO 2D (LaserScan, 25 Hz) |

### `Bluerov2_realistic_2D.urdf.xml` — Active Model

This is the model loaded by both `bluerov2_realist_bassin.launch.py` and `net_full_inspection_true_auv.launch.py`.

**Simulated sensors:**

| Sensor | Frame | Gazebo type | ROS 2 topic | Rate | Characteristics |
|---|---|---|---|---|---|
| **Sonoptix ECHO 2D** | `sonoptix_link` | `gpu_lidar` | `/sonoptix/points` (LaserScan) | **25 Hz** | 256 rays, ±60°, range 0.3–15 m, σ=0.03 m, forward tilt +5° |
| **Ping360** | `ping360_link` | `gpu_lidar` | `/ping360/scan` (LaserScan) | 10 Hz | 360 rays, range 0.75–15 m, σ=0.02 m |
| **IMU** | `imu_link` | `imu` | `/imu` | 50 Hz | Gyro σ=2×10⁻⁴, accel σ=1.7×10⁻² |
| **DVL** | `base_link` | `dvl` (custom) | `/dvl/velocity` | 15 Hz | 4 beams, σ=0.01 m/s |
| **Camera** | `camera_link` | `camera` | `/camera/image_raw` | — | Disabled (commented out) |

> The Sonoptix ECHO 2D models the forward-facing multi-beam sonar as a flat horizontal fan (1 vertical row × 256 horizontal rays) covering ±60°. This is sufficient for 2D net detection and much lighter than a 3D PointCloud2.

**Key physical parameters:**
- Mass: 12.5 kg
- Centre of buoyancy: `[0, 0, +0.1 m]` (above centre of mass → passive pitch stability)
- Hydrodynamic drag: added mass Xu̇=6.36, Yv̇=7.12, Zẇ=12.0; quadratic drag Xu|u|=−18.18

**Performance profile** (xacro arg `optimize:=True/False`):

| Sensor | Normal rate | Optimized rate |
|---|---|---|
| Ping360 | 360 samples @ 10 Hz | 90 samples @ 1 Hz |
| Sonoptix | 64 h-samples @ 15 Hz (xacro) / **256 @ 25 Hz** (URDF) | 16 samples @ 2 Hz |
| DVL | 15 Hz | 5 Hz |
| IMU | 50 Hz | 25 Hz |

> **Note:** The Sonoptix sensor block is **hardcoded** in the URDF at 25 Hz / 256 rays (not controlled by the Xacro optimize flag). The optimize profile only reduces the *auxiliary* Sonoptix Xacro properties.

---

## Available World Files

| File | Description |
|---|---|
| `small_net.xml` | **Default** — small aquaculture net (radius ≈ 3.4 m) |
| `small_net_current.xml` | Small net with underwater current |
| `small_net_deforme.xml` | Deformed net |
| `ocean_40m.xml` | Large net (radius ≈ 20 m, 40 m deep) |
| `Bassin_ntnu.xml` | NTNU basin (realistic pool environment) |
| `Bassin_ntnu_waves.xml` | NTNU basin with waves (needs gz-waves) |
| `cube_obstacle.xml` | Obstacle cube (avoidance tests) |

---

## Helper Nodes

### `simulated_depth_sensor`

Simulates a realistic pressure-based depth sensor.

- Subscribes to `/odom` (perfect Gazebo odometry).
- Extracts the Z coordinate and injects Gaussian noise (σ = 2 cm).
- Publishes `geometry_msgs/PoseWithCovarianceStamped` on `/depth/pose` with a proper Z covariance matrix — as expected by the EKF.

### `imu_republisher`

Fixes Gazebo Harmonic IMU covariance matrices.

Gazebo publishes IMU data with **all-zero covariance matrices**. The EKF interprets zero covariance as "perfect sensor" and rejects all other inputs. This node intercepts `/imu`, injects realistic variance values (derived from the URDF noise parameters), and republishes on `/imu/fixed`.

| Topic | Type | Direction |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Subscribe (depth sensor) |
| `/imu` | `sensor_msgs/Imu` | Subscribe (IMU republisher) |
| `/depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Publish |
| `/imu/fixed` | `sensor_msgs/Imu` | Publish |

---

## Performance / Optimize Mode

The `AUV_guidance` launch files accept `optimize:=True`, which patches the physics step size of the world file **in memory only** before passing it to Gazebo. The original `.xml` files in `world/` are never modified on disk.

| Parameter | Normal | Optimized |
|---|---|---|
| `max_step_size` | 1 ms | 6 ms |
| URDF sensor rates | Full (Ping360 10 Hz, DVL 15 Hz, IMU 50 Hz) | Reduced (Ping360 1 Hz, DVL 5 Hz, IMU 25 Hz) |

Use `optimize:=True` on low-end machines or for headless batch simulations.

---

## Maintenance

- **Change robot mass / buoyancy:** Edit `urdf/Bluerov2_realistic_2D.urdf.xml` — adjust the `<mass>` tag or the `<center_of_buoyancy>` in the Hydrodynamics plugin.
- **Tune the Sonoptix 2D field of view:** Edit the `<min_angle>` / `<max_angle>` in the `sonoptix_sonar` sensor block (currently ±1.047 rad = ±60°).
- **Tune Sonoptix resolution:** Edit `<samples>` (currently 256) in the sonoptix sensor block.
- **Add a sensor:** Add the Gazebo sensor plugin block in the URDF, bound to a physical link. Add a static TF publisher in the launch file to link it to the ROS TF tree.
- **Tune IMU noise:** Edit `ORIENT_VAR`, `ANGVEL_VAR`, `LINACC_VAR` in `imu_republisher.py` to match your real hardware specs.
- **Tune depth noise:** Edit the standard deviation in `simulated_depth_sensor.py`.
- **Add world objects:** Modify the `.xml` files in `world/` — add `<model>` blocks for rocks, cages, or obstacles.
