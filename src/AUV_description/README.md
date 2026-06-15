# AUV Description

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This package defines the **robot body, sensors, and simulation worlds**. It holds the URDF/Xacro model of the BlueROV2, all Gazebo world files, and two helper nodes that make simulation sensor data realistic.

---

## Quick Start

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_description
source install/setup.bash
```

The main mission launch files (`AUV_guidance`) already include everything from this package. The standalone launch files below are mainly useful for development and debugging:

```bash
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
| `urdf/` | Xacro/URDF model — robot shape, mass, buoyancy, sensors |
| `world/` | SDF world files (`small_net.xml`, `ocean_40m.xml`, …) |
| `meshes/` | 3D visual and collision models (.stl / .dae) |
| `launch/` | Standalone launch files for individual environments |
| `config/` | Sensor configuration files |

### Available world files

| File | Description |
|---|---|
| `small_net.xml` | Small aquaculture net (radius ≈ 3.4 m) |
| `ocean_40m.xml` | Large net (radius ≈ 20 m, 40 m deep) |
| `bluerov2_bassin.xml` | Empty pool |
| `Bassin_ntnu_waves.xml` | Pool with waves (needs gz-waves) |

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
| URDF sensor rates | Full rate (e.g. Ping360 10Hz, Sonoptix 15Hz, DVL 15Hz, IMU 50Hz) | Reduced rate (Ping360 1Hz, Sonoptix 2Hz, DVL 5Hz, IMU 25Hz) |

Use `optimize:=True` on low-end machines or for headless batch simulations.

---

## URDF Models & Meshes

The `urdf/` directory contains three robot models:
- `BlueROV2.urdf.xml`: Bare-bones robot (hull + thrusters only).
- `BlueROV2captors.urdf.xml`: Adds basic sensors (camera, basic IMU/depth).
- `Bluerov2_realistic.urdf.xml`: **Main model.** Includes all advanced sensors (Ping360, Sonoptix, DVL) and dynamic `optimize` mode toggles.

The `meshes/` directory contains the detailed 3D visual and collision models (.stl / .dae) that give the BlueROV2 and the aquaculture nets their realistic shapes.

---

## Maintenance

- **Change robot mass / buoyancy:** Edit `urdf/Bluerov2_realistic.urdf.xml` — adjust `<mass>` tags or the buoyancy plugin parameters.
- **Add a sensor:** Add the Gazebo sensor plugin block in the URDF, bound to a physical link. Add a static TF publisher in the launch file to link it to the ROS TF tree.
- **Tune IMU noise:** Edit `ORIENT_VAR`, `ANGVEL_VAR`, `LINACC_VAR` in `imu_republisher.py` to match your real hardware specs.
- **Tune depth noise:** Edit the standard deviation in `simulated_depth_sensor.py`.
- **Add world objects:** Modify the `.xml` files in `world/` — add `<model>` blocks for rocks, cages, or obstacles.
