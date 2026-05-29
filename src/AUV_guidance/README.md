# AUV Guidance

## 1. Introduction for Beginners

Welcome to the **AUV Guidance** package! Think of this package as the "brain" of the underwater robot (AUV). 

While the controller moves the muscles (propellers), the guidance package decides **where** the robot should go and **what** it should look at. During a mission, this package tells the robot to dive, find the net with its sonars, approach it safely, and then orbit around it to perform an inspection. 

This package also contains important "bridges." Since a simulated robot and a real-world robot don't always speak the exact same language, these bridges translate the brain's desired movements (like "push forward with 10 Newtons of force") into the specific commands required by either the Gazebo simulator or the real BlueROV2 hardware.

---

## 2. Quick Start Guide

### Prerequisites
Make sure your ROS 2 workspace is sourced and built.

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_guidance
source install/setup.bash
```

### Running the Mission

The full inspection mission is typically launched via a master launch file, which starts the simulation, controllers, and guidance nodes in the correct sequence.

**Simulated Full Inspection:**
To launch the complete, automated net inspection mission in simulation:
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py
```

**Available Launch Arguments (Balises):**
You can customize the mission execution using the following arguments (append `arg:=value` to the command):

| Argument | Default | Description |
|---|---|---|
| `headless` | `False` | Run Gazebo in the background without the 3D graphical interface (saves GPU resources). |
| `use_hardware` | `False` | Set to `True` to deploy on the real BlueROV2. It disables Gazebo and launches MAVROS + `bluerov2_bridge`. |
| `rviz` | `False` | Launch RViz2 to visualize sensor data, point clouds, and TF trees. |
| `world_file` | `small_net.xml` | Specify the Gazebo world file to load (located in `AUV_description/world/`). |
| `gz_delay` | `8.0` | Seconds to wait for Gazebo to initialize before spawning the robot and mission nodes. |
| `optimize` | `False` | **Performance mode**: coarser physics step (6 ms vs 1 ms), reduced URDF sensor rates, slower control loops (5 Hz vs 20 Hz). Use on low-end machines or for headless batch runs. |

*Example:*
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True rviz:=True use_hardware:=False
```

*Performance mode example (low-end machine or CI):*
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True optimize:=True
```

**(For developers) Running specific nodes manually:**
If you need to test the logic phases individually:
```bash
ros2 run AUV_guidance phase2_mission     # Starts the approach logic
ros2 run AUV_guidance phase3_inspection  # Starts the orbiting logic
```

---

## 3. Technical Architecture

This package orchestrates the high-level mission state machine. It processes sonar point clouds to extract geometric features (wall distance, perpendicular angle, pitch) and computes generalized forces (Wrench) to send to the lower-level thruster bridges.

### Core Nodes

1. **`phase2_mission` (net_approach.py)**: 
   - **Role**: Handles the dive and initial net approach.
   - **Logic**: Uses a state machine (`DESCENDING`, `SCANNING`, `ALIGNING`, `APPROACHING`, `STABILIZING`, `STANDOFF`). It uses the Ping360 sonar to find the general direction of the net, aligns to it, and then uses the Sonoptix sonar to approach until exactly 1.5m away. It finally creates a `local_origin` TF frame.

2. **`phase3_inspection` (phase3_inspection.py & phase3_inspection_big_net.py)**:
   - **Role**: Executes a reactive 360° wall-following orbit around the net.
   - **Logic**: Uses 4 simultaneous PID controllers (Depth, Distance, Lateral Speed, Yaw). It tracks the net's curvature without relying on a predefined map. It tracks laps by integrating the robot's yaw from odometry. A "LOST_WALL" recovery state is implemented in case the sonar loses track of the net.

3. **`sim_thruster_bridge.py`**:
   - **Role**: Translates a 6-DOF `Wrench` command into 8 individual `Float64` motor commands (`/cmd_vel_X`) specifically for the Gazebo plugins using a pseudo-inverse Thruster Allocation Matrix.

4. **`bluerov2_bridge.py`**:
   - **Role**: Translates a 6-DOF `Wrench` command into MAVROS `OverrideRCIn` (RC PWM signals) to control the real BlueROV2 running ArduSub.

### Subscribed Topics
- `/odometry/filtered` (`nav_msgs/Odometry`): Localization data.
- `/ping360/scan` (`sensor_msgs/LaserScan`): 2D mechanical scanning sonar.
- `/sonoptix/points` (`sensor_msgs/PointCloud2`): 3D multibeam imaging sonar.
- `/auv/command_wrench` (`geometry_msgs/Wrench`): Desired body forces (used by the bridges).

### Published Topics
- `/auv/command_wrench` (`geometry_msgs/Wrench`): Output of Phase 2 and 3 nodes.
- `/cmd_vel_[1-8]` (`std_msgs/Float64`): Simulation motor commands (sim_thruster_bridge).
- `/mavros/rc/override` (`mavros_msgs/OverrideRCIn`): Hardware motor commands (bluerov2_bridge).
- `/mission/phase` (`std_msgs/String`): Current state of the mission.

---

## 4. Maintenance Guide

If you are a developer taking over this project, here is how you can modify or improve the guidance code:

- **Modifying the Inspection Speed**: In `phase3_inspection.py`, find the `target_vy` variable in the `_do_walking` method. It is currently set to `0.25` m/s. Adjust this to make the robot orbit faster or slower.
- **Adjusting the Standoff Distance**: If the robot is too close or too far from the net, change the `STANDOFF_DIST` constant (currently 1.5m) at the top of the phase scripts.
- **Improving Wall Recovery**: If the robot frequently loses the net and struggles to find it in `LOST_WALL` state, consider tweaking the `RECOVERY_YAW_CMD` (the speed at which it rotates to search) and `LOST_WALL_TIMEOUT` parameters.
- **Hardware vs Simulation**: When moving from Gazebo to the real pool, ensure your launch files remap the outputs correctly. You must run `bluerov2_bridge` instead of `sim_thruster_bridge`, and ensure MAVROS is properly configured.

---

## 5. Performance / Optimize Mode

Both launch files (`net_full_inspection.launch.py` and `net_inspection_big_net.launch.py`) expose an `optimize` argument that reduces simulation load without changing any source file.

### What changes with `optimize:=True`

| Parameter | Normal mode (`False`) | Optimize mode (`True`) |
|---|---|---|
| Gazebo `max_step_size` | `0.001` s (1 ms) | `0.006` s (6 ms) |
| URDF sensor update rates | Full rate | Reduced rate (via `xacro optimize:=true`) |
| Mission control loop (`control_rate_hz`) | 20 Hz | 5 Hz |
| Yaw EMA filter alpha (`yaw_ema_alpha`) | `0.15` (smoothed) | `1.0` (no smoothing) |

### Affected launch files

| Launch file | Default world | Target scenario |
|---|---|---|
| `net_full_inspection.launch.py` | `small_net.xml` | Small aquaculture net (radius ≈ 3.4 m) |
| `net_inspection_big_net.launch.py` | `ocean_40m.xml` | Large net (radius ≈ 20 m) |

### How it works

1. **Physics step** — The launch script reads the selected world `.xml`, patches `<max_step_size>` in memory with a regex, and writes the result to a temporary file. The original world files in `AUV_description/world/` are **never modified on disk**.
2. **URDF sensors** — The `optimize` flag is forwarded to Xacro (`xacro … optimize:=True/False`), which can toggle sensor update rates at model-generation time.
3. **Control loops** — `control_rate_hz` is set to `5.0` Hz (optimize) vs `20.0` Hz (normal) for both `net_approach` and `phase3_inspection*` nodes.
4. **Yaw filter** — `yaw_ema_alpha` is set to `1.0` (raw, no EMA) in optimize mode to reduce CPU load.

### When to use it

- **`optimize:=True`** → headless CI/CD runs, low-end laptops, or batch data collection where physics fidelity is secondary.
- **`optimize:=False`** (default) → final validation, demos, or any run where sensor timing accuracy matters.
