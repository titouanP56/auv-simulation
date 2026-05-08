# AUV_guidance

This package acts as the **Guidance** module in the `ros2_AUV` workspace. It is responsible for decision making, state machines, and generating continuous trajectories for the controllers to follow.

## Main Features

- **Mission State Machines**: High-level scripts that manage the sequence of operations for specific tasks (e.g., finding a target, approaching, and standing off).
- **Trajectory Generation**: Continuous path planning nodes that output target poses (`/cmd_setpoint`) for the MPC or PID controllers to track.
- **Mission Integration (Launch)**: Top-level launch files that orchestrate the startup of the simulator, the guidance nodes, and the controllers.

## Main Nodes and Scripts

### Guidance Nodes
- **`net_approach.py`**: A state machine designed to bring the AUV from the surface down to a specific depth, scan for an underwater wall (or net) using the Ping360 sonar, approach it rapidly using Sonoptix data, and establish a standoff distance to define a new local origin before triggering the inspection via the `/mission/phase2_done` topic.
- **`phase3_inspection.py`**: A reactive PID-based wall-following node that directly maps errors to thruster forces. It controls perpendicularity and standoff distance using filtered Sonoptix 3D point clouds (median filter, spike rejection), while using a constant lateral sway thrust to perform a 360-degree orbit. It tracks lap completion by integrating the robot's yaw and automatically descends in successive depth steps (e.g., dropping by 0.5m down to -6.0m limit).
- **`phase3_inspection_big_net.py`**: Similar to `phase3_inspection.py` but configured for much larger and deeper environments, allowing the AUV to descend up to a depth limit of -29.5m.
- **`sim_thruster_bridge.py`**: Bridges the gap between high-level force commands (`Wrench`) and individual thruster PWM/Float levels in Gazebo simulation.
- **`bluerov2_bridge.py`**: Translates `Wrench` commands into MAVROS-compatible messages for real BlueROV2 hardware.

### Launch Files
- **`net_full_inspection.launch.py`**: The primary mission bring-up script. Orchestrates the approach and the cyclic reactive inspection sequentially in the `small_net.xml` environment. Supports `use_hardware:=True` to switch from Gazebo to MAVROS/Hardware.
- **`net_full_inspection_deforme.launch.py`**: Fully synchronized with the primary mission launch. Executed in the `small_net_deforme.xml` environment to test mission robustness against deformations. Also supports the `use_hardware` flag.
- **`net_inspection_big_net.launch.py`**: Launch script for the deep `ocean_40m.xml` environment. Spawns the AUV at a 20m radius and executes the approach along with the `phase3_inspection_big_net` node.

## Dependencies

- `rclpy`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`
- `tf2_ros`

## Usage

**1. Launch the Full Cyclic Mission (Recommended):**
Sequentially executes approach and multi-level reactive inspection.
```bash
ros2 launch AUV_guidance net_full_inspection.launch.py headless:=False
```

**2. Run Individual Nodes (Requires active simulation) (not recommanded):**
```bash
ros2 run AUV_guidance net_approach
```
```bash
ros2 run AUV_guidance phase3_inspection
```
