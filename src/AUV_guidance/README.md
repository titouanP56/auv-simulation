# AUV_guidance

This package acts as the **Guidance** module in the `ros2_AUV` workspace. It is responsible for decision making, state machines, and generating continuous trajectories for the controllers to follow.

## Main Features

- **Mission State Machines**: High-level scripts that manage the sequence of operations for specific tasks (e.g., finding a target, approaching, and standing off).
- **Trajectory Generation**: Continuous path planning nodes that output target poses (`/cmd_setpoint`) for the MPC or PID controllers to track.
- **Mission Integration (Launch)**: Top-level launch files that orchestrate the startup of the simulator, the guidance nodes, and the controllers.

## Main Nodes and Scripts

### Guidance Nodes
- **`net_approach.py`**: A state machine designed to bring the AUV from the surface down to a specific depth, scan for an underwater wall (or net) using the Ping360 sonar, approach it using Sonoptix data, and establish a standoff distance to define a new local origin.
- **`phase3_inspection.py`**: A reactive wall-following node that integrates Sonoptix point clouds for surface perpendicularity and distance regulation. Supports cyclic multi-level depth inspection (e.g., -2m, -4m, -6m steps).

### Launch Files
- **`net_full_inspection.launch.py`**: The primary mission bring-up script. Orchestrates the approach and the cyclic reactive inspection sequentially in the `small_net.xml` environment.

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
