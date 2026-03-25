# AUV_guidance

This package acts as the **Guidance** module in the `ros2_AUV` workspace. It is responsible for decision making, state machines, and generating continuous trajectories for the controllers to follow.

## Main Features

- **Mission State Machines**: High-level scripts that manage the sequence of operations for specific tasks (e.g., finding a target, approaching, and standing off).
- **Trajectory Generation**: Continuous path planning nodes that output target poses (`/cmd_setpoint`) for the MPC or PID controllers to track.
- **Mission Integration (Launch)**: Top-level launch files that orchestrate the startup of the simulator, the guidance nodes, and the controllers.

## Main Nodes and Scripts

### Guidance Nodes
- **`net_approach.py`**: A state machine designed to bring the AUV from the surface down to a specific depth, scan for an underwater wall (or net) using the Ping360 sonar, approach it using Sonoptix data, and establish a standoff distance to define a new local origin.
- **`lawnmower_trajectory_node.py`**: Generates a classic "lawnmower" (zigzag) sweeping trajectory relative to a predefined local origin. It continuously publishes waypoints intended for the MPC.

### Launch Files
- **`net_inspection.launch.py`**: A comprehensive bring-up script. It launches the Gazebo simulation environment (Dynamics), the robot state publisher, the EKF (Navigation), the trajectory generator (Guidance), and the MPC (Control), effectively starting the entire Phase 4 inspection mission.

## Dependencies

- `rclpy`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`
- `tf2_ros`

## Usage

**1. Launch the full Phase 4 Integration:**
This will start Gazebo, spawn the robot, start the estimators, wait for them to stabilize, and then launch both the guidance trajectory and the MPC.
```bash
ros2 launch AUV_guidance net_inspection.launch.py
```

**2. Run Individual Nodes (Requires active simulation) (not recommanded):**
```bash
ros2 run AUV_guidance net_approach
```
```bash
ros2 run AUV_guidance lawnmower_trajectory_node
```
