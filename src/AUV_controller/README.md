# AUV Controller (Legacy / Research Archive)

> **Tested environment:** ROS 2 **Jazzy** + Gazebo **Harmonic** on Ubuntu 24.04 LTS.

## 1. Introduction for Beginners

Welcome to the **AUV Controller** package! 

> [!WARNING]
> **This package is currently archived.** In the current architecture, the net inspection mission relies on a reactive PID approach handled entirely within the `AUV_guidance` package. This `AUV_controller` package remains as a research archive containing advanced Model Predictive Control (MPC) and legacy Station Keeping logic.

Think of this package as the theoretical "muscles and reflexes" of the underwater robot (AUV). It was designed to use advanced mathematics to calculate exactly how fast each of the robot's 8 propellers needs to spin to smoothly track complex trajectories using Model Predictive Control (MPC).

---

## 2. Quick Start Guide

### Prerequisites
Make sure ROS 2 Jazzy is installed and your workspace is built. See the [root README installation guide](../../README.md#2-system-requirements--installation) if this is your first time.

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_controller
source install/setup.bash
```

### Running the Controllers

*Note: These nodes are no longer started automatically by the main mission launch files.*

**Run Station Keeping:**
Keeps the robot at a fixed point using a standard PD controller.
```bash
ros2 run AUV_controller station_keeping
```

**Run the Sensor-based MPC:**
Handles path tracking using Model Predictive Control with filtered odometry.
```bash
ros2 run AUV_controller mpc_controller_sensors
```

**Run Testing Tools:**
Force the robot to dive or move forward in an open loop (no sensors). Useful for basic hardware engine testing.
```bash
ros2 run AUV_controller move_down
ros2 run AUV_controller move_forward
```

---

## 3. Technical Architecture

This package uses advanced control theory, primarily Model Predictive Control (MPC) via CasADi and `do_mpc`, and Proportional-Derivative (PD) control.

### Core Nodes

1. **`mpc_controller_sensors`**:
   - An MPC node that subscribes to filtered EKF odometry.
   - Computes an optimal trajectory horizon considering BlueROV2 hydrodynamics (added mass, linear/quadratic drag).
   - Solves a non-linear optimization problem to minimize the cost function (distance to target, energy, thruster smoothness).

2. **`mpc_controller_bluerov`**:
   - A theoretical MPC version relying on perfect ground-truth odometry from Gazebo (`/odom`). Used primarily for baseline testing and tuning the mathematical model.

3. **`station_keeping`**: 
   - A robust Proportional-Derivative (PD) controller.
   - Calculates the error between the current position/yaw and a fixed target.
   - Uses a Moore-Penrose pseudo-inverse Thruster Allocation Matrix (TAM) to map the desired 6-DOF wrench to the 8 thrusters.

### Subscribed Topics
- `/odometry/filtered` (`nav_msgs/Odometry`): Robust state estimation from the EKF.
- `/odom` (`nav_msgs/Odometry`): Exact Gazebo ground-truth odometry (for theoretical models/debugging).
- `/cmd_setpoint` (`geometry_msgs/PoseStamped`): The target position and orientation.

### Published Topics
- `/cmd_vel_[1-8]` (`std_msgs/Float64`): Individual force commands (or angular velocities) sent to the 8 thrusters.
- `/mpc_tracking_error` (`std_msgs/Float64MultiArray`): Tracking error telemetry for Foxglove analysis.

---

## 4. Maintenance Guide

If you are a developer researching or modifying the MPC controllers:

- **Tuning the MPC**: The behavior of the MPC (aggressiveness vs. smoothness) is entirely dictated by the cost function in the `setup_mpc()` methods. To make the robot strictly hold an angle, increase the `yaw_err` penalty. To save energy, increase the penalty on `u_vec_cost`.
- **Modifying the Robot Physics**: If you change the robot's physical frame or add heavy sensors, you must update the `mass_body`, added mass coefficients, and the Thruster Allocation Matrix (TAM) hardcoded in the scripts.
- **Handling Constraints**: If the MPC solver fails or takes too long (>250ms), try relaxing the non-linear constraints (e.g., maximum pitch/roll torques) or increasing the solver tolerances (`ipopt.tol`).
