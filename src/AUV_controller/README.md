# AUV Controller — Research Archive

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

> [!WARNING]
> **This package is archived.** It is not used in the current net inspection mission. The active guidance logic lives entirely in the `AUV_guidance` package. This package remains as a research reference for advanced control work.

---

## What is in here?

This package was developed to explore advanced control strategies for the BlueROV2:

- **Model Predictive Control (MPC)** using [CasADi](https://web.casadi.org/) / `do_mpc` — computes optimal thrust sequences over a finite horizon, accounting for hydrodynamics (added mass, linear/quadratic drag).
- **Station Keeping** — a simpler PD controller that holds the robot at a fixed point using a pseudo-inverse Thruster Allocation Matrix.
- **Open-loop test utilities** (`move_down`, `move_forward`) — useful for basic hardware thruster testing.

---

## Running (for research / development only)
 
```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_controller
source install/setup.bash

# Station keeping at current position
ros2 run AUV_controller station_keeping

# MPC with EKF odometry
ros2 run AUV_controller mpc_controller_sensors

# Open-loop tests
ros2 run AUV_controller move_down
ros2 run AUV_controller move_forward
```

---

## Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/odometry/filtered` | `nav_msgs/Odometry` |
| Subscribe | `/odom` | `nav_msgs/Odometry` (ground truth, for MPC baseline) |
| Subscribe | `/cmd_setpoint` | `geometry_msgs/PoseStamped` |
| Publish | `/cmd_vel_[1-8]` | `std_msgs/Float64` |
| Publish | `/mpc_tracking_error` | `std_msgs/Float64MultiArray` |

---

## Maintenance (if you revive this)

- **MPC tuning:** The cost function weights in `setup_mpc()` control behaviour. Increase `yaw_err` weight to hold heading more tightly; increase `u_vec_cost` to save thruster energy.
- **Robot physics:** If you change the frame or add heavy sensors, update `mass_body`, added mass coefficients, and the Thruster Allocation Matrix (TAM).
- **Solver issues:** If the MPC solver fails or exceeds 250 ms, relax the non-linear constraints or increase `ipopt.tol` in the solver settings.
- **Dependencies:** Requires `casadi` and `do_mpc` Python packages (`pip install casadi do_mpc`).
