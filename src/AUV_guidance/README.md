# AUV Guidance

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This package is the **mission brain** of the AUV. It contains the full autonomous inspection state machine (Phase 2 + Phase 3) and the thruster bridges that translate wrench commands into motor signals.

The **active mission pipeline** uses the **Sonoptix ECHO 2D** sonar for net detection (via `net_approach_2D_sono` and `phase3_inspection_2D_sono`). The Ping360 is used only for the initial search phase.

---

## Quick Start

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_guidance
source install/setup.bash

# Full autonomous mission — 2D Sonoptix pipeline (recommended)
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py

# Full autonomous mission — large net variant
ros2 launch AUV_guidance net_inspection_big_net.launch.py
```

See [Section 3 — Launch Arguments](#3-launch-arguments) for all options.

---

## Package Contents

| Executable | File | Role |
|---|---|---|
| `net_approach_2D_sono` | `net_approach_2D_sono.py` | **Phase 2** (active): dive → Ping360 search → approach with Sonoptix 2D → standoff |
| `net_approach` | `net_approach.py` | Phase 2 (legacy): uses 3D Sonoptix PointCloud2 |
| `phase3_inspection_2D_sono` | `phase3_inspection_2D_sono.py` | **Phase 3** (active): orbit with Sonoptix 2D distance + yaw |
| `phase3_inspection` | `phase3_inspection.py` | Phase 3 (legacy): uses 3D Sonoptix PoseStamped |
| `phase3_inspection_big_net` | `phase3_inspection_big_net.py` | Phase 3: orbit large net (different PID gains) |
| `sim_thruster_bridge` | `sim_thruster_bridge.py` | Wrench → 8× Float64 thruster commands (Gazebo) |
| `bluerov2_bridge` | `bluerov2_bridge.py` | Wrench → MAVROS RC PWM (real BlueROV2) |

---

## 1. Mission Architecture

### Phase 2 — `net_approach_2D_sono` (active)

Responsible for getting the robot from its spawn point to a stable 1.5 m standoff in front of the net. The Ping360 is used for the initial global search; the **Sonoptix 2D** takes over as soon as the robot starts approaching.

**State machine:**

```
DESCENDING
    │ depth stable for 2 s at TARGET_DEPTH (−3 m)
    ▼
GLOBAL_SEARCH
    │ AUV holds position; ping360_nearest performs a full 360° scan
    │ transitions on /perception/full_scan_ready = True
    │ (uses /perception/net_orientation from Ping360)
    ▼
ALIGNING
    │ PD yaw control until facing the net (within 10°, hold 1 s)
    ▼
APPROACHING
    │ forward surge + Sonoptix 2D distance control
    │ (uses /perception/net_distance — Float32 from sonoptix_2D_perception)
    ▼
STABILIZING
    │ hold standoff for STABILIZE_TIME = 3 s
    ▼
STANDOFF
    │ waits 5 s, then publishes /mission/phase2_done = True
    └─ broadcasts local_origin TF frame for Phase 3
```

**Key subscriptions:**

| Topic | Type | Purpose |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | Robot pose |
| `/perception/net_orientation` | `geometry_msgs/PoseStamped` | Net direction (from ping360_nearest) |
| `/perception/full_scan_ready` | `std_msgs/Bool` | Confirms a full Ping360 rotation is done |
| `/perception/net_distance` | `std_msgs/Float32` | Net distance (from sonoptix_2D_perception) |
| `/perception/perception_valid` | `std_msgs/Bool` | RANSAC validity gate |

**Key publications:**

| Topic | Type | Purpose |
|---|---|---|
| `/auv/command_wrench` | `geometry_msgs/Wrench` | Body forces (Fx, Fz, Mz) |
| `/mission/phase2_done` | `std_msgs/Bool` | Signals Phase 3 to start |
| `/mission/local_origin` | `geometry_msgs/PoseStamped` | Origin for Phase 3 relative telemetry |

---

### Phase 3 — `phase3_inspection_2D_sono` (active)

Orbits the net perimeter at a fixed standoff distance, descending by `DEPTH_STEP` after each complete lap. Uses the **Sonoptix 2D** for both distance regulation and yaw alignment.

**State machine:**

```
WAITING
    │ /mission/phase2_done = True received
    ▼
WALKING_THE_NET  ◄──────────────────────────────┐
    │                                            │
    │ sonar lost or RANSAC invalid > 2 s         │ sonar + RANSAC recovered
    ▼                                            │
LOST_WALL  ─── pulls back (Fx=−5) + rotates ───┘
    │
    │ accumulated yaw ≥ 2π  →  decrement depth, reset yaw
    │ depth ≤ FINAL_DEPTH_LIMIT  →
    ▼
LAP_COMPLETED  ─── publishes /mission/phase3_done = True
```

**Five simultaneous PID controllers:**

| DOF | Controller | Set point |
|---|---|---|
| Depth (Fz) | PID | `TARGET_DEPTH` (decrements per lap) |
| Standoff (Fx) | PID | `STANDOFF_DIST` = 1.5 m (from `/perception/net_distance`) |
| Sway velocity (Fy) | PID | 0.25 m/s lateral orbit speed |
| Yaw (Mz) | PID + EMA + rate-limit | net normal yaw from `/perception/net_yaw_target` |
| Pitch (My) | PID | net cone angle (only in `in_cone_mode`) |

**Range filtering on the sonar distance:**
- **Spike filter**: rejects jumps > 0.3 m (up to 5 consecutive rejections).
- **EMA filter** (α = 0.5): smooths validated range values.

**Cone mode:** When the net is conical, the node detects the estimated orbit radius shrinking below 90% of `R_ref` (measured over the first 30 s) and enables the pitch PID after a 3 s confirmation window.

**Key subscriptions:**

| Topic | Type | Purpose |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | Robot pose |
| `/perception/net_distance` | `std_msgs/Float32` | Net distance from Sonoptix 2D |
| `/perception/net_yaw_target` | `std_msgs/Float32` | Net yaw from Sonoptix 2D |
| `/perception/perception_valid` | `std_msgs/Bool` | RANSAC validity |
| `/mission/phase2_done` | `std_msgs/Bool` | Start trigger |

**Key publications:**

| Topic | Type | Purpose |
|---|---|---|
| `/auv/command_wrench` | `geometry_msgs/Wrench` | Body forces (Fx, Fy, Fz, Mz, My) |
| `/mission/phase3_done` | `std_msgs/Bool` | Mission complete flag |
| `/phase3/*` | `std_msgs/Float64` | Diagnostic telemetry (all variables) |

---

### Thruster Bridges

**`sim_thruster_bridge`** — Simulation only.

Maps the 6-DOF `Wrench` (Fx, Fy, Fz, Mx, My, Mz) onto 8 individual thruster `Float64` commands using a pseudo-inverse Thruster Allocation Matrix (TAM). Publishes to `/cmd_vel_1` … `/cmd_vel_8`.

**`bluerov2_bridge`** — Real hardware only.

Converts the `Wrench` to MAVROS `OverrideRCIn` (1100–1900 µs PWM) for the 8 thrusters running ArduSub. Active only when `use_hardware:=True`.

---

## 2. Key Tuning Constants

All these constants are defined at the top of their respective files (Python, not ROS parameters). Rebuild and re-source after editing.

**`net_approach_2D_sono.py`:**

| Constant | Default | Effect |
|---|---|---|
| `TARGET_DEPTH` | `−3.0` m | Initial dive depth |
| `STANDOFF_DIST` | `1.5` m | Desired distance from net surface |
| `KP_YAW / KD_YAW` | `5.0 / 2.0` | Yaw alignment aggressiveness |
| `KP_SURGE` | `6.0` | Forward approach speed |
| `GLOBAL_SEARCH_TIMEOUT_SEC` | `60` s | Timeout before emitting a warning |

**`phase3_inspection_2D_sono.py` (small net):**

| Constant | Default | Effect |
|---|---|---|
| `ORBIT_DIRECTION` | `+1` | +1 = CCW, -1 = CW orbit |
| `STANDOFF_DIST` | `1.5` m | Distance from net surface |
| `DEPTH_STEP` | `0.5` m | Depth decrement per lap |
| `FINAL_DEPTH_LIMIT` | `−6.0` m | Stop depth |
| `LOST_WALL_TIMEOUT` | `2.0` s | Sonar silence before LOST_WALL |
| `KP_DIST / KI_DIST / KD_DIST` | `4.0 / 0.2 / 0.5` | Standoff PID |
| `KP_VEL_SWAY` | `15.0` | Lateral orbit speed controller |
| `KP_YAW / KI_YAW / KD_YAW` | `5.0 / 0.02 / 1.0` | Yaw alignment PID |

---

## 3. Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `headless` | `False` | Run Gazebo without GUI |
| `use_hardware` | `False` | Real BlueROV2 (disables Gazebo, starts MAVROS) |
| `rviz` | `False` | Open RViz2 |
| `world_file` | `small_net.xml` | World file in `AUV_description/world/` |
| `gz_delay` | `8.0` | Seconds to wait before spawning mission nodes |
| `optimize` | `False` | Performance mode (see below) |

| Launch file | Default world | Pipeline |
|---|---|---|
| `net_full_inspection_true_auv.launch.py` | `small_net.xml` | **2D Sonoptix (active)** |
| `net_full_inspection.launch.py` | `small_net.xml` | 3D Sonoptix (legacy) |
| `net_inspection_big_net.launch.py` | `ocean_40m.xml` | Large net |

---

## 4. Performance / Optimize Mode

| Parameter | Normal (`False`) | Optimized (`True`) |
|---|---|---|
| Gazebo physics step | 1 ms | 6 ms |
| URDF sensor rates | Full | Reduced |
| Control loop rate | 20 Hz | 5 Hz |
| Yaw EMA filter α | 0.15 | 1.0 (raw, no smoothing) |

The world file physics patch is applied in memory only — original files are never modified.

---

## 5. Common Issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot stuck in `GLOBAL_SEARCH` forever | `ping360_nearest` not running or no scan | Check that `ping360_nearest` is in the delayed mission |
| Robot approaches but RANSAC invalid | Sonoptix bridge not started | Ensure `sonoptix_2D_perception` is in delayed_mission |
| Phase 3 immediately enters `LOST_WALL` | No `/perception/net_distance` data | Check `sonoptix_2D_perception` is running and bridge is up |
| Orbit drifts away from net | Standoff PID gains too low | Increase `KP_DIST` in `phase3_inspection_2D_sono.py` |
| Robot oscillates yaw rapidly | Yaw EMA alpha too high | Lower `yaw_ema_alpha` (e.g. `0.15`) or increase `KD_YAW` |
| Phase 3 never exits LOST_WALL | Sonoptix out of net range | Check `STANDOFF_DIST` vs net geometry |
