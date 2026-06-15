# AUV Guidance

> **Tested on:** ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04 LTS

This package is the **mission brain** of the AUV. It contains the full autonomous inspection state machine (Phase 2 + Phase 3) and the thruster bridges that translate wrench commands into motor signals.

---

## Quick Start

```bash
cd ~/AUV_project/ros2_AUV
colcon build --packages-select AUV_guidance
source install/setup.bash

# Full autonomous mission — small net (default)
ros2 launch AUV_guidance net_full_inspection.launch.py

# Full autonomous mission — large net (ocean_40m.xml)
ros2 launch AUV_guidance net_inspection_big_net.launch.py
```

See [Section 3 — Launch Arguments](#3-launch-arguments) for all options.

---

## Package Contents

| Executable | File | Role |
|---|---|---|
| `net_approach` | `net_approach.py` | Phase 2: dive → find net → approach → standoff |
| `phase3_inspection` | `phase3_inspection.py` | Phase 3: orbit small net |
| `phase3_inspection_big_net` | `phase3_inspection_big_net.py` | Phase 3: orbit large net (different PID gains) |
| `sim_thruster_bridge` | `sim_thruster_bridge.py` | Wrench → 8× Float64 thruster commands (Gazebo) |
| `bluerov2_bridge` | `bluerov2_bridge.py` | Wrench → MAVROS RC PWM (real BlueROV2) |

---

## 1. Mission Architecture

### Phase 2 — `net_approach` (net_approach.py)

Responsible for getting the robot from its spawn point to a stable 1.5 m standoff in front of the net.

**State machine:**

```
DESCENDING
    │ depth stable for 2 s at TARGET_DEPTH
    ▼
GLOBAL_SEARCH
    │ AUV holds position; ping360_nearest performs a full 360° scan
    │ transitions immediately on /perception/full_scan_ready = True
    ▼
ALIGNING
    │ PD yaw control until facing the net (within 10°)
    ▼
APPROACHING
    │ forward surge + sonoptix_perception distance control
    ▼
STABILIZING
    │ hold standoff for STABILIZE_TIME = 3 s
    ▼
STANDOFF  ─── publishes /mission/phase2_done = True
              broadcasts local_origin TF frame for Phase 3
```

**Key subscriptions:**

| Topic | Type | Purpose |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | Robot pose |
| `/perception/net_orientation` | `geometry_msgs/PoseStamped` | Net direction (from ping360_nearest) |
| `/perception/full_scan_ready` | `std_msgs/Bool` | Confirms a full rotation is done |
| `/sonoptix/perception` | `geometry_msgs/PoseStamped` | Net distance during approach |
| `/sonoptix/perception_valid` | `std_msgs/Bool` | RANSAC validity gate |

**Key publications:**

| Topic | Type | Purpose |
|---|---|---|
| `/auv/command_wrench` | `geometry_msgs/Wrench` | Body forces (Fx, Fz, Mz) |
| `/mission/phase2_done` | `std_msgs/Bool` | Signals Phase 3 to start |
| `/mission/local_origin` | `geometry_msgs/PoseStamped` | Origin for Phase 3 relative telemetry |

---

### Phase 3 — `phase3_inspection` / `phase3_inspection_big_net`

Orbits the net perimeter at a fixed standoff distance, descending by `DEPTH_STEP` after each complete lap.

**State machine:**

```
WAITING
    │ /mission/phase2_done = True received
    ▼
WALKING_THE_NET  ◄──────────────────────────────┐
    │                                            │
    │ sonar lost or RANSAC invalid > 2 s         │ sonar recovered
    ▼                                            │
LOST_WALL  ─── pulls back + rotates slowly ─────┘
    │
    │ accumulated yaw ≥ 2π  →  decrement depth, reset yaw
    │ depth ≤ FINAL_DEPTH_LIMIT  →
    ▼
LAP_COMPLETED  ─── publishes /mission/phase3_done = True
```

**Four simultaneous PID controllers:**

| DOF | Controller | Set point |
|---|---|---|
| Depth (Fz) | PID | `TARGET_DEPTH` (decrements per lap) |
| Standoff (Fx) | PID | `STANDOFF_DIST` = 1.5 m |
| Sway velocity (Fy) | PID | 0.25 m/s lateral orbit speed |
| Yaw (Mz) | PID + EMA | net normal yaw error |

**Cone mode:** When the net is conical (radius decreasing), the node detects the radius shrinking below 90% of `R_ref` and enables a pitch PID controller to follow the angled net surface to the apex.

**Key subscriptions:**

| Topic | Type | Purpose |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | Robot pose |
| `/sonoptix/perception` | `geometry_msgs/PoseStamped` | Net distance + orientation |
| `/sonoptix/perception_valid` | `std_msgs/Bool` | RANSAC validity |
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

All these constants are defined at the top of their respective files and can be changed without recompiling (they are Python, not ROS parameters). Rebuild and re-source after editing.

**`net_approach.py`:**

| Constant | Default | Effect |
|---|---|---|
| `TARGET_DEPTH` | `-2.0` m | Initial dive depth |
| `STANDOFF_DIST` | `1.5` m | Desired distance from net surface |
| `KP_YAW / KD_YAW` | `5.0 / 2.0` | Yaw alignment aggressiveness |
| `KP_SURGE` | `6.0` | Forward approach speed |
| `GLOBAL_SEARCH_TIMEOUT_SEC` | `60` s | Timeout before emitting a warning |

**`phase3_inspection.py` (small net):**

| Constant | Default | Effect |
|---|---|---|
| `ORBIT_DIRECTION` | `+1` | +1 = CCW, -1 = CW orbit |
| `STANDOFF_DIST` | `1.5` m | Distance from net surface |
| `DEPTH_STEP` | `0.5` m | Depth decrement per lap |
| `FINAL_DEPTH_LIMIT` | `-6.0` m | Stop depth |
| `LOST_WALL_TIMEOUT` | `2.0` s | Sonar silence before LOST_WALL |

**`phase3_inspection_big_net.py` (large net) — same constants but:**

| Constant | Value | Why different |
|---|---|---|
| `FINAL_DEPTH_LIMIT` | `-29.5` m | Big net goes 30 m deep |
| `KP_DEPTH` | `20.0` | Stronger depth hold at depth |
| `KD_DEPTH` | `20.0` | Damping for depth controller |
| `KP_DIST` | `12.0` | Faster standoff correction |
| `KP_VEL_SWAY` | `40.0` | Stronger lateral push |
| `KP_YAW / MAX_YAW_CMD` | `10.0 / 20.0` | More aggressive yaw on large net |

---

## 3. Launch Arguments

Both launch files accept the same arguments:

| Argument | Default | Description |
|---|---|---|
| `headless` | `False` | Run Gazebo without GUI |
| `use_hardware` | `False` | Real BlueROV2 (disables Gazebo, starts MAVROS) |
| `rviz` | `False` | Open RViz2 |
| `world_file` | see below | World file in `AUV_description/world/` |
| `gz_delay` | `8.0` | Seconds to wait before spawning nodes |
| `optimize` | `False` | Performance mode (see below) |

| Launch file | Default world |
|---|---|
| `net_full_inspection.launch.py` | `small_net.xml` |
| `net_inspection_big_net.launch.py` | `ocean_40m.xml` |

---

## 4. Performance / Optimize Mode

| Parameter | Normal (`False`) | Optimized (`True`) |
|---|---|---|
| Gazebo physics step | 1 ms | 6 ms |
| URDF sensor rates | Full | Reduced |
| Control loop rate | 20 Hz | 5 Hz |
| Yaw EMA filter α | 1.0 (raw) | 0.15 (smoothed) |

The world file physics patch is applied in memory only — original files are never modified.

---

## 5. Common Issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot stuck in `GLOBAL_SEARCH` forever | `ping360_nearest` not running or no scan | Check `ping360_nearest` is in the delayed mission |
| Robot approaches and immediately turns away | Yaw target wrong by 180° | Check `_spawn_yaw` sign in the launch file |
| Phase 3 immediately enters `LOST_WALL` | Sonoptix bridge not started | Ensure `sonoptix_perception` is in delayed_mission |
| Orbit drifts away from net | Standoff PID gains too low | Increase `KP_DIST` in the phase3 file |
| Robot oscillates yaw rapidly | Yaw EMA alpha too high | Lower `yaw_ema_alpha` (e.g. `0.15`) or increase `KD_YAW` |
