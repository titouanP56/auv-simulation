# AUV Net Inspection Strategy

This document outlines the multi-phase strategy for the BlueROV2 autonomous inspection of a fishing net in a simulated Gazebo ocean environment.

## Phase 1: Realistic Spawning
- **Objective:** Simulate a realistic deployment of the AUV inside the net.
- **Implementation:** The AUV spawns at a random position within a 20m radius from the center of the 25m radius net. 
- **Depth & Orientation:** It spawns near the surface (between -0.5m and 0.0m) with a completely random yaw orientation. This ensures the search and alignment algorithm is robust regardless of the initial drop angle.

## Phase 2: Descent and Edge Finding
- **Descent:** The AUV immediately dives to a stable navigation depth (e.g., -2.0m) to clear surface interference.
- **Edge Detection:** Using the top-mounted Ping360 mechanical scanning sonar (simulated as a 360° LaserScan), the AUV scans its surroundings to find the shortest distance to the net walls.
- **Alignment:** The AUV rotates (yaw) to face the nearest detected edge directly.
- **Approach:** Once aligned, the AUV moves forward, using the forward-facing Sonoptix ECHO multibeam sonar to approach the net until it reaches a precise 1.5m standoff distance.

## Phase 3: Defining Local Origin
- **Objective:** Establish a repeatable reference frame for the subsequent mapping maneuvers.
- **Action:** Once stable at the 1.5m standoff distance, the AUV records its current Pose. This pose is designated as the "Home" or "Local Origin" reference point for the rest of the mission.

## Phase 4: Circular Inspection (Mapping)
- **Objective:** Map the cylindrical upper portion of the net.
- **Action:** The AUV initiates a lateral movement, performing a 360-degree circular orbit along the inside of the net. It actively maintains the 1.5m distance using the Sonoptix sensor in a control loop.
- **Data Collection:** Sonar and camera data are collected continuously to build a 3D point cloud or map of the net infrastructure.

## Phase 5: Vertical Scanning
- **Objective:** Inspect the conical bottom section of the net.
- **Action:** After completing the horizontal circular inspection, the AUV descends along the Z-axis. It adjusts its pitch and lateral position to follow the tapering trajectory of the conical net structure down towards its apex.
