# auv_perception

The `auv_perception` package provides 3D mapping capabilities for the ROS 2 AUV project. It filters noisy sonar feedback and relies on standard tools like `octomap_server` to generate real-time voxel occupancy grids.

## Features

1. **Sonar Filtering (`sonar_filter_node.py`)**: 
   Subscribes to raw Sonoptix sonar data (`/sonoptix/points`), filters out points with a range greater than 4.0 meters to remove noise and out-of-bounds echoes, and publishes the clean cloud to `/sonoptix/points_filtered`.

2. **3D Mapping (OctoMap)**: 
   Utilizes the standard `octomap_server_node` to build a 3D probabilistic occupancy grid (`odom` to `base_link` tracking). Water surface constraints are managed via the `occupancy_max_z` parameter.

3. **Auto-Saving (`auto_saver_node.py`)**: 
   A background Python node that automatically calls the OctoMap CLI tool via `subprocess.run` to save the current map state to `net_map_autosave.bt` within this package's directory every 60 seconds. It also guarantees a final save upon graceful shutdown (`Ctrl+C`).

## Usage

You can launch the entire perception stack (filtering, mapping, and auto-saving) with a single launch file in parallel with your Gazebo environment and vehicle controller:

```bash
ros2 launch auv_perception mapping.launch.py
```

## Configuration

OctoMap parameters (voxel resolution, raycasting range, tracking frames, and Z-limits) are firmly defined in `config/octomap_params.yaml`.
