"""
net_full_inspection.launch.py
==============================
Full Phase 2 → Phase 3 mission launch.

What this file does
--------------------
1. Starts Gazebo Harmonic with small_net.xml   (GUI toggle: headless:=True/False)
2. Spawns BlueROV2 at a **random point on a circle of radius 2 m** at depth -1 m
3. Starts all sensor bridges, EKF, TF publishers, DVL, depth-sensor, IMU bridges
4. Starts ``net_approach``  (Phase 2) after a startup delay
5. Starts ``phase3_inspection`` (Phase 3) after a slightly longer delay
   — Phase 3 node waits internally for /mission/phase2_done before acting
6. Optionally starts RViz2                     (rviz:=True/False)

Launch arguments
----------------
  headless   False   Set True to run Gazebo without a graphical window
  rviz       False   Set True to open RViz2 for sensor/TF visualisation
  world_file small_net.xml   Name of the world file inside AUV_description/world/
  gz_delay   5.0     Seconds to wait after Gazebo starts before spawning nodes

Usage
-----
  ros2 launch AUV_guidance net_full_inspection.launch.py
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True
  ros2 launch AUV_guidance net_full_inspection.launch.py rviz:=True
  ros2 launch AUV_guidance net_full_inspection.launch.py headless:=True rviz:=False
"""

import math
import os
import random

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


# ── Package paths ─────────────────────────────────────────────────────────────

PKG_DESC   = get_package_share_directory('AUV_description')
PKG_LOC    = get_package_share_directory('my_auv_localization')
PKG_GZ_SIM = get_package_share_directory('ros_gz_sim')


# ── Random spawn on circle ────────────────────────────────────────────────────

_SPAWN_RADIUS = 3.4    # [m]  circle radius
_SPAWN_DEPTH  = -2   # [m]  constant depth (negative = underwater in NED-like frame)

_angle   = random.uniform(0.0, 2.0 * math.pi)
_spawn_x = _SPAWN_RADIUS * math.cos(_angle)
_spawn_y = _SPAWN_RADIUS * math.sin(_angle)
_spawn_z = _SPAWN_DEPTH
# Point the robot's nose toward the net (outward from origin by default)
_spawn_yaw = _angle  # [rad]  robot faces away from centre → toward net

print(
    f"[net_full_inspection] Spawn pose: "
    f"x={_spawn_x:.2f} m  y={_spawn_y:.2f} m  z={_spawn_z:.2f} m  "
    f"yaw={math.degrees(_spawn_yaw):.1f}°  (angle={math.degrees(_angle):.1f}°)"
)


# ── Main launch description ───────────────────────────────────────────────────

def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo without a GUI (server-only). True | False',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch RViz2 for sensor/TF visualisation. True | False',
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='small_net_deforme.xml',
        description='World file name (must live in AUV_description/world/)',
    )

    gz_delay_arg = DeclareLaunchArgument(
        'gz_delay',
        default_value='8.0',
        description='Seconds to wait after Gazebo starts before activating mission nodes',
    )

    headless    = LaunchConfiguration('headless')
    rviz        = LaunchConfiguration('rviz')
    world_file  = LaunchConfiguration('world_file')
    gz_delay    = LaunchConfiguration('gz_delay')

    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='False',
        description='Launch MAVROS and bluerov2_bridge instead of Gazebo sim',
    )

    use_hardware = LaunchConfiguration('use_hardware')

    # ── Gazebo resource path ──────────────────────────────────────────────────

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(PKG_DESC, '..')],
    )

    # ── Gazebo sim ────────────────────────────────────────────────────────────

    sdf_file = PythonExpression([
        "'", os.path.join(PKG_DESC, 'world'), "/' + '", world_file, "'"
    ])

    gz_args = PythonExpression([
        "'-r ", sdf_file, " -s' if ", headless, " else '-r ", sdf_file, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG_GZ_SIM, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=UnlessCondition(use_hardware),
    )

    # ── URDF / robot description ──────────────────────────────────────────────

    urdf_file  = os.path.join(PKG_DESC, 'urdf', 'Bluerov2_realistic.urdf.xml')
    xacro_file = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': ParameterValue(xacro_file, value_type=str),
            'use_sim_time': True,
        }],
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────

    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'BlueROV2',
            '-string', xacro_file,
            '-x', str(_spawn_x),
            '-y', str(_spawn_y),
            '-z', str(_spawn_z),
            '-Y', str(_spawn_yaw),
        ],
        output='screen',
        condition=UnlessCondition(use_hardware),
    )

    # ── Sensor + topic bridges (Gazebo ↔ ROS 2) ───────────────────────────────

    bridge_args = []
    bridge_remappings = []

    # 8 thrusters
    for i in range(1, 9):
        gz_topic  = f'/model/BlueROV2/joint/thruster_{i}_joint/cmd_thrust'
        ros_topic = f'/cmd_vel_{i}'
        bridge_args.append(f'{gz_topic}@std_msgs/msg/Float64]gz.msgs.Double')
        bridge_remappings.append((gz_topic, ros_topic))

    # Ground-truth odometry
    bridge_args.append('/model/BlueROV2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')
    bridge_remappings.append(('/model/BlueROV2/odometry', '/odom'))

    # Camera
    bridge_args.append('/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')
    bridge_remappings.append(('/camera/image_raw', '/camera/image_raw'))

    # Sim clock (essential for use_sim_time)
    bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    bridge_remappings.append(('/clock', '/clock'))

    # Ping360 sonar
    bridge_args.append('/ping360/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
    bridge_remappings.append(('/ping360/scan', '/ping360/scan'))

    # Sonoptix Echo (3-D sonar)
    bridge_args.append('/sonoptix/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
    bridge_remappings.append(('/sonoptix/points', '/sonoptix/points'))

    # IMU
    bridge_args.append('/imu@sensor_msgs/msg/Imu[gz.msgs.IMU')
    bridge_remappings.append(('/imu', '/imu'))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_hardware),
    )

    # ── Helper sensor nodes ───────────────────────────────────────────────────

    simulated_depth_sensor = Node(
        package='AUV_description',
        executable='simulated_depth_sensor',
        name='simulated_depth_sensor',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_hardware),
    )

    dvl_bridge = Node(
        package='auv_dvl_bridge',
        executable='dvl_bridge_node',
        name='dvl_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_hardware),
    )

    imu_republisher = Node(
        package='AUV_description',
        executable='imu_republisher',
        name='imu_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_hardware),
    )

    sim_thruster_bridge_node = Node(
        package='AUV_guidance',
        executable='sim_thruster_bridge',
        name='sim_thruster_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_hardware),
    )

    # ── EKF localisation ──────────────────────────────────────────────────────

    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG_LOC, 'launch', 'localization.launch.py')
        ),
    )

    # ── Static TF publishers ──────────────────────────────────────────────────

    ping360_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ping360_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'ping360_link', 'BlueROV2/base_link/ping360_sonar',
        ],
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'base_link', 'BlueROV2/base_link/imu_sensor',
        ],
    )

    sonoptix_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sonoptix_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'sonoptix_link', 'BlueROV2/base_link/sonoptix_sonar',
        ],
    )

    # ── MAVROS & Hardware Bridge ──────────────────────────────────────────────

    try:
        mavros_pkg_path = get_package_share_directory('mavros')
    except Exception:
        mavros_pkg_path = ''

    mavros_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavros_pkg_path, 'launch', 'node.launch.py')
        ),
        launch_arguments={
            'fcu_url': 'udp://192.168.2.1:14550@192.168.2.2:14555',
            'gcs_url': 'udp://@localhost:14550',
            'tgt_system': '1',
            'tgt_component': '1',
        }.items(),
        condition=IfCondition(use_hardware),
    )

    bluerov2_bridge_node = Node(
        package='AUV_guidance',
        executable='bluerov2_bridge',
        name='bluerov2_bridge',
        output='screen',
        condition=IfCondition(use_hardware),
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────────

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    # ── Mission nodes (delayed) ───────────────────────────────────────────────
    # Both nodes are launched together after gz_delay seconds.
    # - net_approach (Phase 2) starts immediately and runs its state machine.
    # - phase3_inspection (Phase 3) starts at the same time but stays WAITING
    #   until it receives True on /mission/phase2_done — no extra delay needed.

    net_approach_node = Node(
        package='AUV_guidance',
        executable='net_approach',
        name='net_approach',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    phase3_node = Node(
        package='AUV_guidance',
        executable='phase3_inspection',
        name='phase3_inspection',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_mission = TimerAction(
        period=gz_delay,
        actions=[net_approach_node, phase3_node],
    )

    # ── Assembly ──────────────────────────────────────────────────────────────

    return LaunchDescription([
        # Arguments first
        headless_arg,
        rviz_arg,
        world_file_arg,
        gz_delay_arg,
        use_hardware_arg,

        # Environment
        gz_resource_path,

        # Simulation
        gz_sim,
        robot_state_publisher,
        create_entity,

        # Bridges & helpers
        bridge,
        simulated_depth_sensor,
        dvl_bridge,
        imu_republisher,
        sim_thruster_bridge_node,

        # Hardware MAVROS
        mavros_node,
        bluerov2_bridge_node,

        # Localisation
        robot_localization,

        # TF
        ping360_tf,
        imu_tf,
        sonoptix_tf,

        # Optional visualisation
        rviz_node,

        # Mission (delayed)
        delayed_mission,
    ])
