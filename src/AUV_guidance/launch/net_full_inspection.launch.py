

import math
import os
import random
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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


# ── Spawn coordinates ────────────────────────────────────────────────────

_SPAWN_RADIUS = 3.4    # [m]  circle radius
_SPAWN_DEPTH  = -2   # [m]  depth

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
        default_value='small_net.xml',
        description='World file name (must live in AUV_description/world/)',
    )

    gz_delay_arg = DeclareLaunchArgument(
        'gz_delay',
        default_value='8.0',
        description='Seconds to wait after Gazebo starts before activating mission nodes',
    )

    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='False',
        description='Launch MAVROS and bluerov2_bridge instead of Gazebo sim',
    )

    optimize_arg = DeclareLaunchArgument(
        'optimize',
        default_value='False',
        description='Performance mode: coarser physics step (0.006 vs 0.001), '
                    'lighter/slower sensors, slower control loops (5 Hz vs 10 Hz). True | False',
    )

    headless    = LaunchConfiguration('headless')
    rviz        = LaunchConfiguration('rviz')
    world_file  = LaunchConfiguration('world_file')
    gz_delay    = LaunchConfiguration('gz_delay')
    use_hardware = LaunchConfiguration('use_hardware')
    optimize     = LaunchConfiguration('optimize')

    # ── Gazebo resource path ──────────────────────────────────────────────────

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(PKG_DESC, '..')],
    )

    # ── Gazebo sim (step size set by 'optimize' flag) ─────────────────────────

    def _launch_gazebo(context):
        """Build the Gazebo action with max_step_size chosen by the optimize flag."""
        use_hw = context.launch_configurations.get('use_hardware', 'False')
        if use_hw.lower() in ('true', '1'):
            return []

        opt = context.launch_configurations.get('optimize', 'False')
        is_opt = opt.lower() in ('true', '1')
        world_name = context.launch_configurations['world_file']
        headless_val = context.launch_configurations['headless']
        is_headless = headless_val.lower() in ('true', '1')

        # Read world file and patch physics step size
        world_path = os.path.join(PKG_DESC, 'world', world_name)
        step = '0.006' if is_opt else '0.001'
        with open(world_path) as f:
            content = f.read()
        content = re.sub(
            r'<max_step_size>[^<]+</max_step_size>',
            f'<max_step_size>{step}</max_step_size>',
            content,
        )
        fd, tmp_path = tempfile.mkstemp(suffix='.xml', prefix='gz_world_')
        with os.fdopen(fd, 'w') as f:
            f.write(content)

        gz_arg_str = f'-r {tmp_path} -s' if is_headless else f'-r {tmp_path}'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(PKG_GZ_SIM, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_arg_str}.items(),
        )]

    gz_sim = OpaqueFunction(function=_launch_gazebo)

    # ── URDF / robot description ──────────────────────────────────────────────

    urdf_file  = os.path.join(PKG_DESC, 'urdf', 'Bluerov2_realistic.urdf.xml')
    xacro_file = Command(['xacro ', urdf_file, ' optimize:=', optimize])

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
    # bridge_args.append('/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')
    # bridge_remappings.append(('/camera/image_raw', '/camera/image_raw'))

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


    control_rate = PythonExpression(["5.0 if '", optimize, "'.lower() in ('true', '1') else 20.0"])
    yaw_ema_alpha_val = PythonExpression(["1.0 if '", optimize, "'.lower() in ('true', '1') else 0.15"])

    net_approach_node = Node(
        package='AUV_guidance',
        executable='net_approach',
        name='net_approach',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_rate_hz': ParameterValue(control_rate, value_type=float),
        }],
    )

    phase3_node = Node(
        package='AUV_guidance',
        executable='phase3_inspection',
        name='phase3_inspection',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_rate_hz': ParameterValue(control_rate, value_type=float),
            'yaw_ema_alpha': ParameterValue(yaw_ema_alpha_val, value_type=float),
        }],
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
        optimize_arg,

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
