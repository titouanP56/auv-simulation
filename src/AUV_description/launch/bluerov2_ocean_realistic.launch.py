import os
import math
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

# ── Phase 1: Random Spawn helper ─────────────────────────────────────────────
def _random_spawn_in_circle(radius: float):
    """Return (x, y) uniformly sampled inside a circle of given radius."""
    while True:
        x = random.uniform(-radius, radius)
        y = random.uniform(-radius, radius)
        if math.hypot(x, y) <= radius:
            return x, y


def generate_launch_description():
    """
    Launch file for the fully equipped BlueROV2 simulation in a water basin.
    
    This launch file:
    1. Starts Gazebo Harmonic with a specific world.
    2. Spawns the BlueROV2 URDF (equipped with sensors) into the simulation.
    3. Bridges Gazebo topics to ROS 2 topics (Thrusters, Odometry, Camera, Sonars, IMU, Clock).
    4. Launches necessary helper nodes (Robot State Publisher, Depth Sensor Simulator, DVL bridge, IMU repurblisher).
    5. Starts the robot_localization EKF node.
    6. Publishes static transforms to link Gazebo generated frames to the ROS TF tree.
    """
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auv_description = get_package_share_directory('AUV_description')

    sdf_file = os.path.join(pkg_auv_description, 'world', 'ocean_40m.xml')
    urdf_file = os.path.join(pkg_auv_description, 'urdf', 'Bluerov2_realistic.urdf.xml')

    # Parse Xacro/URDF file into an XML string
    xacro_file = Command(['xacro ', urdf_file])

    # 1. Start Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    # ── Phase 1: sample a random realistic spawn pose ─────────────────────────
    # Net diameter = ~25 m radius; AUV can drop anywhere within 20 m of center.
    #NET_SPAWN_RADIUS = 20.0   # [m] max distance from net center
    #NET_CENTER_X     = 0.0    # [m] net center in world frame (adjust if needed)
    #NET_CENTER_Y     = 0.0    # [m] net center in world frame (adjust if needed)

    #spawn_dx, spawn_dy = _random_spawn_in_circle(NET_SPAWN_RADIUS)
    #spawn_x   = NET_CENTER_X + spawn_dx
    #spawn_y   = NET_CENTER_Y + spawn_dy

# Constantes
    NET_SPAWN_RADIUS = 22.0   # Rayon fixe
    NET_CENTER_X     = 0.0
    NET_CENTER_Y     = 0.0

# 1. On choisit un angle aléatoire entre 0 et 2*PI
    angle = random.uniform(0, 2 * math.pi)

# 2. On calcule la position sur la circonférence
    spawn_dx = NET_SPAWN_RADIUS * math.cos(angle)
    spawn_dy = NET_SPAWN_RADIUS * math.sin(angle)

# 3. Position finale dans le monde
    spawn_x = NET_CENTER_X + spawn_dx
    spawn_y = NET_CENTER_Y + spawn_dy


    spawn_z   = random.uniform(-0.5, 0.0)           # near surface
    spawn_yaw = random.uniform(-math.pi, math.pi)   # fully random heading

    print(f"[Phase 1] Spawn pose: x={spawn_x:.2f} y={spawn_y:.2f} "
          f"z={spawn_z:.2f} yaw={math.degrees(spawn_yaw):.1f}°")

    # 2. Spawn the robot model in Gazebo
    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'BlueROV2',
            '-string', xacro_file,
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', str(spawn_z),
            '-Y', str(spawn_yaw),  # Yaw angle (radians) — sets initial heading
        ],
        output='screen'
    )


    # 3. Setup bridges between Gazebo (gz) and ROS 2
    bridge_args = []
    bridge_remappings = []

    # Bridge for 8 thrusters: Gazebo Double -> ROS 2 Float64
    for i in range(1, 9):
        gz_topic = f'/model/BlueROV2/joint/thruster_{i}_joint/cmd_thrust'
        ros_topic = f'/cmd_vel_{i}'
        bridge_args.append(f'{gz_topic}@std_msgs/msg/Float64]gz.msgs.Double')
        bridge_remappings.append((gz_topic, ros_topic))

    # Add Ground Truth Odometry bridge
    bridge_args.append('/model/BlueROV2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')
    bridge_remappings.append(('/model/BlueROV2/odometry', '/odom'))

    # Add Camera Image bridge
    bridge_args.append('/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')
    bridge_remappings.append(('/camera/image_raw', '/camera/image_raw'))

    # Add Clock bridge (Essential for use_sim_time synchronization)
    bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    bridge_remappings.append(('/clock', '/clock'))

    # Add Ping360 Sonar bridge (Gazebo gpu_lidar -> ROS 2 LaserScan)
    bridge_args.append('/ping360/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
    bridge_remappings.append(('/ping360/scan', '/ping360/scan'))

    # Add Sonoptix Echo bridge (Gazebo 2D gpu_lidar -> ROS 2 PointCloud2)
    bridge_args.append('/sonoptix/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
    bridge_remappings.append(('/sonoptix/points', '/sonoptix/points'))

    # Add Raw IMU bridge
    bridge_args.append('/imu@sensor_msgs/msg/Imu[gz.msgs.IMU')
    bridge_remappings.append(('/imu', '/imu'))

    # Execute the bridge node with all configurations
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Helper Nodes
    # Publish URDF links to TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(xacro_file, value_type=str), 'use_sim_time': True}],
    )

    # Simulates a noisy depth sensor from /odom
    simulated_depth_sensor = Node(
        package='AUV_description',
        executable='simulated_depth_sensor',
        name='simulated_depth_sensor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Converts Gazebo's custom DVL messages to ROS 2 TwistWithCovarianceStamped
    dvl_bridge_node = Node(
        package='auv_dvl_bridge',
        executable='dvl_bridge_node',
        name='dvl_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Republish /imu with realistic non-zero covariances (Gazebo publishes all-zero covariances causing EKF issues)
    imu_republisher = Node(
        package='AUV_description',
        executable='imu_republisher',
        name='imu_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Start State Estimation (Extended Kalman Filter)
    pkg_my_auv_localization = get_package_share_directory('my_auv_localization')
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_auv_localization, 'launch', 'localization.launch.py')
        )
    )

    # 6. Static TF Publishers
    # Gazebo plugins often append the model/link name to the frame_id.
    # These static transforms link the ROS URDF frames to the Gazebo frames
    # so data from the sensors is correctly positioned on the robot.

    # Static transform to link URDF's ping360_link with Gazebo's auto-generated frame
    ping360_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ping360_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'ping360_link', 'BlueROV2/base_link/ping360_sonar']
    )

    # Static transform for IMU frame
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'BlueROV2/base_link/imu_sensor']
    )

    # Static transform to link URDF's sonoptix_link with Gazebo's auto-generated frame
    sonoptix_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sonoptix_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'sonoptix_link', 'BlueROV2/base_link/sonoptix_sonar']
    )

    # ── Gazebo Resource Path ─────────────────────────────────────────────
    from launch.actions import SetEnvironmentVariable, TimerAction
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(get_package_share_directory('AUV_description'), '..')]
    )

    # 7. Net Approach Node (delayed: wait for Gazebo + EKF to be ready)
    net_approach = TimerAction(
        period=5.0,  # [s] give Gazebo, bridge, and EKF time to start
        actions=[
            Node(
                package='AUV_guidance',
                executable='net_approach',
                name='net_approach',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        create_entity,
        robot_state_publisher,
        bridge,
        simulated_depth_sensor,
        dvl_bridge_node,
        imu_republisher,
        robot_localization_launch,
        ping360_tf,
        imu_tf,
        sonoptix_tf,
        net_approach,   # Net approach: delayed start
    ])
