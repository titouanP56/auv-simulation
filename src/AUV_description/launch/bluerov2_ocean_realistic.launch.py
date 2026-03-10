import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_auv_description = get_package_share_directory('AUV_description')
    pkg_ros_gz_sim      = get_package_share_directory('ros_gz_sim')
    pkg_localization    = get_package_share_directory('my_auv_localization')

    # ── URDF / XACRO ────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_auv_description, 'urdf', 'bluerov2_realistic.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # ── Gazebo simulation ────────────────────────────────────────────────
    # Using the 40m deep ocean world
    world_file = os.path.join(pkg_auv_description, 'world', 'ocean_40m.xml')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Adding -v 4 for verbose output to help debugging if needed
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # ── Spawn robot ──────────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', doc.toxml(),
                   '-name', 'bluerov2_realistic',
                   '-allow_renaming', 'true',
                   # Spawn at z=-1 to be underwater
                   '-x', '0.0', '-y', '0.0', '-z', '-1'],
        output='screen'
    )

    # ── Robot State Publisher ────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── Gazebo ↔ ROS Bridges ─────────────────────────────────────────────
    bridge_args = []
    bridge_remappings = []

    # Thruster commands (ROS → Gz)
    for i in range(1, 9):
        gz_topic  = f'/model/bluerov2_realistic/joint/thruster_{i}_joint/cmd_thrust'
        ros_topic = f'/cmd_vel_{i}'
        bridge_args.append(f'{gz_topic}@std_msgs/msg/Float64]gz.msgs.Double')
        bridge_remappings.append((gz_topic, ros_topic))

    # Odometry (Gz → ROS /odom)
    bridge_args.append('/model/bluerov2_realistic/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')
    bridge_remappings.append(('/model/bluerov2_realistic/odometry', '/odom'))

    # TF (Gz → ROS)
    bridge_args.append('/model/bluerov2_realistic/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
    bridge_remappings.append(('/model/bluerov2_realistic/tf', '/tf'))

    # IMU (Gz → ROS)
    bridge_args.append('/model/bluerov2_realistic/imu@sensor_msgs/msg/Imu[gz.msgs.IMU')
    bridge_remappings.append(('/model/bluerov2_realistic/imu', '/imu/fixed'))

    # Clock (Gz → ROS)
    bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── DVL bridge ──────────────────────────────────────────────────────
    dvl_bridge = Node(
        package='auv_dvl_bridge',
        executable='dvl_bridge_node',
        name='dvl_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── Simulated Depth Sensor ──────────────────────────────────────────
    depth_sensor = Node(
        package='AUV_description', # Note: logic indicates it's in this pkg
        executable='simulated_depth_sensor',
        name='simulated_depth_sensor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── EKF (robot_localization) ─────────────────────────────────────────
    ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    # ── Static Transforms ────────────────────────────────────────────────
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # ── Gazebo Resource Path ─────────────────────────────────────────────
    # Necessary for Gazebo to find meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(get_package_share_directory('AUV_description'), '..')]
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        spawn_entity,
        robot_state_publisher,
        bridge,
        dvl_bridge,
        depth_sensor,
        ekf_node,
        static_tf_world_odom,
    ])
