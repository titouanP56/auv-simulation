import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Launch file for the basic BlueROV2 simulation in a water basin.
    
    This is a simplified version compared to `bluerov2_bassin_captors.launch.py`.
    It only spawns the ROV with its thrusters and basic odometry, without extra 
    sensors like Camera, Sonars, DVL, or IMU. It also does NOT start the EKF.
    """
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auv_description = get_package_share_directory('AUV_description')

    sdf_file = os.path.join(pkg_auv_description, 'world', 'Bassin_ntnu.xml')
    urdf_file = os.path.join(pkg_auv_description, 'urdf', 'BlueROV2.urdf.xml')

    # Parse Xacro/URDF file into an XML string
    xacro_file = Command(['xacro ', urdf_file])

    # 1. Start Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {sdf_file}'}.items(),
    )

    # 2. Spawn the robot model in Gazebo
    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'BlueROV2',
                   '-string', xacro_file,
                   '-z', '-0.3'], # Spawn slightly underwater to avoid floor collision if any
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

    # Add Clock bridge for use_sim_time synchronization
    bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')

    # Execute the bridge node with all configurations
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen'
    )

    # 4. Helper Nodes
    # Publish URDF links to TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(xacro_file, value_type=str)}],
    )

    return LaunchDescription([
        gz_sim,
        create_entity,
        robot_state_publisher,
        bridge,
    ])
