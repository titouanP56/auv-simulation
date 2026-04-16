import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_auv_description = get_package_share_directory('AUV_description')
    pkg_auv_controller = get_package_share_directory('AUV_controller')

    # Argument for mode selection
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='reactive',
        description='Mode for trajectory generation: reactive or lawnmower'
    )
    mode = LaunchConfiguration('mode')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo Sim without GUI (server only)'
    )
    headless = LaunchConfiguration('headless')

    # 1. Base simulation launch
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_auv_description, 'launch', 'bluerov2_ocean_realistic.launch.py')
        ),
        launch_arguments={
            'headless': headless,
            'world_file': 'small_net.xml'
        }.items(),
    )

    # Perception Node for Net Inspection
    perception_node = Node(
        package='auv_perception',
        executable='net_local_estimator',
        name='net_local_estimator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2. Lawnmower Trajectory Node
    trajectory_node = Node(
        package='AUV_guidance',
        executable='lawnmower_trajectory_node',
        name='lawnmower_trajectory_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'enabled': PythonExpression(["True if '", mode, "' == 'lawnmower' else False"])}
        ]
    )

    # Reactive Wall Follower Node
    reactive_node = Node(
        package='AUV_guidance',
        executable='reactive_wall_follower',
        name='reactive_wall_follower',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'enabled': PythonExpression(["True if '", mode, "' == 'reactive' else False"])},
            {'target_distance': 2.0},
            {'nominal_velocity': 0.3},
            {'circle_center_x': 0.0},
            {'circle_center_y': 0.0},
            {'circle_radius': 5.0},
            {'lookahead_time': 2.0}
        ]
    )

    # Wall Following Node (Hough + B-Spline — Chou et al. / Ghorbani)
    wall_following_node = Node(
        package='AUV_guidance',
        executable='wall_following_node_petit',
        name='wall_following_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'enabled': PythonExpression(["True if '", mode, "' == 'wall_following' else False"])},
            {'net_radius': 5.0},
            {'target_distance': 1.5},
            {'nominal_velocity': 0.3},
            {'lookahead_time': 2.0},
            {'gain_distance': 0.8},
            {'gain_angle': 1.0},
            {'hough_threshold': 10},
            {'spline_buffer_size': 20},
        ]
    )

    # 3. MPC Controller Node
    mpc_node = Node(
        package='AUV_controller',
        executable='mpc_controller_net_inspection',
        name='mpc_controller_net_inspection',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Delay the start of Phase 4 nodes to ensure EKF and Gazebo are ready
    delayed_phase4 = TimerAction(
        period=10.0,
        actions=[
            perception_node, trajectory_node, reactive_node,
            wall_following_node, mpc_node,
        ]
    )

    return LaunchDescription([
        mode_arg,
        headless_arg,
        base_sim,
        delayed_phase4
    ])
