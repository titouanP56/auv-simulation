import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_auv_perception = get_package_share_directory('auv_perception')

    sonar_filter_node = Node(
        package='auv_perception',
        executable='sonar_filter_node',
        name='sonar_filter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            os.path.join(pkg_auv_perception, 'config', 'octomap_params.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[
            ('cloud_in', '/sonoptix/points_filtered')
        ]
    )

    auto_saver_node = Node(
        package='auv_perception',
        executable='auto_saver_node',
        name='auto_saver_node',
        output='screen'
    )

    return LaunchDescription([
        sonar_filter_node,
        octomap_node,
        auto_saver_node
    ])
