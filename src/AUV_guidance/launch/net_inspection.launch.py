import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_auv_description = get_package_share_directory('AUV_description')
    pkg_auv_controller = get_package_share_directory('AUV_controller')

    # 1. Base simulation launch (Gazebo, Robot, Bridge, EKF)
    # On utilise IncludeLaunchDescription avec un filtre sur les nœuds si possible, 
    # ou on s'assure que les ressources sont suffisantes.
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_auv_description, 'launch', 'bluerov2_ocean_realistic.launch.py')
        )
    )

    # Pour réduire la charge, on peut essayer de ne pas lancer phase2_mission
    # mais comme il est dans le fichier inclus, on va ajuster le délai de la phase 4.


    # 2. Trajectory Generator Node
    trajectory_node = Node(
        package='AUV_guidance',
        executable='lawnmower_trajectory_node',
        name='lawnmower_trajectory_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. MPC Controller Node (Phase 4 version)
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
        actions=[trajectory_node, mpc_node]
    )

    return LaunchDescription([
        base_sim,
        delayed_phase4
    ])
