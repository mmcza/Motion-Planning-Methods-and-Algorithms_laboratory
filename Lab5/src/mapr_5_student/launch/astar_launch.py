import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    map_path = os.path.join(get_package_share_directory('mapr_5_student'), 'maps', 'map_small.yaml')
    # map_path = os.path.join(get_package_share_directory('mapr_5_student'), 'maps', 'map.yaml')

    map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mapr_5_student'), 'launch', 'map_launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

    astar_cmd = Node(
                package='mapr_5_student',
                executable='astar',
                name='astar_node',
                output='screen')
    
    points_cmd = Node(
            package='mapr_5_student',
            executable='points',
            name='points')

    rviz_config_dir = os.path.join(
        get_package_share_directory('mapr_5_student'), 'rviz', 'rviz_cfg.rviz')

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen')

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(map_server_cmd)
    ld.add_action(astar_cmd)
    ld.add_action(points_cmd)
    ld.add_action(rviz_cmd)

    return ld
