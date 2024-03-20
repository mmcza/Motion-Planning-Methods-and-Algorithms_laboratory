import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    backtrace = LaunchConfiguration('backtrace', default='False')

    map_path = os.path.join(get_package_share_directory('mapr_4_student'), 'maps', 'map_small.yaml')
    # map_path = os.path.join(get_package_share_directory('mapr_4_student'), 'maps', 'map.yaml')

    map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mapr_4_student'), 'launch', 'map_launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )


    graph_search = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression(['not ', backtrace])),
                package='mapr_4_student',
                executable='dfs',
                name='graph_search',
                output='screen'),
            Node(
                condition=IfCondition(PythonExpression([backtrace])),
                package='mapr_4_student',
                executable='dfs_backtrace',
                name='graph_search',
                output='screen')
        ]
    )
    
    points_cmd = Node(
            package='mapr_4_student',
            executable='points',
            name='points')


    rviz_config_dir = os.path.join(
        get_package_share_directory('mapr_4_student'), 'rviz', 'rviz_cfg.rviz')

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
    ld.add_action(graph_search)
    ld.add_action(points_cmd)
    ld.add_action(rviz_cmd)

    return ld
