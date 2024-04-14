import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    vertices = LaunchConfiguration('vertices', default='False')

    # map_path = os.path.join(get_package_share_directory('mapr_6_student'), 'maps', 'map_small.yaml')
    map_path = os.path.join(get_package_share_directory('mapr_6_student'), 'maps', 'map_medium.yaml')
    # map_path = os.path.join(get_package_share_directory('mapr_6_student'), 'maps', 'map.yaml')

    map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mapr_6_student'), 'launch', 'map_launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )
    
    rrt_cmd = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression(['not ', vertices])),
                package='mapr_6_student',
                executable='rrt',
                name='rrt_node',
                output='screen'),
            Node(
                condition=IfCondition(PythonExpression([vertices])),
                package='mapr_6_student',
                executable='rrt_vertices',
                name='rrt_node',
                output='screen')
        ]
    )
    
    points_cmd = Node(
            package='mapr_6_student',
            executable='points',
            name='points')

    rviz_config_dir = os.path.join(
        get_package_share_directory('mapr_6_student'), 'rviz', 'rviz_cfg.rviz')

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
    ld.add_action(rrt_cmd)
    ld.add_action(points_cmd)
    ld.add_action(rviz_cmd)

    return ld
