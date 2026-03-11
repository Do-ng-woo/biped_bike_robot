import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'biped_bike_robot'
    pkg_share = get_package_share_directory(package_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'biped_bike_robot.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': False,
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
