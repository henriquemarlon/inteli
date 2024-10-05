import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rviz_dir = get_package_share_directory('turtlebot3_navigation2')
    map_file_path = os.path.join(os.getcwd(), 'assets', 'map.yaml')
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            rviz_dir + '/launch/navigation2.launch.py'),
        launch_arguments={
            "map": f'{map_file_path}',
            "use_sim_time": 'true',
                   "log_level": "INFO"
        }.items()
    )

    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_dir + '/launch/turtlebot3_world.launch.py')
    )

    return LaunchDescription([
        rviz,
        gazebo_launch,
        Node(
            package='intelligible',
            executable='prometheus',
            output='screen',
            name="prometheus"
        ),
        Node(
            package='intelligible',
            executable='theseus_ship',
            output='screen',
            name="theseus_ship",
        ),
        Node(
            package='intelligible',
            executable='hermes',
            output='screen',
            name="hermes",
        ),
        Node(
            package='intelligible',
            executable='oracle',
            output='screen',
            prefix="xterm -e",
            name="oracle"
        ),
    ])
