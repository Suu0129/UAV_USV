from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription(
        [
            DeclareLaunchArgument('start_rviz', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('uav_usv_sim'),
                            'launch',
                            'boat_nav2_navigation.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'start_rviz': start_rviz,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]
    )
