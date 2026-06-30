from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    auto_ownship = LaunchConfiguration('auto_ownship')
    start_rviz = LaunchConfiguration('start_rviz')
    return LaunchDescription(
        [
            DeclareLaunchArgument('scenario', default_value='head_on'),
            DeclareLaunchArgument('auto_ownship', default_value='true'),
            DeclareLaunchArgument('start_rviz', default_value='true'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('uav_usv_sim'),
                            'launch',
                            'colregs_test_scenario.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'scenario': scenario,
                    'auto_ownship': auto_ownship,
                    'start_rviz': start_rviz,
                }.items(),
            ),
        ]
    )
