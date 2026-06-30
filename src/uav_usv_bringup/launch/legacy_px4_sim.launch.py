from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    start_rviz = LaunchConfiguration('start_rviz')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'px4_dir',
                default_value=EnvironmentVariable(
                    'PX4_DIR',
                    default_value='/home/dji/PX4-Autopilot',
                ),
            ),
            DeclareLaunchArgument('start_rviz', default_value='false'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('uav_usv_sim'),
                            'launch',
                            'uav_usv_px4_sim.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'px4_dir': px4_dir,
                    'start_rviz': start_rviz,
                }.items(),
            ),
        ]
    )
