import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _scan_bridge_action(scan_topic):
    try:
        get_package_prefix('ros_gz_bridge')
    except PackageNotFoundError:
        return LogInfo(
            msg=(
                'ros_gz_bridge is not installed; boat lidar obstacle avoidance '
                'will run without LaserScan input.'
            )
        )

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='boat_lidar_bridge',
        output='screen',
        arguments=[
            f'{scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )


def _clock_bridge_action():
    try:
        get_package_prefix('ros_gz_bridge')
    except PackageNotFoundError:
        return LogInfo(
            msg=(
                'ros_gz_bridge is not installed; RViz and TF will use wall time.'
            )
        )

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )


def generate_launch_description():
    package_share = get_package_share_directory('uav_usv_sim')
    default_rviz_config = os.path.join(
        package_share,
        'rviz',
        'boat_front_camera.rviz',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'goal_topic',
                default_value='/goal_pose',
                description='PoseStamped topic published by RViz 2D Goal Pose.',
            ),
            DeclareLaunchArgument(
                'pose_topic',
                default_value='/world/default/pose/info',
                description='Gazebo world pose info topic.',
            ),
            DeclareLaunchArgument(
                'boat_cmd_topic',
                default_value='/model/simple_boat/cmd_vel',
                description='Gazebo Twist topic used by the boat velocity controller.',
            ),
            DeclareLaunchArgument(
                'scan_topic',
                default_value='/boat/scan',
                description='Boat lidar LaserScan topic used for local obstacle avoidance.',
            ),
            DeclareLaunchArgument(
                'scan_range_topic',
                default_value='/boat/scan_range',
                description='LaserScan topic used to show the lidar field of view in RViz.',
            ),
            DeclareLaunchArgument(
                'boat_name',
                default_value='landing_boat',
                description='Gazebo model name of the boat.',
            ),
            DeclareLaunchArgument(
                'arrival_radius',
                default_value='0.8',
                description='Goal acceptance radius in meters.',
            ),
            DeclareLaunchArgument(
                'max_speed',
                default_value='2.6',
                description='Maximum forward boat command in meters per second.',
            ),
            DeclareLaunchArgument(
                'max_turn',
                default_value='1.8',
                description='Maximum yaw-rate command in radians per second.',
            ),
            DeclareLaunchArgument(
                'turn_gain',
                default_value='1.5',
                description='Proportional heading controller gain.',
            ),
            DeclareLaunchArgument(
                'enable_avoidance',
                default_value='true',
                description='Enable local lidar obstacle avoidance.',
            ),
            DeclareLaunchArgument(
                'obstacle_slow_distance',
                default_value='18.0',
                description='Start slowing and steering around obstacles inside this distance.',
            ),
            DeclareLaunchArgument(
                'obstacle_stop_distance',
                default_value='4.0',
                description='Stop forward motion when a front obstacle is closer than this.',
            ),
            DeclareLaunchArgument(
                'obstacle_turn_gain',
                default_value='2.8',
                description='Extra turn gain applied while avoiding lidar obstacles.',
            ),
            DeclareLaunchArgument(
                'obstacle_clear_distance',
                default_value='21.0',
                description='Obstacle distance where the avoidance state can clear.',
            ),
            DeclareLaunchArgument(
                'avoidance_hold_time',
                default_value='3.0',
                description='Seconds to keep the selected avoidance side to prevent oscillation.',
            ),
            DeclareLaunchArgument(
                'avoidance_filter_alpha',
                default_value='0.35',
                description='Low-pass filter coefficient for obstacle avoidance turn command.',
            ),
            DeclareLaunchArgument(
                'avoidance_min_speed',
                default_value='0.55',
                description='Minimum forward creep speed during close obstacle avoidance.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz with camera and 2D Goal Pose tools.',
            ),
            DeclareLaunchArgument(
                'marker_topic',
                default_value='/boat/navigation_markers',
                description='MarkerArray topic used to show the reference map in RViz.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz_config,
                description='RViz config used when start_rviz is true.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time for RViz, TF, and LaserScan display.',
            ),
            Node(
                package='uav_usv_sim',
                executable='rviz_goal_boat_control',
                name='rviz_goal_boat_control',
                output='screen',
                parameters=[
                    {
                        'goal_topic': LaunchConfiguration('goal_topic'),
                        'pose_topic': LaunchConfiguration('pose_topic'),
                        'boat_cmd_topic': LaunchConfiguration('boat_cmd_topic'),
                        'scan_topic': LaunchConfiguration('scan_topic'),
                        'scan_range_topic': LaunchConfiguration('scan_range_topic'),
                        'boat_name': LaunchConfiguration('boat_name'),
                        'marker_topic': LaunchConfiguration('marker_topic'),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool,
                        ),
                        'enable_avoidance': ParameterValue(
                            LaunchConfiguration('enable_avoidance'),
                            value_type=bool,
                        ),
                        'arrival_radius': ParameterValue(
                            LaunchConfiguration('arrival_radius'),
                            value_type=float,
                        ),
                        'max_speed': ParameterValue(
                            LaunchConfiguration('max_speed'),
                            value_type=float,
                        ),
                        'max_turn': ParameterValue(
                            LaunchConfiguration('max_turn'),
                            value_type=float,
                        ),
                        'turn_gain': ParameterValue(
                            LaunchConfiguration('turn_gain'),
                            value_type=float,
                        ),
                        'obstacle_slow_distance': ParameterValue(
                            LaunchConfiguration('obstacle_slow_distance'),
                            value_type=float,
                        ),
                        'obstacle_stop_distance': ParameterValue(
                            LaunchConfiguration('obstacle_stop_distance'),
                            value_type=float,
                        ),
                        'obstacle_turn_gain': ParameterValue(
                            LaunchConfiguration('obstacle_turn_gain'),
                            value_type=float,
                        ),
                        'obstacle_clear_distance': ParameterValue(
                            LaunchConfiguration('obstacle_clear_distance'),
                            value_type=float,
                        ),
                        'avoidance_hold_time': ParameterValue(
                            LaunchConfiguration('avoidance_hold_time'),
                            value_type=float,
                        ),
                        'avoidance_filter_alpha': ParameterValue(
                            LaunchConfiguration('avoidance_filter_alpha'),
                            value_type=float,
                        ),
                        'avoidance_min_speed': ParameterValue(
                            LaunchConfiguration('avoidance_min_speed'),
                            value_type=float,
                        ),
                    }
                ],
            ),
            _clock_bridge_action(),
            _scan_bridge_action('/boat/scan'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='uav_usv_goal_rviz',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')],
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
        ]
    )
