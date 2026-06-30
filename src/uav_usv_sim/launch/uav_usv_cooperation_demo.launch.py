from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('uav_usv_sim')
    px4_dir = LaunchConfiguration('px4_dir')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    target_radius = LaunchConfiguration('target_radius')
    drone_depart_delay = LaunchConfiguration('drone_depart_delay')
    deck_hover_altitude = LaunchConfiguration('deck_hover_altitude')
    deck_lock_radius = LaunchConfiguration('deck_lock_radius')
    deck_land_altitude = LaunchConfiguration('deck_land_altitude')
    deck_align_radius = LaunchConfiguration('deck_align_radius')
    deck_settle_time = LaunchConfiguration('deck_settle_time')
    deck_descent_rate = LaunchConfiguration('deck_descent_rate')
    deck_touchdown_tolerance = LaunchConfiguration('deck_touchdown_tolerance')
    deck_touchdown_hold_time = LaunchConfiguration('deck_touchdown_hold_time')
    takeoff_climb_rate = LaunchConfiguration('takeoff_climb_rate')
    drone_cruise_speed = LaunchConfiguration('drone_cruise_speed')
    drone_deck_approach_speed = LaunchConfiguration('drone_deck_approach_speed')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'px4_dir',
                default_value=EnvironmentVariable(
                    'PX4_DIR',
                    default_value='/home/dji/PX4-Autopilot',
                ),
                description='PX4-Autopilot source directory. Change this to your local PX4 path.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz with the boat front camera display.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [package_share, 'rviz', 'boat_front_camera.rviz']
                ),
                description='RViz config used when start_rviz is true.',
            ),
            DeclareLaunchArgument(
                'target_radius',
                default_value='5.0',
                description='Fixed arrival and UAV standoff radius around the lighthouse in meters.',
            ),
            DeclareLaunchArgument(
                'drone_depart_delay',
                default_value='10.0',
                description='Seconds the drone waits after takeoff before flying to the lighthouse.',
            ),
            DeclareLaunchArgument(
                'deck_hover_altitude',
                default_value='4.0',
                description='Drone hover altitude above the landing pad area before landing command.',
            ),
            DeclareLaunchArgument(
                'deck_lock_radius',
                default_value='0.25',
                description='Setpoint distance threshold before final deck lock can be considered.',
            ),
            DeclareLaunchArgument(
                'deck_align_radius',
                default_value='0.8',
                description='Distance where the drone pauses above the landing pad before descent.',
            ),
            DeclareLaunchArgument(
                'deck_settle_time',
                default_value='1.5',
                description='Seconds to hold above the landing pad before vertical descent.',
            ),
            DeclareLaunchArgument(
                'deck_descent_rate',
                default_value='0.35',
                description='Final vertical descent rate in meters per second.',
            ),
            DeclareLaunchArgument(
                'deck_touchdown_tolerance',
                default_value='0.12',
                description='Gazebo position tolerance required before deck lock.',
            ),
            DeclareLaunchArgument(
                'deck_touchdown_hold_time',
                default_value='0.8',
                description='Seconds to hold touchdown pose before deck lock.',
            ),
            DeclareLaunchArgument(
                'deck_land_altitude',
                default_value='0.1',
                description='Height above the deck surface where the drone is relocked.',
            ),
            DeclareLaunchArgument(
                'takeoff_climb_rate',
                default_value='0.8',
                description='Vertical takeoff climb rate in meters per second.',
            ),
            DeclareLaunchArgument(
                'drone_cruise_speed',
                default_value='2.0',
                description='Drone horizontal cruise speed limit in meters per second.',
            ),
            DeclareLaunchArgument(
                'drone_deck_approach_speed',
                default_value='1.2',
                description='Drone speed limit when approaching the deck hover point.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, 'launch', 'uav_usv_px4_sim.launch.py']
                    )
                ),
                launch_arguments={
                    'px4_dir': px4_dir,
                    'start_rviz': start_rviz,
                    'rviz_config': rviz_config,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            package_share,
                            'launch',
                            'cooperative_lighthouse_mission.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'target_radius': target_radius,
                    'drone_depart_delay': drone_depart_delay,
                    'deck_hover_altitude': deck_hover_altitude,
                    'deck_lock_radius': deck_lock_radius,
                    'deck_land_altitude': deck_land_altitude,
                    'deck_align_radius': deck_align_radius,
                    'deck_settle_time': deck_settle_time,
                    'deck_descent_rate': deck_descent_rate,
                    'deck_touchdown_tolerance': deck_touchdown_tolerance,
                    'deck_touchdown_hold_time': deck_touchdown_hold_time,
                    'takeoff_climb_rate': takeoff_climb_rate,
                    'drone_cruise_speed': drone_cruise_speed,
                    'drone_deck_approach_speed': drone_deck_approach_speed,
                }.items(),
            ),
        ]
    )
