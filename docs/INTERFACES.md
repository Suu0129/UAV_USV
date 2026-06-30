# System Interface Contract

Changes to this document are architecture changes and require integration review.

## 1. TF contract

Production tree:

```text
map
└── odom
    └── landing_boat/base_link
        ├── landing_boat/mid360_link
        ├── landing_boat/imu_link
        ├── landing_boat/front_camera_link
        └── landing_boat/landing_pad_link
```

Ownership:

- `map -> odom`: localization package.
- `odom -> landing_boat/base_link`: odometry/localization package.
- Sensor fixed transforms: description plus `robot_state_publisher`.
- Gazebo truth TF is permitted only in simulation mode.

One TF edge must have exactly one publisher.

## 2. Core topics

| Topic | Type | Owner |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | navigation |
| `/odom` | `nav_msgs/Odometry` | localization |
| `/map` | `nav_msgs/OccupancyGrid` | mapping/localization |
| `/boat/scan` | `sensor_msgs/LaserScan` | simulation or sensor driver |
| `/maritime/tracks/lidar` | `uav_usv_interfaces/TrackedObjectArray` | perception |
| `/maritime/tracks/camera` | `uav_usv_interfaces/TrackedObjectArray` | perception |
| `/maritime/tracks/ais` | `uav_usv_interfaces/TrackedObjectArray` | perception |
| `/maritime/tracks/fused` | `uav_usv_interfaces/TrackedObjectArray` | fusion |
| `/maritime/collision_risks` | `uav_usv_interfaces/CollisionRisk` | COLREGs |
| `/maritime/colregs_decisions` | `uav_usv_interfaces/ColregsDecision` | COLREGs |

## 3. Coordinate and unit conventions

- ROS map coordinates: ENU, metres, radians, SI units.
- Positive yaw: counter-clockwise around +Z.
- AIS COG/heading: clockwise from true north, radians.
- Convert AIS conventions before publishing a `TrackedObject.pose`.
- Timestamps use the original measurement time, not processing completion time.

## 4. Control ownership

At runtime only one node may command the USV execution topic.

Control chain:

```text
mission/COLREGs constraints
    -> navigation
    -> /cmd_vel
    -> USV control
    -> simulated or real actuators
```

Perception, localization, and COLREGs packages must not publish actuator commands directly.

## 5. Interface compatibility

- Additive message changes require a minor interface version update.
- Removing or changing field meaning requires a coordinated major update.
- New publishers should provide a mock or rosbag so downstream teams can develop before hardware is ready.
