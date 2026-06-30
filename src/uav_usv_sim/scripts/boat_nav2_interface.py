#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist as RosTwist
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.transport13 import Node as GzTransportNode
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, lower, upper):
    return max(lower, min(value, upper))

class VelocityPid:
    """PID correction around a velocity feed-forward command."""

    def __init__(self, kp, ki, kd, integral_limit, derivative_alpha):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.derivative_alpha = clamp(derivative_alpha, 0.0, 1.0)
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return self.kp * error

        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )

        raw_derivative = 0.0
        if self.previous_error is not None:
            raw_derivative = (error - self.previous_error) / dt
        self.previous_error = error
        self.filtered_derivative += self.derivative_alpha * (
            raw_derivative - self.filtered_derivative
        )

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )


class BoatNav2Interface(Node):
    """Adapts the Gazebo boat simulation to the ROS interfaces Nav2 expects."""

    def __init__(self):
        super().__init__('boat_nav2_interface')

        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter('model_pose_topic', '/boat/pose')
        self.declare_parameter('boat_cmd_topic', '/model/simple_boat/cmd_vel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/boat/scan')
        self.declare_parameter('scan_range_topic', '/boat/scan_range')
        self.declare_parameter('boat_name', 'landing_boat')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'landing_boat/base_link')
        self.declare_parameter('lidar_frame_id', 'landing_boat/hull/front_lidar')
        self.declare_parameter('lidar_offset_x', 0.55)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 1.55)
        self.declare_parameter('map_resolution', 0.5)
        self.declare_parameter('map_width', 240.0)
        self.declare_parameter('map_height', 180.0)
        self.declare_parameter('map_publish_period', 2.0)
        self.declare_parameter('cmd_timeout', 0.8)
        self.declare_parameter('marker_topic', '/boat/nav2_reference_markers')
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('enable_velocity_pid', True)
        self.declare_parameter('linear_kp', 0.18)
        self.declare_parameter('linear_ki', 0.03)
        self.declare_parameter('linear_kd', 0.01)
        self.declare_parameter('linear_integral_limit', 0.5)
        self.declare_parameter('angular_kp', 0.28)
        self.declare_parameter('angular_ki', 0.04)
        self.declare_parameter('angular_kd', 0.015)
        self.declare_parameter('angular_integral_limit', 0.8)
        self.declare_parameter('derivative_filter_alpha', 0.2)
        self.declare_parameter('velocity_measurement_alpha', 0.3)
        self.declare_parameter('linear_setpoint_alpha', 0.45)
        self.declare_parameter('angular_setpoint_alpha', 0.6)
        self.declare_parameter('max_linear_output', 1.4)
        self.declare_parameter('max_angular_output', 2.2)
        self.declare_parameter('command_deadband', 0.01)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.model_pose_topic = self.get_parameter('model_pose_topic').value
        self.boat_cmd_topic = self.get_parameter('boat_cmd_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_range_topic = self.get_parameter('scan_range_topic').value
        self.boat_name = self.get_parameter('boat_name').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.lidar_offset_x = float(self.get_parameter('lidar_offset_x').value)
        self.lidar_offset_y = float(self.get_parameter('lidar_offset_y').value)
        self.lidar_offset_z = float(self.get_parameter('lidar_offset_z').value)
        self.map_resolution = float(self.get_parameter('map_resolution').value)
        self.map_width = float(self.get_parameter('map_width').value)
        self.map_height = float(self.get_parameter('map_height').value)
        self.map_publish_period = float(
            self.get_parameter('map_publish_period').value
        )
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.marker_topic = self.get_parameter('marker_topic').value
        self.control_frequency = max(
            1.0,
            float(self.get_parameter('control_frequency').value),
        )
        self.enable_velocity_pid = bool(
            self.get_parameter('enable_velocity_pid').value
        )
        derivative_filter_alpha = float(
            self.get_parameter('derivative_filter_alpha').value
        )
        self.velocity_measurement_alpha = clamp(
            float(self.get_parameter('velocity_measurement_alpha').value),
            0.01,
            1.0,
        )
        self.linear_setpoint_alpha = clamp(
            float(self.get_parameter('linear_setpoint_alpha').value),
            0.01,
            1.0,
        )
        self.angular_setpoint_alpha = clamp(
            float(self.get_parameter('angular_setpoint_alpha').value),
            0.01,
            1.0,
        )
        self.max_linear_output = max(
            0.0,
            float(self.get_parameter('max_linear_output').value),
        )
        self.max_angular_output = max(
            0.0,
            float(self.get_parameter('max_angular_output').value),
        )
        self.command_deadband = max(
            0.0,
            float(self.get_parameter('command_deadband').value),
        )
        self.linear_pid = VelocityPid(
            float(self.get_parameter('linear_kp').value),
            float(self.get_parameter('linear_ki').value),
            float(self.get_parameter('linear_kd').value),
            float(self.get_parameter('linear_integral_limit').value),
            derivative_filter_alpha,
        )
        self.angular_pid = VelocityPid(
            float(self.get_parameter('angular_kp').value),
            float(self.get_parameter('angular_ki').value),
            float(self.get_parameter('angular_kd').value),
            float(self.get_parameter('angular_integral_limit').value),
            derivative_filter_alpha,
        )

        self.gz_node = GzTransportNode()
        self.gz_cmd_pub = self.gz_node.advertise(self.boat_cmd_topic, GzTwist)
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose_v)
        self.gz_node.subscribe(Pose, self.model_pose_topic, self._on_model_pose)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', transient_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.scan_range_pub = self.create_publisher(LaserScan, self.scan_range_topic, 1)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            transient_qos,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            RosTwist,
            self.cmd_vel_topic,
            self._on_cmd_vel,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            10,
        )

        self.boat_pose = None
        self.previous_pose = None
        self.previous_pose_time = None
        self.last_cmd_time = time.monotonic()
        self.last_marker_time = 0.0
        self.target_cmd = (0.0, 0.0)
        self.filtered_setpoint = [0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0]
        self.last_output_cmd = (0.0, 0.0)
        self.last_control_time = time.monotonic()
        self.command_timed_out = False
        self.empty_map = self.make_empty_map()

        self.map_timer = self.create_timer(self.map_publish_period, self.publish_map)
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.update_velocity_control,
        )
        self.marker_timer = self.create_timer(1.0, self.publish_reference_markers)

        self.publish_map()
        self.get_logger().info(
            'Nav2 interface ready: %s -> PID -> %s, odom=/odom, map=/map, '
            'scan=%s, pose=%s, velocity_pid=%s.'
            % (
                self.cmd_vel_topic,
                self.boat_cmd_topic,
                self.scan_topic,
                self.model_pose_topic,
                self.enable_velocity_pid,
            )
        )

    def destroy_node(self):
        self.publish_gz_cmd(0.0, 0.0)
        super().destroy_node()

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name != self.boat_name:
                continue
            self.process_pose(pose)
            return

    def _on_model_pose(self, msg):
        self.process_pose(msg)

    def process_pose(self, pose):
        now = self.get_clock().now()
        stamp = now.to_msg()
        dt = None
        if self.previous_pose_time is not None:
            dt = (now - self.previous_pose_time).nanoseconds * 1e-9

        vx = 0.0
        wz = 0.0
        if self.previous_pose is not None and dt and dt > 1e-4:
            dx = pose.position.x - self.previous_pose.position.x
            dy = pose.position.y - self.previous_pose.position.y
            yaw = yaw_from_quaternion(pose.orientation)
            prev_yaw = yaw_from_quaternion(self.previous_pose.orientation)
            vx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
            yaw_delta = math.atan2(
                math.sin(yaw - prev_yaw),
                math.cos(yaw - prev_yaw),
            )
            wz = yaw_delta / dt

        self.boat_pose = pose
        self.previous_pose = pose
        self.previous_pose_time = now
        alpha = self.velocity_measurement_alpha
        self.filtered_velocity[0] += alpha * (vx - self.filtered_velocity[0])
        self.filtered_velocity[1] += alpha * (wz - self.filtered_velocity[1])
        self.publish_tf_and_odom(pose, stamp, vx, wz)

    def _on_cmd_vel(self, msg):
        self.last_cmd_time = time.monotonic()
        self.command_timed_out = False
        self.target_cmd = (
            clamp(float(msg.linear.x), 0.0, self.max_linear_output),
            clamp(
                float(msg.angular.z),
                -self.max_angular_output,
                self.max_angular_output,
            ),
        )

    def _on_scan(self, msg):
        scan_range = LaserScan()
        scan_range.header = msg.header
        scan_range.angle_min = msg.angle_min
        scan_range.angle_max = msg.angle_max
        scan_range.angle_increment = msg.angle_increment
        scan_range.time_increment = msg.time_increment
        scan_range.scan_time = msg.scan_time
        scan_range.range_min = msg.range_min
        scan_range.range_max = msg.range_max
        scan_range.ranges = [msg.range_max] * len(msg.ranges)
        scan_range.intensities = [0.0] * len(msg.ranges)
        self.scan_range_pub.publish(scan_range)

    def publish_gz_cmd(self, linear_x, angular_z):
        msg = GzTwist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.gz_cmd_pub.publish(msg)

    def update_velocity_control(self):
        now = time.monotonic()
        dt = clamp(now - self.last_control_time, 1e-3, 0.2)
        self.last_control_time = now

        if now - self.last_cmd_time > self.cmd_timeout:
            if not self.command_timed_out:
                self.command_timed_out = True
                self.reset_velocity_control()
                self.publish_gz_cmd(0.0, 0.0)
            return

        linear_target, angular_target = self.target_cmd
        self.filtered_setpoint[0] += self.linear_setpoint_alpha * (
            linear_target - self.filtered_setpoint[0]
        )
        self.filtered_setpoint[1] += self.angular_setpoint_alpha * (
            angular_target - self.filtered_setpoint[1]
        )

        if self.enable_velocity_pid and self.boat_pose is not None:
            linear_error = (
                self.filtered_setpoint[0] - self.filtered_velocity[0]
            )
            angular_error = (
                self.filtered_setpoint[1] - self.filtered_velocity[1]
            )
            linear_output = (
                self.filtered_setpoint[0]
                + self.linear_pid.update(linear_error, dt)
            )
            angular_output = (
                self.filtered_setpoint[1]
                + self.angular_pid.update(angular_error, dt)
            )
        else:
            linear_output = self.filtered_setpoint[0]
            angular_output = self.filtered_setpoint[1]

        linear_output = clamp(linear_output, 0.0, self.max_linear_output)
        angular_output = clamp(
            angular_output,
            -self.max_angular_output,
            self.max_angular_output,
        )
        if (
            abs(linear_target) <= self.command_deadband
            and abs(self.filtered_setpoint[0]) <= self.command_deadband
        ):
            linear_output = 0.0
            self.linear_pid.reset()
        if (
            abs(angular_target) <= self.command_deadband
            and abs(self.filtered_setpoint[1]) <= self.command_deadband
        ):
            angular_output = 0.0
            self.angular_pid.reset()

        self.last_output_cmd = (linear_output, angular_output)
        self.publish_gz_cmd(linear_output, angular_output)

    def reset_velocity_control(self):
        self.target_cmd = (0.0, 0.0)
        self.filtered_setpoint = [0.0, 0.0]
        self.last_output_cmd = (0.0, 0.0)
        self.linear_pid.reset()
        self.angular_pid.reset()

    def publish_tf_and_odom(self, pose, stamp, vx, wz):
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self.map_frame_id
        map_to_odom.child_frame_id = self.odom_frame_id
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame_id
        odom_to_base.child_frame_id = self.base_frame_id
        odom_to_base.transform.translation.x = pose.position.x
        odom_to_base.transform.translation.y = pose.position.y
        odom_to_base.transform.translation.z = pose.position.z
        odom_to_base.transform.rotation.x = pose.orientation.x
        odom_to_base.transform.rotation.y = pose.orientation.y
        odom_to_base.transform.rotation.z = pose.orientation.z
        odom_to_base.transform.rotation.w = pose.orientation.w

        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = stamp
        base_to_lidar.header.frame_id = self.base_frame_id
        base_to_lidar.child_frame_id = self.lidar_frame_id
        base_to_lidar.transform.translation.x = self.lidar_offset_x
        base_to_lidar.transform.translation.y = self.lidar_offset_y
        base_to_lidar.transform.translation.z = self.lidar_offset_z
        base_to_lidar.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(
            [map_to_odom, odom_to_base, base_to_lidar]
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = pose.position.x
        odom.pose.pose.position.y = pose.position.y
        odom.pose.pose.position.z = pose.position.z
        odom.pose.pose.orientation.x = pose.orientation.x
        odom.pose.pose.orientation.y = pose.orientation.y
        odom.pose.pose.orientation.z = pose.orientation.z
        odom.pose.pose.orientation.w = pose.orientation.w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def make_empty_map(self):
        width = max(1, int(round(self.map_width / self.map_resolution)))
        height = max(1, int(round(self.map_height / self.map_resolution)))
        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame_id
        grid.info.resolution = self.map_resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = -0.5 * width * self.map_resolution
        grid.info.origin.position.y = -0.5 * height * self.map_resolution
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (width * height)
        self.mark_static_landmarks(grid)
        return grid

    def mark_static_landmarks(self, grid):
        landmarks = [
            (35.0, 18.0, 2.8),
            (-42.0, 44.0, 1.8),
            (34.0, -56.0, 1.8),
            (78.0, 28.0, 1.8),
        ]
        for x, y, radius in landmarks:
            self.mark_occupied_circle(grid, x, y, radius)

    def mark_occupied_circle(self, grid, world_x, world_y, radius):
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        center_x = int((world_x - origin_x) / resolution)
        center_y = int((world_y - origin_y) / resolution)
        radius_cells = max(1, int(math.ceil(radius / resolution)))

        for cy in range(center_y - radius_cells, center_y + radius_cells + 1):
            if cy < 0 or cy >= grid.info.height:
                continue
            for cx in range(center_x - radius_cells, center_x + radius_cells + 1):
                if cx < 0 or cx >= grid.info.width:
                    continue
                dx = (cx - center_x) * resolution
                dy = (cy - center_y) * resolution
                if math.hypot(dx, dy) <= radius:
                    grid.data[cy * grid.info.width + cx] = 100

    def publish_map(self):
        self.empty_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.empty_map)

    def publish_reference_markers(self):
        now = time.monotonic()
        if now - self.last_marker_time < 0.9:
            return
        self.last_marker_time = now
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        markers.markers.append(
            self.make_marker(1, 'navigation_area', Marker.CUBE, 0.0, 0.0, -0.03,
                             240.0, 180.0, 0.02, 0.02, 0.18, 0.32, 0.18, stamp)
        )
        markers.markers.append(
            self.make_marker(2, 'static_obstacle_footprints', Marker.CYLINDER,
                             35.0, 18.0, 0.18, 6.0, 6.0, 0.35,
                             1.0, 0.82, 0.1, 0.95, stamp)
        )
        markers.markers.append(
            self.make_marker(3, 'static_obstacle_volume', Marker.CYLINDER,
                             35.0, 18.0, 5.0, 4.4, 4.4, 10.0,
                             0.95, 0.86, 0.32, 0.55, stamp)
        )
        markers.markers.append(
            self.make_text(4, 'labels', 'Lighthouse', 35.0, 18.0, 11.0, 1.8, stamp)
        )
        for idx, (x, y, label) in enumerate(
            [(-42.0, 44.0, 'Buoy A'), (34.0, -56.0, 'Buoy B'), (78.0, 28.0, 'Buoy C')]
        ):
            marker_id = 10 + idx * 3
            markers.markers.append(
                self.make_marker(marker_id, 'static_obstacle_footprints',
                                 Marker.CYLINDER, x, y, 0.16, 4.0, 4.0, 0.32,
                                 1.0, 0.12, 0.08, 0.95, stamp)
            )
            markers.markers.append(
                self.make_marker(marker_id + 1, 'static_obstacle_volume',
                                 Marker.CYLINDER, x, y, 2.0, 3.0, 3.0, 4.0,
                                 0.92, 0.18, 0.12, 0.55, stamp)
            )
            markers.markers.append(
                self.make_text(marker_id + 2, 'labels', label, x, y, 5.0, 1.4, stamp)
            )
        self.marker_pub.publish(markers)

    def make_marker(self, marker_id, namespace, marker_type, x, y, z,
                    sx, sy, sz, r, g, b, a, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(sx)
        marker.scale.y = float(sy)
        marker.scale.z = float(sz)
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)
        marker.lifetime = Duration(seconds=0.0).to_msg()
        return marker

    def make_text(self, marker_id, namespace, text, x, y, z, scale, stamp):
        marker = self.make_marker(
            marker_id, namespace, Marker.TEXT_VIEW_FACING, x, y, z,
            scale, scale, scale, 1.0, 1.0, 1.0, 0.95, stamp)
        marker.text = text
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = BoatNav2Interface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
