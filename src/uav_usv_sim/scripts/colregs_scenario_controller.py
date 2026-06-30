#!/usr/bin/env python3
import math
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ColregsScenarioController(Node):
    """Drives standard encounter vessels through the existing Gazebo Twist API."""

    def __init__(self):
        super().__init__('colregs_scenario_controller')

        self.declare_parameter('pose_topic', '/world/default/pose/info')
        self.declare_parameter('ownship_name', 'landing_boat')
        self.declare_parameter('target_name', 'target_vessel')
        self.declare_parameter('ownship_cmd_topic', '/model/simple_boat/cmd_vel')
        self.declare_parameter('target_cmd_topic', '/model/target_vessel/cmd_vel')
        self.declare_parameter('auto_ownship', True)
        self.declare_parameter('ownship_speed', 1.0)
        self.declare_parameter('ownship_heading', 0.0)
        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('target_heading', math.pi)
        self.declare_parameter('heading_kp', 1.8)
        self.declare_parameter('max_turn_rate', 0.8)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('scenario_name', 'head_on')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.ownship_name = self.get_parameter('ownship_name').value
        self.target_name = self.get_parameter('target_name').value
        self.auto_ownship = bool(self.get_parameter('auto_ownship').value)
        self.ownship_speed = float(self.get_parameter('ownship_speed').value)
        self.ownship_heading = float(self.get_parameter('ownship_heading').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.target_heading = float(self.get_parameter('target_heading').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_turn_rate = float(self.get_parameter('max_turn_rate').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.scenario_name = self.get_parameter('scenario_name').value
        control_rate = max(1.0, float(self.get_parameter('control_rate').value))

        self.gz_node = GzTransportNode()
        self.ownship_pub = self.gz_node.advertise(
            self.get_parameter('ownship_cmd_topic').value,
            Twist,
        )
        self.target_pub = self.gz_node.advertise(
            self.get_parameter('target_cmd_topic').value,
            Twist,
        )
        self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose)

        self.poses = {}
        self.last_pose_time = 0.0
        self.timer = self.create_timer(1.0 / control_rate, self._on_timer)
        self.get_logger().info(
            'COLREGs scenario "%s": ownship auto=%s speed=%.2f heading=%.1f deg, '
            'target speed=%.2f heading=%.1f deg.'
            % (
                self.scenario_name,
                self.auto_ownship,
                self.ownship_speed,
                math.degrees(self.ownship_heading),
                self.target_speed,
                math.degrees(self.target_heading),
            )
        )

    def destroy_node(self):
        if self.auto_ownship:
            self._publish(self.ownship_pub, 0.0, 0.0)
        self._publish(self.target_pub, 0.0, 0.0)
        super().destroy_node()

    def _on_pose(self, msg):
        for pose in msg.pose:
            if pose.name in (self.ownship_name, self.target_name):
                self.poses[pose.name] = pose
        self.last_pose_time = time.monotonic()

    def _heading_command(self, name, speed, heading):
        pose = self.poses.get(name)
        if pose is None:
            return 0.0, 0.0
        yaw_error = wrap_pi(heading - yaw_from_quaternion(pose.orientation))
        turn_rate = clamp(
            self.heading_kp * yaw_error,
            -self.max_turn_rate,
            self.max_turn_rate,
        )
        speed_scale = clamp(1.0 - abs(yaw_error) / math.pi, 0.3, 1.0)
        return speed * speed_scale, turn_rate

    def _on_timer(self):
        if time.monotonic() - self.last_pose_time > self.pose_timeout:
            return

        target_speed, target_turn = self._heading_command(
            self.target_name,
            self.target_speed,
            self.target_heading,
        )
        self._publish(self.target_pub, target_speed, target_turn)

        if self.auto_ownship:
            own_speed, own_turn = self._heading_command(
                self.ownship_name,
                self.ownship_speed,
                self.ownship_heading,
            )
            self._publish(self.ownship_pub, own_speed, own_turn)

    @staticmethod
    def _publish(publisher, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColregsScenarioController()
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
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
