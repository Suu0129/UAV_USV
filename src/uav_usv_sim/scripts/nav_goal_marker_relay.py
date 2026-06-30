#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class NavGoalMarkerRelay(Node):
    """Relays RViz PoseStamped goals to Nav2 and publishes a persistent marker."""

    def __init__(self):
        super().__init__('nav_goal_marker_relay')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/boat/nav_goal_marker')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('default_frame_id', 'map')

        self.goal_topic = self.get_parameter('goal_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.default_frame_id = self.get_parameter('default_frame_id').value
        action_name = self.get_parameter('action_name').value

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            marker_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self._on_goal,
            10,
        )
        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.pending_goal = None
        self.retry_timer = self.create_timer(0.5, self._send_pending_goal)

        self.get_logger().info(
            'RViz goals on %s are relayed to Nav2 action %s; marker=%s.'
            % (self.goal_topic, action_name, self.marker_topic)
        )

    def _on_goal(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = self.default_frame_id
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.pending_goal = msg
        self._publish_marker(msg)
        self.get_logger().info(
            'Navigation goal selected: x=%.2f y=%.2f frame=%s'
            % (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.header.frame_id,
            )
        )
        self._send_pending_goal()

    def _send_pending_goal(self):
        if self.pending_goal is None or not self.action_client.server_is_ready():
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.pending_goal
        self.pending_goal = None
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error('Failed to send Nav2 goal: %s' % exc)
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the selected goal')
            return
        self.get_logger().info('Nav2 accepted the selected goal')

    def _publish_marker(self, goal):
        markers = MarkerArray()

        base = Marker()
        base.header = goal.header
        base.ns = 'navigation_goal'
        base.id = 0
        base.type = Marker.CYLINDER
        base.action = Marker.ADD
        base.pose = goal.pose
        base.pose.position.z = 0.18
        base.scale.x = 1.6
        base.scale.y = 1.6
        base.scale.z = 0.35
        base.color.r = 1.0
        base.color.g = 0.22
        base.color.b = 0.08
        base.color.a = 0.95
        base.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(base)

        arrow = Marker()
        arrow.header = goal.header
        arrow.ns = 'navigation_goal'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = goal.pose
        arrow.pose.position.z = 0.55
        arrow.scale.x = 3.2
        arrow.scale.y = 0.35
        arrow.scale.z = 0.45
        arrow.color.r = 1.0
        arrow.color.g = 0.75
        arrow.color.b = 0.08
        arrow.color.a = 1.0
        arrow.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(arrow)

        label = Marker()
        label.header = goal.header
        label.ns = 'navigation_goal'
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = goal.pose.position.x
        label.pose.position.y = goal.pose.position.y
        label.pose.position.z = 2.2
        label.pose.orientation.w = 1.0
        label.scale.z = 1.1
        label.color.r = 1.0
        label.color.g = 0.9
        label.color.b = 0.55
        label.color.a = 1.0
        label.text = 'Navigation Goal'
        label.lifetime = Duration(seconds=0.0).to_msg()
        markers.markers.append(label)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalMarkerRelay()
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
