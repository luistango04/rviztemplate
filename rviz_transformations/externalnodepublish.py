#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros

class ExternalTFPublisher(Node):
    def __init__(self):
        super().__init__('externalnodepublish')

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Dictionary with topic names and corresponding world and child frames
        topics_info = {
           '/vrpn_mocap/bunker_mini/pose': ('world', 'bunker_octo'),
             '/vrpn_mocap/Telloextra/pose': ('world', 'Telloextra'),
            '/vrpn_mocap/Tello2b/pose': ('world', 'xwing'),
            '/vrpn_mocap/Tello3b/pose': ('world', 'ywing'),
            '/vrpn_mocap/Tiramisu/pose': ('world', 'tiramisu')
        }

        # Create subscriptions based on the dictionary
        for topic, (world_frame, child_frame) in topics_info.items():
            self.create_dynamic_subscription(topic, world_frame, child_frame)

    def create_dynamic_subscription(self, topic: str, world_frame: str, child_frame: str):
        """
        Creates a subscription dynamically for each topic with corresponding world_frame and child_frame.
        """
        if topic.startswith('/vrpn_mocap'):
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, world=world_frame, child=child_frame: self.calibration_body_callback(msg, world, child),
                qos_profile
            )
        else:
            self.create_subscription(
                TransformStamped,
                topic,
                lambda msg, world=world_frame, child=child_frame: self.external_transform_callback(msg, world, child),
                10
            )

    def calibration_body_callback(self, msg: PoseStamped, header_frame_id: str, child_frame_id: str):
        # Convert PoseStamped to TransformStamped
        transform = TransformStamped()
        
        # Set the current time and frame IDs based on the input parameters
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = header_frame_id  # Parent frame ID
        transform.child_frame_id = child_frame_id    # Child frame ID
        
        # Fill in the translation and rotation from the PoseStamped message
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        
        # Send the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()