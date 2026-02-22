import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator_node')

        self.publisher = self.create_publisher(Point, 'drone_position', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update)  # 20Hz

        self.t = 0.0
        self.get_logger().info("Drone Simulator Node Started")

    def update(self):
        # Fake circular motion for visualization
        x = 5.0 * math.cos(self.t)
        y = 5.0 * math.sin(self.t)
        z = 1.0

        # Publish position topic
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z
        self.publisher.publish(msg)

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "drone"

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)

        self.t += 0.05


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
