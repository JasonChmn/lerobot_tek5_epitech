"""
drop_box_marker_pub.py

Souscrit à /drop_box_position (PoseStamped) et publie un marqueur
Cube semi-transparent rouge sur /drop_box_marker pour RViz.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class DropBoxMarkerPublisher(Node):
    """Publie un cube RViz qui suit la drop_box."""

    def __init__(self) -> None:
        super().__init__("drop_box_marker_pub")
        self._sub = self.create_subscription(
            PoseStamped, "drop_box_position", self._cb_position, 10
        )
        self._pub = self.create_publisher(Marker, "drop_box_marker", 10)
        # Cube de 10cm × 10cm × 5cm (mêmes dimensions que la drop_box MuJoCo)
        self._marker = Marker()
        self._marker.header.frame_id = "world"
        self._marker.ns = "drop_box"
        self._marker.id = 0
        self._marker.type = Marker.CUBE
        self._marker.action = Marker.ADD
        self._marker.scale.x = 0.10
        self._marker.scale.y = 0.10
        self._marker.scale.z = 0.05
        self._marker.color.r = 1.0
        self._marker.color.g = 0.0
        self._marker.color.b = 0.0
        self._marker.color.a = 0.4  # semi-transparent
        self._marker.lifetime.sec = 0

    def _cb_position(self, msg: PoseStamped) -> None:
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._marker.pose = msg.pose
        self._pub.publish(self._marker)


def main(args=None):
    rclpy.init(args=args)
    node = DropBoxMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
