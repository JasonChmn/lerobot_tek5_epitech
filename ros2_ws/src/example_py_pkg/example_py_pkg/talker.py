"""Nœud d'exemple : publie un compteur sur /chatter à 2 Hz.

Vérifie que votre environnement fonctionne :
    colcon build && source install/setup.bash
    ros2 run example_py_pkg talker
    # dans l'autre container :
    ros2 topic echo /chatter
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(String, "chatter", 10)
        self.count = 0
        self.create_timer(0.5, self.tick)

    def tick(self):
        msg = String(data=f"tek5 {self.count}")
        self.pub.publish(msg)
        self.count += 1


def main():
    rclpy.init()
    rclpy.spin(Talker())


if __name__ == "__main__":
    main()
