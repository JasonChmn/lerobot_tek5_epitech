#!/usr/bin/env python3
"""driver_node.py — pilote ROS2 du SO-ARM101.

Fonctionne en simulation (SO101Sim) ou sur le bras réel (SO101Follower).
Paramètre ROS2 : `use_sim` (bool, défaut True).

Services :
  - /pick_ball          : ferme le gripper, va au-dessus de la boule, saisit
  - /place_ball         : va au-dessus du point de pose, ouvre le gripper
  - /set_joint_positions : positionne les joints en une fois (degrés, gripper 0-100)

Topics :
  - /joint_states       : capteurs joints à ~30 Hz
  - /external_cam/image_raw : image caméra (sim uniquement)
"""
import threading
import time

import rclpy
import rclpy.node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Empty


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class DriverNode(rclpy.node.Node):
    """Nœud driver SO-ARM101 — publie joint_states et expose services."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        self._arm = None
        self._lock = threading.Lock()
        self._ball_position = None

        # -- connecter le bras ------------------------------------------------
        mode = "simulation (SO101Sim)" if self._use_sim else "bras réel (SO101Follower)"
        self.get_logger().info(f"Mode : {mode}")
        self._connect_arm()

        # -- pubs / subs ------------------------------------------------------
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self._cam_pub = self.create_publisher(Image, "external_cam/image_raw", 10)
        self._cam_info_pub = self.create_publisher(CameraInfo, "external_cam/camera_info", 10)

        # -- services ---------------------------------------------------------
        self._pick_srv = self.create_service(Empty, "pick_ball", self._cb_pick)
        self._place_srv = self.create_service(Empty, "place_ball", self._cb_place)
        self._set_joints_srv = self.create_service(
            "so101_driver/SetJointPositions",
            self._cb_set_joints,
        )

        # -- timers -----------------------------------------------------------
        self._pub_timer = self.create_timer(1.0 / 30.0, self._publish_joint_states)
        self._cam_timer = self.create_timer(1.0 / 15.0, self._publish_camera)

        self.get_logger().info("Driver node pret.")

    # -- cycle de vie ---------------------------------------------------------

    def _connect_arm(self):
        """Initialiser le bras (sim ou réel)."""
        if self._use_sim:
            import sys
            sys.path.insert(0, "sim")
            from so101_sim import SO101Sim
            self._arm = SO101Sim(seed=42)
        else:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            self._arm = SO101Follower(SO101FollowerConfig(use_degrees=True))

        self._arm.connect()
        self.get_logger().info("Bras connecte.")

        if self._use_sim:
            self._ball_position = self._arm.get_ball_position().tolist()
            self._cam_K = self._arm.get_camera_intrinsics()
            self._cam_T = self._arm.get_camera_extrinsics()

            self.declare_parameter("cam_K", self._cam_K.flatten().tolist())
            self.declare_parameter("cam_T", self._cam_T.flatten().tolist())
            self.get_logger().info("Parametres camera publies (cam_K, cam_T).")

    def destroy_node(self):
        if self._arm:
            self._arm.disconnect()
        super().destroy_node()

    # -- callbacks de boucle --------------------------------------------------

    def _publish_joint_states(self):
        """Publier les positions actuelles (~30 Hz)."""
        try:
            with self._lock:
                obs = self._arm.get_observation()
        except Exception:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [obs[f"{j}.pos"] for j in JOINT_NAMES]
        self._joint_pub.publish(js)

    def _publish_camera(self):
        """Publier l'image et les infos cam (sim uniquement)."""
        if not self._use_sim:
            return
        try:
            with self._lock:
                img = self._arm.render_camera()
        except Exception:
            return

        # Image
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "external_cam"
        img_msg.height, img_msg.width = img.shape[:2]
        img_msg.encoding = "rgb8"
        img_msg.step = img_msg.width * 3
        img_msg.data = img.tobytes()
        self._cam_pub.publish(img_msg)

        # CameraInfo
        cam_info = CameraInfo()
        cam_info.header.stamp = img_msg.header.stamp
        cam_info.header.frame_id = "external_cam"
        cam_info.width = img_msg.width
        cam_info.height = img_msg.height
        cam_info.k = self._cam_K.flatten().tolist()
        cam_info.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        cam_info.p = self._cam_K.flatten().tolist()
        self._cam_info_pub.publish(cam_info)

    # -- callbacks services ---------------------------------------------------

    def _cb_pick(self, request, response):
        """Fermer le gripper, saisir la boule."""
        self.get_logger().info("pick_ball : initialisation...")

        home = {j: 0.0 for j in JOINT_NAMES[:5]} | {"gripper.pos": 50.0}
        self._send_settle(home)

        if self._use_sim and self._ball_position is not None:
            over_ball = {
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 10.0,
                "elbow_flex.pos": 40.0,
                "wrist_flex.pos": -30.0,
                "wrist_roll.pos": 0.0,
                "gripper.pos": 100.0,
            }
            self._send_settle(over_ball, steps=150)
            self.get_logger().info("pick_ball : au-dessus de la boule.")

        close = {"gripper.pos": 0.0}
        self._send_settle(close, steps=80)
        self.get_logger().info("pick_ball : gripper ferme. Pret.")
        return response

    def _cb_place(self, request, response):
        """Aller au point de pose et ouvrir le gripper."""
        self.get_logger().info("place_ball : initialisation...")

        pose_home = {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": 10.0,
            "elbow_flex.pos": 40.0,
            "wrist_flex.pos": -30.0,
            "wrist_roll.pos": 0.0,
            "gripper.pos": 100.0,
        }
        self._send_settle(pose_home, steps=150)

        open_g = {"gripper.pos": 100.0}
        self._send_settle(open_g, steps=80)
        self.get_logger().info("place_ball : gripper ouvert. Pose.")
        return response

    def _cb_set_joints(self, request, response):
        """Positionner les joints en une fois (degre, gripper 0-100)."""
        positions = dict(zip(JOINT_NAMES, request.positions))
        self._send_settle(positions)
        response.success = True
        response.message = f"Positions appliquees : {positions}"
        return response

    # -- utilitaires ----------------------------------------------------------

    def _send_settle(self, action, steps=100):
        """Envoyer une action et laisser le bras converger."""
        for _ in range(steps):
            with self._lock:
                self._arm.send_action(action)
            time.sleep(0.01)


def main():
    rclpy.init()
    node = None
    try:
        node = DriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
