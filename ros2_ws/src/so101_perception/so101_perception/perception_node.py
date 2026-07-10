#!/usr/bin/env python3
"""perception_node.py — détection de la boule dorée et recalage 3D.

Topics :
  Sub : /external_cam/image_raw (sensor_msgs/Image)
  Pub : /ball_position_3d (geometry_msgs/PoseStamped)

Calibration (lue depuis le topic /camera_info ou paramètre) :
  - Matrice intrinseque K (3x3)
  - Transformee camera -> monde (4x4)

Fonctionnement :
  1. Segmenter la boule en HSV (jaune/or).
  2. Trouver le contour le plus rond -> centre (u, v).
  3. Projeter (u, v) dans l'espace 3D via K^{-1} * [u, v, 1].
  4. Appliquer la TF camera->monde pour obtenir la position monde.
"""
import threading

import cv2
import numpy as np
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Image


# -- plages HSV pour la boule dorée -------------------------------------------
# Ajuster selon l'eclairage de la scene MuJoCo.
HSV_MIN = np.array([15, 80, 150], dtype=np.uint8)
HSV_MAX = np.array([35, 255, 255], dtype=np.uint8)


class PerceptionNode(rclpy.node.Node):
    """Detecte la boule dorée et publie sa position 3D."""

    def __init__(self):
        super().__init__("so101_perception")
        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # -- calibration ------------------------------------------------------
        self._K = None  # intrinseques (3x3)
        self._T_cam_world = None  # extrinseques (4x4)
        self._current_ball_pose = None  # derniere detection (PoseStamped ou None)
        self._ball_radius = self.declare_parameter("ball_radius", 0.015).value

        # -- lire calibration depuis params (publies par driver en sim) -------
        self.declare_parameter("cam_K", [])
        self.declare_parameter("cam_T", [])
        cam_k_list = self.get_parameter("cam_K").value
        cam_t_list = self.get_parameter("cam_T").value
        if cam_k_list and len(cam_k_list) == 9:
            self._K = np.array(cam_k_list, dtype=np.float64).reshape(3, 3)
            self.get_logger().info("cam_K lu depuis parametres ROS 2.")
        if cam_t_list and len(cam_t_list) == 16:
            self._T_cam_world = np.array(cam_t_list, dtype=np.float64).reshape(4, 4)
            self.get_logger().info("cam_T lu depuis parametres ROS 2.")

        # -- subscriptions ----------------------------------------------------
        self._cam_info_sub = self.create_subscription(
            CameraInfo, "external_cam/camera_info", self._cb_camera_info, 10
        )
        self._image_sub = self.create_subscription(
            Image, "external_cam/image_raw", self._cb_image, 10
        )

        # -- publisher --------------------------------------------------------
        self._ball_pub = self.create_publisher(
            PoseStamped, "ball_position_3d", 10
        )

        # -- param fetch client for driver cam_K / cam_T ----------------------
        self._param_cli = self.create_client(
            GetParameters, "/so101_driver/get_parameters"
        )
        self._fetch_timer = self.create_timer(1.0, self._fetch_cam_params)

        self._pub_timer = self.create_timer(1.0 / 10.0, self._publish_result)  # 10 Hz — spec SUJET (5-10 Hz)
        self._ball_count = 0
        self.get_logger().info("Perception node pret.")

    # -- callbacks -----------------------------------------------------------

    def _cb_camera_info(self, msg):
        """Recuperer la matrice K."""
        K = np.array(msg.k).reshape(3, 3)
        with self._lock:
            if self._K is None:
                self.get_logger().info("Camera intrinsics recue (CameraInfo).")
            self._K = K

    def _cb_image(self, msg):
        """Convertir l'image et lancer la detection."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().warn(f"Erreur cv_bridge : {e}")
            return

        # -- detection -------------------------------------------------------
        ball_pos_3d = self._detect_ball(cv_image)

        # -- stockage thread-safe ---------------------------------------------
        with self._lock:
            self._current_ball_pose = ball_pos_3d
            if self._ball_count % 30 == 0:
                self.get_logger().info(
                    f"image recue (K={'OK' if self._K is not None else 'None'}, "
                    f"T={'OK' if self._T_cam_world is not None else 'None'}), "
                    f"detection={'OK' if ball_pos_3d is not None else 'None'}"
                )
        self._ball_count += 1

    def _detect_ball(self, cv_image):
        """Segmenter la boule en HSV, trouver centre, projeter en 3D."""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)

        # morphologie pour nettoyer le bruit
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn("detect_ball: no contours found")
            return None

        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if self._ball_count % 50 == 0:
            self.get_logger().info(f"detection: contours={len(contours)}, area={area:.1f}")
        if area < 50:
            return None

        # centre du contour
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return None
        u = M["m10"] / M["m00"]
        v = M["m01"] / M["m00"]

        # -- projection 3D ---------------------------------------------------
        return self._project_to_3d(u, v)

    def _project_to_3d(self, u, v):
        """Convertir pixel (u, v) en position monde via K et TF camera->monde."""
        # 1. Pixel -> rayon camera (coordonnees normales)
        if self._K is None:
            return None
        K_inv = np.linalg.inv(self._K)
        ray = K_inv @ np.array([u, v, 1.0])  # rayon pinhole (+z avant, +v bas)

        # Convention camera MuJoCo : x a droite, y VERS LE HAUT, regarde le long
        # de -z (cf. DESIGN.md). Le rayon pinhole doit donc etre re-exprime :
        ray = np.array([ray[0], -ray[1], -ray[2]])

        # 2. Estimer la profondeur : la boule est sur le plan z=0 (table)
        #    On intersecte le rayon avec le plan z=0 dans le systeme monde
        #    (hypothese : la boule repose sur la table)
        if self._T_cam_world is None:
            # fallback : utiliser la position z de la boule en sim
            # pour sim, la boule est sur la table -> on peut estimer z
            return None

        T = self._T_cam_world
        # Position camera dans le monde
        cam_pos = T[:3, 3]
        # Orientation camera -> monde
        cam_R = T[:3, :3]
        # Rayon dans le systeme monde
        ray_world = cam_R @ ray

        # Intersection avec le plan du CENTRE de la boule (z = rayon, boule au sol)
        # cam_pos + t * ray_world = (x, y, ball_radius)
        if abs(ray_world[2]) < 1e-6:
            return None
        t = (self._ball_radius - cam_pos[2]) / ray_world[2]
        if t <= 0:
            return None  # intersection derriere la camera

        pos_world = cam_pos + t * ray_world
        pos_world[2] = self._ball_radius

        # -- construire PoseStamped ------------------------------------------
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(pos_world[0])
        pose.pose.position.y = float(pos_world[1])
        pose.pose.position.z = float(pos_world[2])
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_result(self):
        """Publier la derniere detection."""
        with self._lock:
            ball_pose = getattr(self, "_current_ball_pose", None)
        if ball_pose is not None:
            ball_pose.header.stamp = self.get_clock().now().to_msg()
            self._ball_pub.publish(ball_pose)

    def _fetch_cam_params(self):
        """Recuperer cam_K et cam_T depuis le driver (une fois), puis arrete le timer."""
        with self._lock:
            if self._K is not None and self._T_cam_world is not None:
                self._fetch_timer.cancel()
                return

        if not self._param_cli.service_is_ready():
            self.get_logger().warn("attente du driver pour cam_K/cam_T...")
            return

        req = GetParameters.Request(names=["cam_K", "cam_T"])

        def _on_response(future):
            try:
                resp = future.result()
                if not resp.values or len(resp.values) < 2:
                    self.get_logger().warn(
                        "Reponse parameters vide, reessai..."
                    )
                    return

                k_vals = resp.values[0].double_array_value
                t_vals = resp.values[1].double_array_value

                if not k_vals or len(k_vals) != 9:
                    self.get_logger().warn(
                        f"cam_K invalide ({len(k_vals) if k_vals else 0} floats), reessai..."
                    )
                    return
                if not t_vals or len(t_vals) != 16:
                    self.get_logger().warn(
                        f"cam_T invalide ({len(t_vals) if t_vals else 0} floats), reessai..."
                    )
                    return

                K = np.array(k_vals, dtype=np.float64).reshape(3, 3)
                T = np.array(t_vals, dtype=np.float64).reshape(4, 4)

                with self._lock:
                    self._K = K
                    self._T_cam_world = T

                self.get_logger().info(
                    "cam_K / cam_T recus du driver."
                )
                self._fetch_timer.cancel()

            except Exception as e:
                self.get_logger().warn(f"Erreur fetch params: {e}, reessai...")

        future = self._param_cli.call_async(req)
        future.add_done_callback(_on_response)


def main():
    rclpy.init()
    node = None
    try:
        node = PerceptionNode()
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
