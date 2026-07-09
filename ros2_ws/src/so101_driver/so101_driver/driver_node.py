#!/usr/bin/env python3
"""driver_node.py — pilote ROS2 du SO-ARM101.

Seul nœud qui parle au backend. Fonctionne en simulation (SO101Sim) ou sur
le bras réel (SO101Follower) via le paramètre ROS2 `use_sim` (bool, défaut True).

Topics :
  Sub : /joint_command  (sensor_msgs/JointState)
        name = noms de joints URDF, position = DEGRÉS (gripper : 0-100 %).
        Message partiel accepté (seuls les joints nommés sont mis à jour).
  Pub : /joint_states   (sensor_msgs/JointState, 30 Hz)
        position = RADIANS (convention ROS/REP-103 — requis par
        robot_state_publisher, donc par RViz). Gripper : % converti en rad
        via les limites URDF.
  Pub : /external_cam/image_raw   (sensor_msgs/Image, 15 Hz, sim uniquement)
  Pub : /external_cam/camera_info (sensor_msgs/CameraInfo, 15 Hz, sim uniquement)

Services :
  /set_joint_positions (so101_interfaces/SetJointPositions) :
        consigne 6 joints [deg×5, %], ordre JOINT_NAMES. Non bloquant.
  /pick_ball, /place_ball (std_srvs/Empty) : séquences de démonstration.

Paramètres :
  use_sim (bool) · cam_K, cam_T (float64[], publiés en sim pour la perception)

Boucle de contrôle : la dernière consigne reçue est ré-appliquée à 50 Hz via
send_action() ; get_observation() alimente /joint_states à 30 Hz. Chaque appel
au backend sim fait avancer la physique MuJoCo — le bras converge donc vers la
consigne exactement comme les servos du bras réel.
"""
import math
import threading
import time
from pathlib import Path

import rclpy
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Empty

from so101_interfaces.srv import SetJointPositions

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
ARM_JOINTS = JOINT_NAMES[:5]

# Limites du joint gripper (rad) — identiques URDF so101.urdf et MJCF sim.
# Mapping convention LeRobot : 0 % = fermé (lower), 100 % = ouvert (upper).
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)

CONTROL_HZ = 50.0
JOINT_STATE_HZ = 30.0
CAMERA_HZ = 15.0


def _gripper_pct_to_rad(pct: float) -> float:
    lo, hi = GRIPPER_RANGE_RAD
    pct = min(max(pct, 0.0), 100.0)
    return lo + pct / 100.0 * (hi - lo)


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101 : /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        self._arm = None
        self._arm_lock = threading.Lock()      # accès exclusif au backend
        self._target_lock = threading.Lock()   # consigne courante
        self._target: dict[str, float] = {}    # clés '<joint>.pos', deg / %
        self._last_image = None                # dernier rendu caméra (sim)
        self._cam_K = None
        self._cam_T = None

        mode = "simulation (SO101Sim)" if self._use_sim else "bras réel (SO101Follower)"
        self.get_logger().info(f"Mode : {mode}")
        self._connect_arm()

        # -- pubs / subs -------------------------------------------------------
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self._cmd_sub = self.create_subscription(
            JointState, "joint_command", self._cb_joint_command, 10
        )
        if self._use_sim:
            self._cam_pub = self.create_publisher(Image, "external_cam/image_raw", 10)
            self._cam_info_pub = self.create_publisher(
                CameraInfo, "external_cam/camera_info", 10
            )

        # -- services (groupe réentrant : les séquences pick/place bloquent
        #    leur thread, pas les timers — exécuteur multi-thread requis) ------
        self._srv_group = ReentrantCallbackGroup()
        self._set_joints_srv = self.create_service(
            SetJointPositions, "set_joint_positions", self._cb_set_joints,
            callback_group=self._srv_group,
        )
        self._pick_srv = self.create_service(
            Empty, "pick_ball", self._cb_pick, callback_group=self._srv_group
        )
        self._place_srv = self.create_service(
            Empty, "place_ball", self._cb_place, callback_group=self._srv_group
        )

        # -- timers ------------------------------------------------------------
        self._ctrl_timer = self.create_timer(1.0 / CONTROL_HZ, self._control_step)
        self._js_timer = self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states)
        if self._use_sim:
            self._cam_timer = self.create_timer(1.0 / CAMERA_HZ, self._publish_camera)

        self.get_logger().info("Driver node prêt.")

    # -- cycle de vie -----------------------------------------------------------

    def _connect_arm(self):
        """Initialiser le backend (sim ou réel)."""
        if self._use_sim:
            try:
                from so101_sim import SO101Sim  # pip-installé dans l'image control
            except ImportError:
                # exécution locale depuis la racine du dépôt
                import sys
                sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "sim"))
                from so101_sim import SO101Sim
            self._arm = SO101Sim(seed=42)
        else:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            port = self.declare_parameter("port", "/dev/ttyACM0").value
            robot_id = self.declare_parameter("robot_id", "tek5_arm").value
            self._arm = SO101Follower(
                SO101FollowerConfig(port=port, id=robot_id, use_degrees=True)
            )

        self._arm.connect()
        self.get_logger().info("Bras connecté.")

        if self._use_sim:
            self._cam_K = self._arm.get_camera_intrinsics()
            self._cam_T = self._arm.get_camera_extrinsics()
            self.declare_parameter("cam_K", self._cam_K.flatten().tolist())
            self.declare_parameter("cam_T", self._cam_T.flatten().tolist())
            self.get_logger().info("Paramètres caméra publiés (cam_K, cam_T).")

    def destroy_node(self):
        if self._arm is not None:
            with self._arm_lock:
                self._arm.disconnect()
        super().destroy_node()

    # -- commande ----------------------------------------------------------------

    def _cb_joint_command(self, msg: JointState):
        """Consigne reçue : degrés (gripper 0-100). Mise à jour partielle OK."""
        updates = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                updates[f"{name}.pos"] = float(pos)
            else:
                self.get_logger().warn(f"Joint inconnu ignoré : {name}", throttle_duration_sec=5.0)
        if updates:
            with self._target_lock:
                self._target.update(updates)

    def _control_step(self):
        """Ré-appliquer la consigne courante (50 Hz) — fait stepper la sim."""
        with self._target_lock:
            target = dict(self._target)
        if not target:
            return
        try:
            with self._arm_lock:
                self._arm.send_action(target)
        except Exception as e:
            self.get_logger().error(f"send_action : {e}", throttle_duration_sec=5.0)

    # -- publication --------------------------------------------------------------

    def _publish_joint_states(self):
        """Publier /joint_states en RADIANS (~30 Hz)."""
        try:
            with self._arm_lock:
                obs = self._arm.get_observation()
        except Exception as e:
            self.get_logger().error(f"get_observation : {e}", throttle_duration_sec=5.0)
            return

        if self._use_sim and "external_cam" in obs:
            self._last_image = obs["external_cam"]  # réutilisé par _publish_camera

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [math.radians(obs[f"{j}.pos"]) for j in ARM_JOINTS]
        js.position.append(_gripper_pct_to_rad(obs["gripper.pos"]))
        self._joint_pub.publish(js)

    def _publish_camera(self):
        """Publier la dernière image caméra + CameraInfo (sim, 15 Hz)."""
        img = self._last_image
        if img is None:
            return

        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "external_cam"
        img_msg.height, img_msg.width = img.shape[:2]
        img_msg.encoding = "rgb8"
        img_msg.step = img_msg.width * 3
        img_msg.data = img.tobytes()
        self._cam_pub.publish(img_msg)

        info = CameraInfo()
        info.header = img_msg.header
        info.width = img_msg.width
        info.height = img_msg.height
        info.k = self._cam_K.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P = [K | 0] (caméra fixe, pas de stéréo)
        K = self._cam_K
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0,
                  0.0, K[1, 1], K[1, 2], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._cam_info_pub.publish(info)

    # -- services -------------------------------------------------------------------

    def _cb_set_joints(self, request, response):
        """Consigne directe 6 joints [deg×5, gripper %]. Non bloquant."""
        if len(request.positions) != len(JOINT_NAMES):
            response.success = False
            response.message = f"{len(JOINT_NAMES)} positions attendues, reçu {len(request.positions)}"
            return response
        with self._target_lock:
            for name, pos in zip(JOINT_NAMES, request.positions):
                self._target[f"{name}.pos"] = float(pos)
        response.success = True
        response.message = "Consigne appliquée."
        return response

    def _set_and_settle(self, action: dict, duration: float = 1.5):
        """Poser une consigne et attendre la convergence (la boucle 50 Hz applique)."""
        with self._target_lock:
            self._target.update(action)
        time.sleep(duration)

    def _cb_pick(self, request, response):
        """Démo : approcher la zone de saisie et fermer le gripper."""
        self.get_logger().info("pick_ball : séquence de démonstration...")
        self._set_and_settle({f"{j}.pos": 0.0 for j in ARM_JOINTS} | {"gripper.pos": 100.0})
        self._set_and_settle({
            "shoulder_pan.pos": 0.0, "shoulder_lift.pos": 10.0, "elbow_flex.pos": 40.0,
            "wrist_flex.pos": -30.0, "wrist_roll.pos": 0.0, "gripper.pos": 100.0,
        })
        self._set_and_settle({"gripper.pos": 0.0}, duration=1.0)
        self.get_logger().info("pick_ball : gripper fermé.")
        return response

    def _cb_place(self, request, response):
        """Démo : aller au point de pose et ouvrir le gripper."""
        self.get_logger().info("place_ball : séquence de démonstration...")
        self._set_and_settle({
            "shoulder_pan.pos": -75.0, "shoulder_lift.pos": 10.0, "elbow_flex.pos": 40.0,
            "wrist_flex.pos": -30.0, "wrist_roll.pos": 0.0,
        })
        self._set_and_settle({"gripper.pos": 100.0}, duration=1.0)
        self.get_logger().info("place_ball : gripper ouvert.")
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = DriverNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
