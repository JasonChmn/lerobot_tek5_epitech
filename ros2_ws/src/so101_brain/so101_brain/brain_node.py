#!/usr/bin/env python3
"""brain_node.py — intelligence du bras : IK + boucle pick & place.

Topics :
  Sub : /joint_states (sensor_msgs/JointState)
       : /ball_position_3d (geometry_msgs/PoseStamped)
  Pub : /brain_state (std_msgs/String) — etats de la boucle

Services :
  /brain/start_autonomous  : lance la boucle pick & place
  /brain/stop_autonomous   : arrete la boucle
  /brain/go_to_target      : IK vers une position 3D cible

Parametres :
  - use_sim (bool) : identique au driver
  - pick_place_duration (float) : temps entre chaque cycle

Fonctionnement :
  1. Lire /joint_states -> configuration courante (initial_position pour IK)
  2. Lire /ball_position_3d -> cible
  3. IK numerique via ikpy -> consignes
  4. Envoyer au driver via service /set_joint_positions
  5. Boucle pick & place : home -> over ball -> down -> close -> lift -> over home -> down -> open -> home
"""
import sys
import threading

sys.path.insert(0, "sim")

import numpy as np
import rclpy
import rclpy.node
from ikpy import chain as ik_chain
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray


# -- IK -----------------------------------------------------------------------
IK_CHAIN = ik_chain.Chain.from_urdf_file(
    "sim/so101_sim/assets/so101/so101_new_calib.urdf",
    active_links_mask=[False, True, True, True, True, True, False],
)

# Noms des 5 joints actifs (sans le gripper)
ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

JOINT_NAMES = ARM_JOINT_NAMES + ["gripper"]

# Home config (tous les joints a 0, gripper a 50)
HOME_CONFIG = [0.0] + [0.0] * 5 + [50.0]

# Target offset : la pince doit etre au-dessus de la boule (z + 0.05)
PICK_OFFSET = 0.05
PLACE_Z = 0.03


class BrainNode(rclpy.node.Node):
    """Cerveau du bras : IK + pick & place autonome."""

    def __init__(self):
        super().__init__("so101_brain")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value

        # -- etat interne -----------------------------------------------------
        self._lock = threading.Lock()
        self._current_joints = dict.fromkeys(ARM_JOINT_NAMES, 0.0)  # dernier joint_states lu
        self._ball_pose_3d = None  # derniere position de la boule
        self._autonomous = False
        self._autonomous_thread = None
        self._state = "idle"  # idle, pick, place, error

        # -- IK pre-calcul ----------------------------------------------------
        self.get_logger().info("Chargement ikpy...")
        self.get_logger().info("ikpy charge.")

        # -- pubs / subs ------------------------------------------------------
        self._state_pub = self.create_publisher(String, "brain_state", 10)
        self._joint_sub = self.create_subscription(
            JointState, "joint_states", self._cb_joint_states, 10
        )
        self._ball_sub = self.create_subscription(
            PoseStamped, "ball_position_3d", self._cb_ball, 10
        )

        # -- services ---------------------------------------------------------
        self._start_srv = self.create_service(
            SetBool, "brain/start_autonomous", self._cb_start_autonomous
        )
        self._stop_srv = self.create_service(
            SetBool, "brain/stop_autonomous", self._cb_stop_autonomous
        )
        self._go_to_srv = self.create_service(
            "so101_brain/GoToTarget", self._cb_go_to_target
        )

        # -- client service vers driver ---------------------------------------
        self._set_joints_client = self.create_client(
            "so101_driver/SetJointPositions", "set_joint_positions"
        )
        while not self._set_joints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("En attente du service set_joint_positions du driver...")
        self.get_logger().info("Service set_joint_positions du driver disponible.")

        # -- types de service charges dynamiquement ---------------------------
        from rosidl_runtime_py.utilities import get_service
        self._SetJointPositions = get_service("so101_driver/SetJointPositions")

        # -- timer pour publication de l'etat ---------------------------------
        self._pub_timer = self.create_timer(1.0 / 10.0, self._publish_state)

        self._set_state("ready")
        self.get_logger().info("Brain node pret.")

    # -- callbacks ------------------------------------------------------------

    def _cb_joint_states(self, msg):
        """Mettre a jour les angles courants."""
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in ARM_JOINT_NAMES:
                    self._current_joints[name] = float(pos)

    def _cb_ball(self, msg):
        """Mettre a jour la position de la boule."""
        with self._lock:
            self._ball_pose_3d = msg

    def _cb_start_autonomous(self, request, response):
        """Demarrer le pick & place automatique."""
        if self._autonomous:
            response.success = False
            response.message = "Deja en mode autonome."
            return response
        self._autonomous = True
        self._autonomous_thread = threading.Thread(
            target=self._autonomous_loop, daemon=True
        )
        self._autonomous_thread.start()
        response.success = True
        response.message = "Pick & place automatique demarre."
        return response

    def _cb_stop_autonomous(self, request, response):
        """Arreter le pick & place automatique."""
        self._autonomous = False
        self._set_state("idle")
        response.success = True
        response.message = "Mode autonome arrete."
        return response

    def _cb_go_to_target(self, request, response):
        """IK vers une position 3D cible."""
        try:
            target = [request.target_pose.position.x,
                      request.target_pose.position.y,
                      request.target_pose.position.z]
            q_sol = self._solve_ik(target)
            angles_deg = np.degrees(q_sol[1:6])

            # Envoyer au driver
            action = {}
            for i, j in enumerate(ARM_JOINT_NAMES):
                action[f"{j}.pos"] = float(angles_deg[i])
            action["gripper.pos"] = 50.0

            self.get_logger().info(f"IK -> {dict(zip(ARM_JOINT_NAMES, angles_deg))}")
            self._send_action(action)
            response.success = True
            response.message = f"IK vers {target}"
        except Exception as e:
            response.success = False
            response.message = f"Erreur IK : {e}"
            self.get_logger().error(response.message)
        return response

    # -- IK -------------------------------------------------------------------

    def _get_initial_position(self):
        """Retourne le vecteur 7D pour ikpy (seed avec config courante)."""
        with self._lock:
            current = dict(self._current_joints)
        # Convertir degres -> radians pour ikpy
        q_rad = [0.0]  # link fixe 0
        for j in ARM_JOINT_NAMES:
            q_rad.append(np.radians(current.get(j, 0.0)))
        q_rad.append(0.0)  # gripper_frame fixe
        return q_rad

    def _solve_ik(self, target_position):
        """Resoudre l'IK numerique avec ikpy."""
        initial = self._get_initial_position()

        try:
            q_sol = IK_CHAIN.inverse_kinematics(
                target_position=target_position,
                initial_position=initial,
                orientation_mode="Z",
                target_orientation=[0, 0, -1],
            )
        except Exception:
            q_sol = IK_CHAIN.inverse_kinematics(
                target_position=target_position,
                initial_position=initial,
            )

        # Verifier que la solution est dans l'espace de travail
        fk_result = IK_CHAIN.forward_kinematics(q_sol)
        fk_pos = fk_result[:3, 3]
        error = np.linalg.norm(fk_pos - np.array(target_position))

        if error > 0.02:  # > 2cm : cible probablement inaccessible
            self.get_logger().warn(
                f"IK erreur : {error:.3f}m (seuil 0.02m). Cible {target_position}"
            )
            # Retourner quand meme la solution (ikpy retourne le meilleur compromis)

        return q_sol

    def _angles_to_action(self, q_sol):
        """Converter la solution ikpy (rad, 7D) en action driver (deg, dict)."""
        angles_deg = np.degrees(q_sol[1:6])
        action = {}
        for i, j in enumerate(ARM_JOINT_NAMES):
            action[f"{j}.pos"] = float(angles_deg[i])
        action["gripper.pos"] = 50.0
        return action

    # -- boucle autonome pick & place -----------------------------------------

    def _autonomous_loop(self):
        """Boucle pick & place infinie."""
        self.get_logger().info("Boucle autonome demarre.")
        cycle = 0
        while self._autonomous:
            try:
                cycle += 1
                self.get_logger().info(f"Cycle pick & place #{cycle}")

                # -- etape 1 : home ------------------------------------------------
                self._set_state("home")
                self._go_to_home()

                # -- etape 2 : au-dessus de la boule -------------------------------
                self._set_state("approach")
                if not self._go_over_ball():
                    self.get_logger().warn("Boule non detectee. Attente...")
                    self._wait_ball()
                    continue

                # -- etape 3 : descendre vers la boule -----------------------------
                self._set_state("down_to_ball")
                self._go_to_ball_surface()

                # -- etape 4 : fermer gripper --------------------------------------
                self._set_state("grasp")
                self._send_action({"gripper.pos": 0.0})

                # -- etape 5 : remonter --------------------------------------------
                self._set_state("lift")
                lift_pos = {
                    "shoulder_pan.pos": 0.0,
                    "shoulder_lift.pos": 15.0,
                    "elbow_flex.pos": 50.0,
                    "wrist_flex.pos": -20.0,
                    "wrist_roll.pos": 0.0,
                    "gripper.pos": 20.0,
                }
                self._send_action(lift_pos)

                # -- etape 6 : over home -------------------------------------------
                self._set_state("transport")
                self._go_to_home()

                # -- etape 7 : over place point ------------------------------------
                self._set_state("place_approach")
                place_pos = {
                    "shoulder_pan.pos": 0.0,
                    "shoulder_lift.pos": 15.0,
                    "elbow_flex.pos": 40.0,
                    "wrist_flex.pos": -30.0,
                    "wrist_roll.pos": 0.0,
                    "gripper.pos": 50.0,
                }
                self._send_action(place_pos)

                # -- etape 8 : ouvrir gripper (placer) -----------------------------
                self._set_state("place")
                self._send_action({"gripper.pos": 100.0})

                # -- etape 9 : home ------------------------------------------------
                self._set_state("home")
                self._go_to_home()

                self.get_logger().info(f"Cycle #{cycle} termine.")

            except Exception as e:
                self.get_logger().error(f"Erreur cycle : {e}")
                self._set_state("error")

        self.get_logger().info("Boucle autonome arretee.")

    # -- methodes d'assistance --------------------------------------------------

    def _go_to_home(self):
        """Aller a la configuration home."""
        self._send_action(HOME_CONFIG)

    def _go_over_ball(self):
        """IK vers au-dessus de la boule."""
        with self._lock:
            ball = self._ball_pose_3d
        if ball is None:
            return False

        target = [
            ball.pose.position.x,
            ball.pose.position.y,
            ball.pose.position.z + PICK_OFFSET,
        ]
        q_sol = self._solve_ik(target)
        action = self._angles_to_action(q_sol)
        self._send_action(action)
        return True

    def _go_to_ball_surface(self):
        """IK vers la surface de la boule (z = PLACE_Z)."""
        with self._lock:
            ball = self._ball_pose_3d
        if ball is None:
            return
        target = [
            ball.pose.position.x,
            ball.pose.position.y,
            PLACE_Z,
        ]
        q_sol = self._solve_ik(target)
        action = self._angles_to_action(q_sol)
        self._send_action(action)

    def _wait_ball(self, timeout=30.0):
        """Attendre que la boule soit detectee."""
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout
        while self._autonomous and (self.get_clock().now().nanoseconds / 1e9 < deadline):
            with self._lock:
                if self._ball_pose_3d is not None:
                    return
            rclpy.spin_once(self, timeout_sec=0.5)

    def _send_action(self, action, steps=100):
        """Envoyer une action au bras (via driver)."""
        positions = [0.0] * 6
        for i, name in enumerate(JOINT_NAMES):
            key = f"{name}.pos"
            if key in action:
                positions[i] = float(action[key])
            else:
                with self._lock:
                    if name in self._current_joints:
                        positions[i] = self._current_joints[name]
                    elif name == "gripper":
                        positions[i] = 50.0
                    else:
                        positions[i] = 0.0
        try:
            req = self._SetJointPositions.Request()
            req.positions = positions
            future = self._set_joints_client.call_async(req)
            import time as _time
            while not future.done():
                _time.sleep(0.01)
            resp = future.result()
            if resp.success:
                self.get_logger().debug(f"Action envoyee: {positions}")
            else:
                self.get_logger().warn(f"Action refusee: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Erreur envoi action: {e}")

    def _set_state(self, state):
        """Changer l'etat interne."""
        with self._lock:
            self._state = state

    def _publish_state(self):
        """Publier l'etat courant."""
        with self._lock:
            state = self._state
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = BrainNode()
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
