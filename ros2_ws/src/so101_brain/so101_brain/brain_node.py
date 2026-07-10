#!/usr/bin/env python3
"""brain_node.py — intelligence du bras : IK + boucle pick & place.

Topics :
  Sub : /joint_states     (sensor_msgs/JointState, RADIANS — publié par le driver)
        /ball_position_3d (geometry_msgs/PoseStamped, repère world/robot)
  Pub : /joint_command    (sensor_msgs/JointState, DEGRÉS, gripper 0-100 %)
        /brain_state      (std_msgs/String, 10 Hz)

Services :
  /brain/go_to            (so101_interfaces/GoToTarget) : IK vers une position 3D
  /brain/start            (std_srvs/Trigger)            : lance la boucle pick & place
  /brain/stop             (std_srvs/Trigger)            : arrête la boucle

Conformément au DESIGN : toutes les commandes moteur passent par /joint_command.
Le service /driver/set_joints du driver reste disponible pour les tests manuels.

IK : ikpy sur l'URDF officiel du SO-101 (celui du package so101_sim). Seed =
configuration courante (issue de /joint_states, déjà en radians). Une cible dont
l'erreur résiduelle FK dépasse `ik_tolerance` (2 cm) est rejetée -> retour idle.
"""
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from ikpy import chain as ik_chain
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_interfaces.srv import GoToTarget

ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
JOINT_NAMES = ARM_JOINT_NAMES + ["gripper"]

# Offsets de la stratégie pick & place (m)
PICK_APPROACH_Z = 0.10   # approche au-dessus de la boule
GRASP_Z = 0.02           # hauteur pince à la saisie (boule r=0.015 posée au sol)
BALL_LOST_TIMEOUT = 30.0

# Point de pose : fallback si /drop_box_position (publié par le driver) est muet.
# La drop_box de la scène par défaut est en (0.05, -0.25).
PLACE_XY_FALLBACK = (0.05, -0.25)
# z=0.10 avec contrainte -Z stricte = hors workspace ikpy (erreur 31 mm) ;
# 0.08 passe avec la contrainte, et _solve_ik a un fallback orientation libre.
PLACE_APPROACH_Z = 0.08

IK_TOLERANCE_M = 0.02


def _find_urdf() -> str:
    """URDF pour ikpy : priorité au package so101_sim installé (pip),
    fallback sur l'arborescence du dépôt (exécution locale)."""
    try:
        from so101_sim import ASSETS_DIR
        p = Path(ASSETS_DIR) / "so101_new_calib.urdf"
        if p.exists():
            return str(p)
    except ImportError:
        pass
    repo = Path(__file__).resolve().parents[4]  # .../ros2_ws/src/so101_brain/so101_brain
    p = repo / "sim" / "so101_sim" / "assets" / "so101" / "so101_new_calib.urdf"
    if p.exists():
        return str(p)
    raise FileNotFoundError("URDF so101_new_calib.urdf introuvable (so101_sim non installé ?)")


class BrainNode(rclpy.node.Node):
    """Cerveau du bras : IK + pick & place autonome via /joint_command."""

    def __init__(self):
        super().__init__("so101_brain")

        # -- état interne -------------------------------------------------------
        self._lock = threading.Lock()
        self._current_rad = dict.fromkeys(ARM_JOINT_NAMES, 0.0)  # /joint_states (rad)
        self._ball_pose: PoseStamped | None = None
        self._drop_box_xy: tuple[float, float] | None = None  # /drop_box_position
        self._autonomous = False
        self._autonomous_thread: threading.Thread | None = None
        self._state = "init"

        # -- IK -----------------------------------------------------------------
        urdf = _find_urdf()
        self.get_logger().info(f"Chargement chaîne ikpy : {urdf}")
        # 7 maillons : [base fixe, 5 joints actifs, gripper_frame fixe]
        self._chain = ik_chain.Chain.from_urdf_file(
            urdf, active_links_mask=[False, True, True, True, True, True, False]
        )
        self._ik_tol = self.declare_parameter("ik_tolerance", IK_TOLERANCE_M).value

        # -- pubs / subs ----------------------------------------------------------
        self._cmd_pub = self.create_publisher(JointState, "joint_command", 10)
        self._state_pub = self.create_publisher(String, "brain_state", 10)
        self.create_subscription(JointState, "joint_states", self._cb_joint_states, 10)
        self.create_subscription(PoseStamped, "ball_position_3d", self._cb_ball, 10)
        self.create_subscription(PoseStamped, "drop_box_position", self._cb_drop_box, 10)

        # -- services ---------------------------------------------------------------
        self.create_service(GoToTarget, "brain/go_to", self._cb_go_to)
        self.create_service(Trigger, "brain/start", self._cb_start)
        self.create_service(Trigger, "brain/stop", self._cb_stop)

        self.create_timer(1.0 / 10.0, self._publish_state)
        self._set_state("idle")
        self.get_logger().info("Brain node prêt.")

    # -- callbacks ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState):
        """/joint_states est en radians (convention driver/RViz)."""
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in ARM_JOINT_NAMES:
                    self._current_rad[name] = float(pos)

    def _cb_ball(self, msg: PoseStamped):
        with self._lock:
            self._ball_pose = msg

    def _cb_drop_box(self, msg: PoseStamped):
        with self._lock:
            self._drop_box_xy = (msg.pose.position.x, msg.pose.position.y)

    def _cb_go_to(self, request, response):
        """IK vers une position 3D cible, puis publication sur /joint_command."""
        target = [
            request.target_pose.pose.position.x,
            request.target_pose.pose.position.y,
            request.target_pose.pose.position.z,
        ]
        q_sol, err = self._solve_ik(target)
        if q_sol is None:
            response.success = False
            response.message = f"Cible {target} hors espace de travail (erreur IK {err:.3f} m)"
            self.get_logger().warn(response.message)
            return response

        self._send_action(self._q_to_action(q_sol))
        response.success = True
        response.message = f"IK vers {target} (erreur {err * 1000:.1f} mm)"
        self.get_logger().info(response.message)
        return response

    def _cb_start(self, request, response):
        if self._autonomous:
            response.success = False
            response.message = "Déjà en mode autonome."
            return response
        self._autonomous = True
        self._autonomous_thread = threading.Thread(target=self._autonomous_loop, daemon=True)
        self._autonomous_thread.start()
        response.success = True
        response.message = "Pick & place autonome démarré."
        return response

    def _cb_stop(self, request, response):
        self._autonomous = False
        self._set_state("idle")
        response.success = True
        response.message = "Mode autonome arrêté."
        return response

    # -- IK ------------------------------------------------------------------------

    def _seed(self):
        """Vecteur 7D initial pour ikpy = configuration courante (rad)."""
        with self._lock:
            cur = dict(self._current_rad)
        return [0.0] + [cur[j] for j in ARM_JOINT_NAMES] + [0.0]

    def _solve_ik(self, target_position):
        """Retourne (q_sol, erreur) — q_sol=None si la cible est inatteignable."""
        seed = self._seed()
        try:
            q_sol = self._chain.inverse_kinematics(
                target_position=target_position,
                initial_position=seed,
                orientation_mode="Z",           # pince pointant vers le bas
                target_orientation=[0, 0, -1],
            )
        except Exception:
            q_sol = None

        def _fk_err(q):
            fk_pos = self._chain.forward_kinematics(q)[:3, 3]
            return float(np.linalg.norm(fk_pos - np.asarray(target_position)))

        err = _fk_err(q_sol) if q_sol is not None else float("inf")
        if err > self._ik_tol:
            # Fallback : orientation libre. La contrainte -Z stricte sort
            # certaines cibles (ex. transport z>=0.10) du workspace ikpy.
            q_rel = self._chain.inverse_kinematics(
                target_position=target_position, initial_position=seed
            )
            err_rel = _fk_err(q_rel)
            if err_rel < err:
                q_sol, err = q_rel, err_rel
        if err > self._ik_tol:
            return None, err
        return q_sol, err

    def _q_to_action(self, q_sol, gripper_pct=None):
        """Solution ikpy (rad, 7D) -> action driver (deg + gripper %)."""
        angles_deg = np.degrees(q_sol[1:6])
        action = {j: float(a) for j, a in zip(ARM_JOINT_NAMES, angles_deg)}
        if gripper_pct is not None:
            action["gripper"] = float(gripper_pct)
        return action

    # -- commande ---------------------------------------------------------------------

    def _send_action(self, action: dict, settle: float = 0.0):
        """Publier une consigne sur /joint_command (deg, gripper 0-100 %).

        Le driver ré-applique la consigne à 50 Hz : publier une fois suffit.
        `settle` : attente optionnelle de convergence (boucle autonome).
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(action.keys())
        msg.position = [float(v) for v in action.values()]
        self._cmd_pub.publish(msg)
        if settle > 0:
            time.sleep(settle)

    def _go_home(self, settle=1.5):
        self._send_action({j: 0.0 for j in ARM_JOINT_NAMES} | {"gripper": 100.0}, settle)

    # -- boucle autonome -----------------------------------------------------------------

    def _autonomous_loop(self):
        self.get_logger().info("Boucle autonome démarrée.")
        cycle = 0
        while self._autonomous:
            cycle += 1
            try:
                self.get_logger().info(f"Cycle pick & place #{cycle}")

                self._set_state("home")
                self._go_home()

                # -- localiser la boule ------------------------------------------
                # Détection FRAÎCHE exigée : la dernière pose peut dater du cycle
                # précédent (boule déjà déplacée).
                with self._lock:
                    self._ball_pose = None
                ball = self._wait_ball()
                if ball is None:
                    self.get_logger().warn("Boule non détectée — nouvel essai.")
                    continue
                bx, by = ball.pose.position.x, ball.pose.position.y

                # -- approche au-dessus ------------------------------------------
                self._set_state("approach")
                if not self._ik_move([bx, by, PICK_APPROACH_Z], gripper=100.0, settle=2.0):
                    continue

                # -- descente -----------------------------------------------------
                self._set_state("down_to_ball")
                if not self._ik_move([bx, by, GRASP_Z], gripper=100.0, settle=1.5):
                    continue

                # -- saisie -------------------------------------------------------
                self._set_state("grasp")
                self._send_action({"gripper": 0.0}, settle=1.0)

                # -- remontée -----------------------------------------------------
                self._set_state("lift")
                self._ik_move([bx, by, PICK_APPROACH_Z], gripper=0.0, settle=1.5)

                # -- transport vers la boîte de dépôt -----------------------------
                self._set_state("transport")
                with self._lock:
                    px, py = self._drop_box_xy or PLACE_XY_FALLBACK
                if not self._ik_move([px, py, PLACE_APPROACH_Z], gripper=0.0, settle=2.0):
                    continue

                # -- pose ---------------------------------------------------------
                self._set_state("place")
                self._send_action({"gripper": 100.0}, settle=1.0)

                self._set_state("home")
                self._go_home()
                self.get_logger().info(f"Cycle #{cycle} terminé.")

            except Exception as e:
                self.get_logger().error(f"Erreur cycle : {e}")
                self._set_state("error")
                time.sleep(1.0)

        self._set_state("idle")
        self.get_logger().info("Boucle autonome arrêtée.")

    def _ik_move(self, target, gripper, settle):
        """IK + envoi. Retourne False (et repasse idle) si cible inatteignable."""
        q_sol, err = self._solve_ik(target)
        if q_sol is None:
            self.get_logger().warn(
                f"Cible {np.round(target, 3).tolist()} hors espace de travail "
                f"(erreur IK {err:.3f} m > {self._ik_tol} m) — retour idle."
            )
            self._set_state("idle")
            return False
        self._send_action(self._q_to_action(q_sol, gripper_pct=gripper), settle=settle)
        return True

    def _wait_ball(self, timeout=BALL_LOST_TIMEOUT):
        """Attendre une détection (le spin tourne dans le thread principal)."""
        deadline = time.monotonic() + timeout
        while self._autonomous and time.monotonic() < deadline:
            with self._lock:
                if self._ball_pose is not None:
                    return self._ball_pose
            time.sleep(0.2)
        return None

    # -- état -----------------------------------------------------------------------------

    def _set_state(self, state):
        with self._lock:
            self._state = state

    def _publish_state(self):
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
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
