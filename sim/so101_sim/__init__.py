# SO101Sim — backend simulation MuJoCo du SO-ARM101.
#
# Expose STRICTEMENT la même interface que lerobot.robots.so_follower.SO101Follower :
#   connect() / disconnect() / get_observation() / send_action() / is_connected
#
# Le nœud driver ROS2 écrit par les étudiants doit fonctionner à l'identique
# avec SO101Sim ou SO101Follower (paramètre ROS2 `use_sim`).
#
# Unités : degrés pour les 5 joints du bras, 0-100 pour le gripper
# (équivalent de SO101FollowerConfig(use_degrees=True) sur le bras réel).

import time
from pathlib import Path

import mujoco
import numpy as np

ASSETS_DIR = Path(__file__).resolve().parent / "assets" / "so101"
DEFAULT_SCENE = ASSETS_DIR / "scene_tek5.xml"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

# Ouverture gripper : mapping 0-100 (convention LeRobot) <-> radians MJCF
GRIPPER_RANGE_RAD = None  # rempli à l'init depuis le modèle


class SO101Sim:
    """Jumeau numérique du SO-ARM101. API identique à SO101Follower."""

    def __init__(
        self,
        scene_path: str | Path = DEFAULT_SCENE,
        camera_name: str = "external_cam",
        camera_width: int = 320,
        camera_height: int = 240,
        sim_dt_per_step: int = 5,
        seed: int | None = None,
    ):
        self.scene_path = str(scene_path)
        self.camera_name = camera_name
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.sim_dt_per_step = sim_dt_per_step
        self._rng = np.random.default_rng(seed)

        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._connected = False
        self._last_step_t = 0.0

    # ------------------------------------------------------------------ API

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:  # signature alignée sur le réel
        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)

        self._joint_qpos_adr = {
            name: self.model.joint(name).qposadr[0] for name in ALL_JOINTS
        }
        self._actuator_id = {
            name: self.model.actuator(name).id for name in ALL_JOINTS
        }
        g_jnt = self.model.joint(GRIPPER)
        self._gripper_range = (float(g_jnt.range[0]), float(g_jnt.range[1]))

        mujoco.mj_forward(self.model, self.data)
        self.randomize_ball()
        self._connected = True
        self._last_step_t = time.perf_counter()

    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._connected = False

    def get_observation(self) -> dict:
        """Clés identiques au bras réel ('<joint>.pos').
        L'image caméra n'est PAS incluse ; les appelants doivent appeler
        render_camera() explicitement lorsqu'ils en ont besoin.
        """
        self._require_connected()
        self._step_sim()
        obs = {}
        for name in ARM_JOINTS:
            obs[f"{name}.pos"] = float(
                np.degrees(self.data.qpos[self._joint_qpos_adr[name]])
            )
        obs[f"{GRIPPER}.pos"] = self._gripper_rad_to_pct(
            self.data.qpos[self._joint_qpos_adr[GRIPPER]]
        )
        return obs

    def send_action(self, action: dict) -> dict:
        """Consignes de position. Clés attendues : '<joint>.pos' (degrés, gripper 0-100)."""
        self._require_connected()
        applied = {}
        for key, val in action.items():
            name = key.removesuffix(".pos")
            if name not in self._actuator_id:
                continue
            if name == GRIPPER:
                target = self._gripper_pct_to_rad(val)
            else:
                target = np.radians(val)
            self.data.ctrl[self._actuator_id[name]] = target
            applied[key] = val
        self._step_sim()
        return applied

    # ------------------------------------------------------- utilitaires sim

    def render_camera(self) -> np.ndarray:
        """Image RGB (H, W, 3) uint8 de la caméra externe fixe."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self.camera_height, width=self.camera_width
            )
        self._renderer.update_scene(self.data, camera=self.camera_name)
        return self._renderer.render()

    def randomize_ball(self, radius_range=(0.18, 0.30), angle_range_deg=(-50, 60)) -> None:
        """Replace la boule dorée à une position aléatoire atteignable."""
        r = self._rng.uniform(*radius_range)
        a = np.radians(self._rng.uniform(*angle_range_deg))
        adr = self.model.joint("gold_ball_free").qposadr[0]
        self.data.qpos[adr : adr + 3] = [r * np.cos(a), r * np.sin(a), 0.03]
        self.data.qpos[adr + 3 : adr + 7] = [1, 0, 0, 0]
        self.data.qvel[:] = 0
        # laisser la boule se poser au sol (settling physique)
        for _ in range(200):
            mujoco.mj_step(self.model, self.data)

    def get_ball_position(self) -> np.ndarray:
        """Position monde de la boule — DEBUG/ÉVALUATION uniquement.
        Interdit dans le pipeline étudiant : la position doit venir de la perception."""
        return self.data.body("gold_ball").xpos.copy()

    def get_drop_box_position(self) -> np.ndarray:
        """Position monde de la drop_box."""
        return self.data.body("drop_box").xpos.copy()

    def get_camera_extrinsics(self) -> np.ndarray:
        """Pose caméra->monde (4x4). C'est la 'TF fournie' du sujet."""
        cam = self.data.camera(self.camera_name)
        T = np.eye(4)
        T[:3, :3] = cam.xmat.reshape(3, 3)
        T[:3, 3] = cam.xpos
        return T

    def get_camera_intrinsics(self) -> np.ndarray:
        """Matrice K (3x3) dérivée du fovy MJCF et de la résolution de rendu."""
        cam_id = self.model.camera(self.camera_name).id
        fovy = np.radians(self.model.cam_fovy[cam_id])
        fy = self.camera_height / (2 * np.tan(fovy / 2))
        fx = fy  # pixels carrés
        return np.array(
            [
                [fx, 0, self.camera_width / 2],
                [0, fy, self.camera_height / 2],
                [0, 0, 1],
            ]
        )

    # ---------------------------------------------------------------- privé

    def _step_sim(self) -> None:
        for _ in range(self.sim_dt_per_step):
            mujoco.mj_step(self.model, self.data)

    def _gripper_pct_to_rad(self, pct: float) -> float:
        lo, hi = self._gripper_range
        return lo + np.clip(pct, 0, 100) / 100.0 * (hi - lo)

    def _gripper_rad_to_pct(self, rad: float) -> float:
        lo, hi = self._gripper_range
        return float(np.clip((rad - lo) / (hi - lo) * 100.0, 0, 100))

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("SO101Sim non connecté : appeler connect() d'abord.")
