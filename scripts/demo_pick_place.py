#!/usr/bin/env python3
"""demo_pick_place.py — démo instructeur : pick & place complet en simulation.

Séquence : home → approche au-dessus de la boule (pince vers le bas, -Z) →
descente → fermeture pince → remontée → transport vers la drop_box → ouverture
→ home. Sans ROS2 : SO101Sim + ikpy directement. Utilise la vérité terrain
(get_ball_position / get_drop_box_position) — AUTORISÉ ici (démo instructeur,
hors pipeline étudiant, cf. DESIGN §7).

Usage :
    python3 scripts/demo_pick_place.py            # démo, frames dans demo_frames/
    python3 scripts/demo_pick_place.py --no-frames
    python3 scripts/demo_pick_place.py --seed 3

Équivalent ROS2 (pipeline complet, containers lancés) :
    ros2 service call /brain/start std_srvs/srv/Trigger
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, "sim")

import numpy as np
from ikpy import chain as ik_chain

from so101_sim import SO101Sim

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Stratégie (m) — cohérente avec brain_node.py
PICK_APPROACH_Z = 0.10
GRASP_Z = 0.02            # boule r=0.015 posée au sol
PLACE_APPROACH_Z = 0.08   # z=0.10 avec contrainte -Z stricte = hors workspace (erreur IK 31 mm)
IK_TOL = 0.02

STEPS_PER_WP = 180        # ré-application de la consigne (converge comme le driver 50 Hz)
GRIP_STEPS = 80


def find_urdf() -> str:
    try:
        from so101_sim import ASSETS_DIR
        p = Path(ASSETS_DIR) / "so101_new_calib.urdf"
        if p.exists():
            return str(p)
    except ImportError:
        pass
    p = Path(__file__).resolve().parents[1] / "sim/so101_sim/assets/so101/so101_new_calib.urdf"
    if p.exists():
        return str(p)
    raise FileNotFoundError("so101_new_calib.urdf introuvable")


def solve_ik(chain, target, seed):
    """IK contrainte pince vers le bas (-Z) ; fallback orientation libre si la
    contrainte sort la cible du workspace (cf. transport : 31 mm sinon)."""
    q = chain.inverse_kinematics(
        target_position=target, initial_position=seed,
        orientation_mode="Z", target_orientation=[0, 0, -1],
    )
    err = float(np.linalg.norm(chain.forward_kinematics(q)[:3, 3] - target))
    if err > IK_TOL:
        q = chain.inverse_kinematics(target_position=target, initial_position=seed)
        err = float(np.linalg.norm(chain.forward_kinematics(q)[:3, 3] - target))
    if err > IK_TOL:
        raise RuntimeError(f"cible {np.round(target,3).tolist()} inatteignable (err {err*1000:.1f} mm)")
    return q, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0, help="seed sim (position boule)")
    ap.add_argument("--no-frames", action="store_true", help="ne pas écrire d'images")
    ap.add_argument("--outdir", default="demo_frames")
    args = ap.parse_args()

    frames_dir = None
    if not args.no_frames:
        frames_dir = Path(args.outdir)
        frames_dir.mkdir(exist_ok=True)
        import cv2  # noqa: F401 (import tardif : inutile en --no-frames)

    chain = ik_chain.Chain.from_urdf_file(
        find_urdf(), active_links_mask=[False, True, True, True, True, True, False]
    )

    sim = SO101Sim(seed=args.seed)
    sim.connect()
    frame_i = 0

    def snap(tag):
        nonlocal frame_i
        if frames_dir is None:
            return
        import cv2
        img = sim.get_observation()["external_cam"]
        cv2.imwrite(str(frames_dir / f"{frame_i:02d}_{tag}.png"),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        frame_i += 1

    def apply(action, steps):
        for _ in range(steps):
            sim.send_action(action)

    def q_to_action(q, gripper):
        a = {f"{j}.pos": float(d) for j, d in zip(ARM_JOINTS, np.degrees(q[1:6]))}
        a["gripper.pos"] = float(gripper)
        return a

    def current_seed():
        obs = sim.get_observation()
        return [0.0] + [np.radians(obs[f"{j}.pos"]) for j in ARM_JOINTS] + [0.0]

    ball = np.asarray(sim.get_ball_position(), dtype=float)
    box = np.asarray(sim.get_drop_box_position(), dtype=float)
    print(f"boule : {np.round(ball, 3).tolist()} | drop_box : {np.round(box, 3).tolist()}")

    # -- home, pince ouverte --------------------------------------------------
    apply({f"{j}.pos": 0.0 for j in ARM_JOINTS} | {"gripper.pos": 100.0}, STEPS_PER_WP)
    snap("home")

    waypoints = [
        ("approach",  [ball[0], ball[1], PICK_APPROACH_Z],  100.0),
        ("grasp_pos", [ball[0], ball[1], GRASP_Z],          100.0),
    ]
    for tag, tgt, grip in waypoints:
        q, err = solve_ik(chain, tgt, current_seed())
        print(f"{tag:9s} -> {np.round(tgt,3).tolist()}  (IK {err*1000:.1f} mm)")
        apply(q_to_action(q, grip), STEPS_PER_WP)
        snap(tag)

    # -- fermeture pince (saisie) ---------------------------------------------
    obs = sim.get_observation()
    hold = {f"{j}.pos": obs[f"{j}.pos"] for j in ARM_JOINTS} | {"gripper.pos": 0.0}
    apply(hold, GRIP_STEPS)
    snap("grasp")

    waypoints = [
        ("lift",      [ball[0], ball[1], PICK_APPROACH_Z],  0.0),
        ("transport", [box[0],  box[1],  PLACE_APPROACH_Z], 0.0),
    ]
    for tag, tgt, grip in waypoints:
        q, err = solve_ik(chain, tgt, current_seed())
        print(f"{tag:9s} -> {np.round(tgt,3).tolist()}  (IK {err*1000:.1f} mm)")
        apply(q_to_action(q, grip), STEPS_PER_WP)
        snap(tag)

    # -- lâcher --------------------------------------------------------------
    obs = sim.get_observation()
    hold = {f"{j}.pos": obs[f"{j}.pos"] for j in ARM_JOINTS} | {"gripper.pos": 100.0}
    apply(hold, GRIP_STEPS)
    snap("release")

    apply({f"{j}.pos": 0.0 for j in ARM_JOINTS} | {"gripper.pos": 100.0}, STEPS_PER_WP)
    snap("home_end")

    # -- verdict (vérité terrain, OK en démo) ---------------------------------
    final = np.asarray(sim.get_ball_position(), dtype=float)
    dxy = float(np.linalg.norm(final[:2] - box[:2]))
    print(f"boule finale : {np.round(final,3).tolist()} | distance drop_box (xy) : {dxy*1000:.0f} mm")
    print("SUCCÈS" if dxy < 0.06 else "ÉCHEC (boule hors zone de dépôt)")
    sim.disconnect()


if __name__ == "__main__":
    main()
