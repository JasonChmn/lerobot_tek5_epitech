"""Visu live du jumeau numérique — fenêtre interactive MuJoCo, bras animé.

À lancer HORS Docker, sur une machine avec affichage (X11/Wayland) :
    pip install mujoco
    python3 scripts/view_live.py [--seconds 60] [--cam]

Contrôles fenêtre : clic droit = orbite, molette = zoom,
double-clic sur un corps + Ctrl+drag = appliquer une force (perturber la boule).

--cam : affiche en plus la caméra externe (la vue qu'auront les étudiants)
        dans une fenêtre OpenCV. Nécessite opencv-python (PAS headless).

Ne fonctionne pas dans le container (MUJOCO_GL=osmesa = offscreen only).
"""
import argparse
import math
import sys
import time

sys.path.insert(0, "sim")

import mujoco.viewer
from so101_sim import SO101Sim


def targets(t: float) -> dict:
    """Trajectoire de démo : balayage pan + flexion + cycle gripper."""
    return {
        "shoulder_pan.pos": 40.0 * math.sin(0.6 * t),
        "shoulder_lift.pos": -25.0 + 15.0 * math.sin(0.9 * t),
        "elbow_flex.pos": 35.0 + 20.0 * math.sin(0.7 * t + 1.0),
        "wrist_flex.pos": 20.0 * math.sin(1.1 * t),
        "wrist_roll.pos": 60.0 * math.sin(0.4 * t),
        "gripper.pos": 50.0 + 50.0 * math.sin(1.5 * t),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--cam", action="store_true", help="fenêtre OpenCV caméra externe")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    cv2 = None
    if args.cam:
        import cv2  # noqa: F401 — import tardif, optionnel

    s = SO101Sim(seed=args.seed)
    s.connect()

    # launch_passive : la physique reste pilotée par SO101Sim (send_action),
    # le viewer ne fait que refléter model/data à chaque sync().
    with mujoco.viewer.launch_passive(s.model, s.data) as viewer:
        t0 = time.perf_counter()
        last_cam = 0.0
        while viewer.is_running() and (time.perf_counter() - t0) < args.seconds:
            t = time.perf_counter() - t0
            s.send_action(targets(t))  # steppe la sim en interne
            viewer.sync()
            if args.cam and (t - last_cam) > 0.1:  # caméra à ~10 Hz, comme le sujet
                img = s.render_camera()
                cv2.imshow("external_cam (vue etudiants)", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
                last_cam = t
            time.sleep(0.01)  # ~100 Hz max côté hôte, cohérent avec le bras réel

    if args.cam:
        cv2.destroyAllWindows()
    s.disconnect()


if __name__ == "__main__":
    main()
