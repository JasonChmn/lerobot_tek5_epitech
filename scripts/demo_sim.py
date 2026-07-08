"""Démo instructeur : rend une image de la scène et fait bouger le bras.

Usage local (hôte, aucune variable env) : python3 scripts/demo_sim.py [sortie.png]
Dans Docker : les variables MUJOCO_GL/PYOPENGL_PLATFORM sont déjà dans l'image.
Pour la fenêtre interactive (hors Docker, avec affichage) :
    python3 -m mujoco.viewer --mjcf sim/so101_sim/assets/so101/scene_tek5.xml
"""
import sys
sys.path.insert(0, "sim")

import cv2
from so101_sim import SO101Sim

out = sys.argv[1] if len(sys.argv) > 1 else "sim_view.png"
s = SO101Sim(seed=0)
s.connect()
for _ in range(200):
    s.send_action({"shoulder_pan.pos": 25.0, "elbow_flex.pos": 30.0, "gripper.pos": 80.0})
img = s.get_observation()["external_cam"]
cv2.imwrite(out, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
print(f"image écrite : {out} | boule (vérité terrain) : {s.get_ball_position()}")
s.disconnect()
