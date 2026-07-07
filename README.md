# tek5-robotics — SO-ARM101 : ROS2, IK, perception, pick & place

Module robotique Tek5 (Epitech). Un bras SO-ARM101 (écosystème
LeRobot/HuggingFace), un jumeau numérique MuJoCo, et un pipeline complet
perception → IK → pick & place à construire en ROS2.

## Structure

```
core/lerobot_min/   Driver réel SO-ARM101 (extrait minimal de LeRobot, Apache-2.0)
sim/                Jumeau numérique MuJoCo (SO101Sim, même API que le driver réel)
docker/             Images control + perception, docker-compose (réseau DDS partagé)
ros2_ws/            Workspace étudiant (contient un package d'exemple)
docs/               SUJET.md (sujet étudiant) · README_IK.md (guide IK/ikpy)
scripts/            Démos et outils instructeur
```

## Démarrage rapide

```bash
cd docker
docker compose build
docker compose up -d control perception
docker compose exec control bash    # terminal 1
docker compose exec perception bash # terminal 2
# dans chaque : cd /ros2_ws && colcon build && source install/setup.bash
```

Test de la sim sans ROS2 (dans le container control) :

```bash
python3 -c "from so101_sim import SO101Sim; s=SO101Sim(); s.connect(); print(list(s.get_observation())); s.disconnect()"
```

## Bras réel (bonus, Linux uniquement)

```bash
# identifier le port : ls /dev/ttyACM*
docker compose --profile real up -d control-real
# calibration première utilisation : voir scripts/calibrate_real.py
```

## Provenance et licences

- `core/lerobot_min` : sous-ensemble de [LeRobot](https://github.com/huggingface/lerobot)
  (Apache-2.0, HuggingFace Inc.), réduit aux modules moteurs Feetech,
  robot SO-follower et caméras OpenCV. Fichiers modifiés annotés en tête.
- `sim/so101_sim/assets/so101` : modèles URDF/MJCF officiels
  [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
  (variante SO-101). `so101_tek5.xml` = recoloration (bras gris) pour que la
  boule dorée soit le seul objet or de la scène.
