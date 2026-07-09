# tek5-robotics — SO-ARM101 : ROS2, IK, perception, pick & place

Module robotique Tek5 (Epitech). Un bras SO-ARM101 (écosystème
LeRobot/HuggingFace), un jumeau numérique MuJoCo, et un pipeline complet
perception → IK → pick & place à construire en ROS2.

**Prérequis : hôte Linux** (natif ou VM) avec Docker. La visualisation
(RViz) et le bonus bras réel l'exigent — c'est l'environnement du métier.

## Structure
core/lerobot_min/   Driver réel SO-ARM101 (extrait minimal de LeRobot, Apache-2.0)
sim/                Jumeau numérique MuJoCo (SO101Sim, même API que le driver réel)
docker/             Images control + perception, docker-compose (réseau DDS partagé)
ros2_ws/            Workspace étudiant (contient un package d'exemple)
docs/               SUJET.md (sujet étudiant) · README_IK.md (guide IK/ikpy)
scripts/            Démos et outils instructeur

## Démarrage rapide

```bash
cd docker
docker compose build
xhost +local:docker                      # accès X11 (RViz)
docker compose up -d control perception viz
docker compose exec control bash    # terminal 1
docker compose exec perception bash # terminal 2
# dans chaque : cd /ros2_ws && colcon build && source install/setup.bash
```

Test de la sim sans ROS2 (dans le container control) :

```bash
python3 -c "from so101_sim import SO101Sim; s=SO101Sim(); s.connect(); print(list(s.get_observation())); s.disconnect()"
```

Visualisation (RViz depuis le container control) :

```bash
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && rviz2"
```

Bouger un joint via ROS2 (le bras doit suivre dans RViz — jalon S2) :

```bash
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  '{name: [shoulder_pan], position: [30.0]}'"
```

Conventions : `/joint_command` en degrés (gripper 0-100 %),
`/joint_states` en radians (REP-103, requis par robot_state_publisher).

Voir `docs/INSTRUCTION.md` §5 pour la configuration RViz et le dépannage.

> **Note pédagogique** : aucune config RViz pré-chargée sur la branche étudiante.
> Les étudiants doivent ajouter eux-mêmes les panneaux TF + RobotModel
> (`/robot_description`), ce qui force la compréhension des arbres de transformation
> et du `joint_states` manquant (bras en morceaux = driver non démarré).
> La config se sauvegarde via `File → Save Config` dans le workspace.

## Bras réel (bonus)

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
