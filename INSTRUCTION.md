# INSTRUCTION — Runbook de test (Docker)

Blocs copy-paste, un bloc = un scénario. **Tout tourne en Docker.**
Explications, pièges GL, config RViz détaillée et dépannage :
`README_DETAILED.md` §10 (Annexe). Outils instructeur local (viewer MuJoCo) :
README_DETAILED §6.
Prérequis : hôte Linux, Docker.

---

## 1. From scratch → pick & place qui tourne (le happy path)

```bash
cd docker
docker compose build                          # première fois / deps changées
xhost +local:docker                           # une fois par session hôte
docker compose up -d control perception viz   # control : colcon build + driver + brain
docker compose logs -f control                # attendre "Brain node prêt." (Ctrl-C)
docker compose exec control bash              # ── entre dans le container ──
```

Dans le container (toutes les commandes suivantes) :

```bash
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
ros2 service call /brain/start std_srvs/srv/Trigger
ros2 topic echo /brain_state
# attendu : home → approach → down_to_ball → grasp → lift → transport → place → home (cycle)
ros2 service call /brain/stop std_srvs/srv/Trigger
```

La démo se regarde dans **RViz** (bloc 2) : bras qui bouge + flux caméra.

## 2. RViz (nouveau terminal, sur l'hôte)

```bash
cd docker
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && \
  source /ros2_ws/install/setup.bash && rviz2 -d /ros2_ws/tek5_prof.rviz"
# config préchargée (RobotModel + TF + Image). Sans -d : RViz vierge (cf. §10.2).
# fenêtre noire / crash GL → préfixer : LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d ...
```

Config minimale : Fixed Frame `base_link` · Add RobotModel
(`/robot_description`) · Add Image (`/external_cam/image_raw`).
Détail pas-à-pas + Save Config : README_DETAILED §10.2.

## 3. Tests unitaires du pipeline (dans le container control)

```bash
# bouger un joint (jalon S2) :
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan], position: [30.0]}"
ros2 topic echo /joint_states --once            # radians

# la perception voit la boule ?
ros2 topic echo /ball_position_3d --once

# IK vers une cible (jalon S4, sans boucle) :
ros2 service call /brain/go_to so101_interfaces/srv/GoToTarget \
  "{target_pose: {pose: {position: {x: 0.25, y: 0.05, z: 0.10}}}}"

# jog moteurs :
python3 /opt/so101/scripts/jog.py status
python3 /opt/so101/scripts/jog.py sweep elbow_flex
```

## 4. Pick & place standalone (dans le container, sans ROS2)

Solution de référence directe (SO101Sim + ikpy, sans le pipeline ROS2).
Vérité terrain autorisée : démo instructeur hors pipeline.

```bash
docker compose exec control bash -lc "cd /opt/so101 && \
  python3 scripts/demo_pick_place.py --no-frames"           # verdict en console
docker compose exec control bash -lc "cd /opt/so101 && \
  python3 scripts/demo_pick_place.py --seed 3 --outdir /ros2_ws/demo_frames"  # frames récupérables
# affiche l'erreur IK par waypoint + verdict (distance boule↔boîte < 60 mm)
```

## 5. Smoke test image (dans le container)

```bash
docker compose exec control bash -lc \
  "python3 /opt/so101/scripts/demo_sim.py /ros2_ws/sortie.png"
# une image écrite dans ros2_ws/ (visible sur l'hôte via le volume) = sim OK
```

## 6. Rebuild après modif de code

```bash
cd docker
docker compose restart control    # re-exécute colcon build + relance driver/brain
```

## 7. Bras réel (bonus)

```bash
cd docker
# identifier le port sur l'hôte : ls /dev/ttyACM*
docker compose --profile real up -d control-real
# scan / calibration / jog, dans le container real :
docker compose exec control-real bash -lc \
  "python3 /opt/so101/scripts/jog.py --real --port /dev/ttyACM0 scan"       # 6 moteurs, IDs 1-6
docker compose exec control-real bash -lc \
  "python3 /opt/so101/scripts/calibrate_real.py /dev/ttyACM0 tek5_arm"      # première utilisation
docker compose exec control-real bash -lc \
  "python3 /opt/so101/scripts/jog.py --real shoulder_pan 20"
```

## 8. Nettoyage

```bash
sudo rm -rf ros2_ws/build ros2_ws/install ros2_ws/log   # créés root par le container
```

---

## Si ça casse

| Symptôme | Voir |
|---|---|
| `could not connect to display` / fenêtre noire RViz | README_DETAILED §10.2 |
| `/brain_state` bloqué sur `home` (pas de détection) | `ros2 topic echo /ball_position_3d` + logs perception |
| retour `idle` + warn IK | cible hors workspace, erreur FK dans les logs |
| bras immobile / en morceaux dans RViz | driver pas lancé ou build raté : `docker compose logs control` |
