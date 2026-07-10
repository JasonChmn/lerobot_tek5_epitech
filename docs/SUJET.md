# Tek5 — Bras SO-ARM101 : simulation, IK, perception, pick & place

## Ce qu'on vous donne

| Élément | Où | Rôle |
|---|---|---|
| `so101-core` | `core/lerobot_min/` (installé dans l'image `control`) | Driver du bras **réel** : `SO101Follower` (bus série Feetech). Extrait du projet LeRobot (HuggingFace) |
| `so101-sim` | `sim/` (installé dans l'image `control`) | Jumeau numérique MuJoCo : `SO101Sim`, **même API** que `SO101Follower` |
| Scène simulée | `sim/so101_sim/assets/so101/scene_tek5.xml` | Bras + boule dorée + boîte de dépôt + caméra externe fixe |
| URDF | `sim/so101_sim/assets/so101/so101_new_calib.urdf` | Pour votre IK (voir `docs/README_IK.md`) |
| TF caméra | `SO101Sim.get_camera_extrinsics()` / `get_camera_intrinsics()` | Pose et matrice K de la caméra externe — la calibration est **fournie** |
| Docker | `docker/` | Deux containers : `control` et `perception`, réseau ROS2 partagé |
| Exemple ROS2 | `ros2_ws/src/example_py_pkg` | Package minimal à copier — vérifiez la comm inter-containers avec `talker` avant tout |

L'API commune aux deux backends :

```python
robot.connect()
obs = robot.get_observation()   # {"shoulder_pan.pos": deg, ..., "gripper.pos": 0-100,
                                #  "external_cam": image RGB (sim uniquement)}
robot.send_action({"shoulder_pan.pos": 20.0, "gripper.pos": 50.0})
robot.disconnect()
```

## Ce que vous écrivez

Tout le reste. En particulier, trois nœuds ROS2 minimum, répartis sur les
deux containers :

### 1. `driver_node` (container control)
- Instancie `SO101Sim` **ou** `SO101Follower` selon un paramètre ROS2
  `use_sim` (défaut `true`). Le reste de votre stack ne doit **jamais**
  savoir lequel tourne.
- Publie `/joint_states` (`sensor_msgs/JointState`, ≥20 Hz).
- Publie `/external_cam/image_raw` (`sensor_msgs/Image`, ~10 Hz, en sim).
- Souscrit `/joint_command` (`sensor_msgs/JointState` : consignes en degrés,
  gripper 0-100).

### 2. `perception_node` (container perception)
- Souscrit `/external_cam/image_raw`.
- Détecte la boule dorée (HSV, YOLO, ce que vous voulez — justifiez).
- Calcule sa position 3D dans le **repère robot** et publie
  `/ball_position` (`geometry_msgs/PointStamped`, 5-10 Hz).
- Le vrai problème n'est pas la détection (triviale) mais **pixel → 3D** :
  intrinsics K + extrinsics fournis + une hypothèse que vous devez
  identifier vous-mêmes (indice : la boule ne vole pas).

### 3. `brain_node` (container control)
- Orchestration : lit `/ball_position`, calcule l'IK (ikpy, voir
  `docs/README_IK.md`), séquence approche → saisie → transport → dépôt
  dans la boîte.
- Machine à états explicite exigée. Gestion de l'asynchronisme : le contrôle
  ne bloque **jamais** en attente de perception (dernière valeur connue).

## Visualiser votre robot

Le service `viz` du compose (`docker compose up -d control viz`) publie le
modèle du bras et expose un pont websocket sur le port 8765.

- **Foxglove Studio** (recommandé, tous OS) : app native ou
  https://app.foxglove.dev → *Open connection* → `ws://localhost:8765` →
  panneau 3D (ajouter *RobotModel*) + panneau *Image* sur votre topic caméra.
- **RViz2** (équivalent, si vous avez ROS2 Jazzy installé nativement sur
  Ubuntu) : mêmes topics, mêmes affichages (*RobotModel*, *TF*, *Image*).
  Attention : RViz doit voir le graphe DDS — lancez vos containers en
  `--network host` (Linux uniquement) avec le même `ROS_DOMAIN_ID`.
  Aucun avantage fonctionnel pour ce projet ; c'est l'outil standard de
  l'écosystème ROS, utile à connaître.

Dans les deux cas : tant que votre `driver_node` ne publie pas
`/joint_states`, le bras apparaît **en morceaux** — c'est normal, c'est le
jalon S2. La boule n'apparaîtra que si votre perception la publie (Marker) :
la visu affiche ce que *votre* code voit, pas la vérité terrain.

## Contraintes

- Consignes moteur : passer par `/joint_command`, jamais d'accès direct au
  backend depuis `brain_node` ou `perception_node`.
- `get_ball_position()` de la sim est **interdit** dans votre pipeline
  (autorisé dans vos tests unitaires pour valider votre perception).
- Fréquences découplées : perception lente (5-10 Hz), driver ≥20 Hz.
- Code sous Git, historique par membre visible.

## Jalons

- **S2 (visio)** : `talker` inter-containers OK, `driver_node` publie
  `/joint_states` en sim, le bras bouge sur consigne manuelle
  (`ros2 topic pub /joint_command ...`).
- **S4 (soutenance mi-parcours)** : IK fonctionnelle — le bras atteint une
  position XYZ arbitraire (erreur < 1 cm, vérifiée), pince commandable.
- **S7 (défense finale)** : pick & place complet de la boule dorée en simu,
  positions de boule randomisées (`randomize_ball()`), 3 essais consécutifs.
- **Bonus** : même code, `use_sim:=false`, sur le bras physique.

## Barème indicatif

A : pipeline robuste (randomisation, cas limites gérés, architecture propre) ·
B : pick & place fonctionnel · C : IK + perception séparées fonctionnelles
mais intégration fragile · D : bras contrôlable en position via ROS2.
