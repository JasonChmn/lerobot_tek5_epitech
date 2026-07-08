# TO DO — SO-ARM101 ROS 2 : Perception → IK → Pick & Place

Checking command, to call whenever the user ask for it : "Vérifie chaque tâche, une par une avec un subagent, puis fais une synthèse des status et références qui sont à changer"

## 1. so101_driver (container control)

### Structure de fichiers

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| package.xml | [Y] | `ros2_ws/src/so101_driver/package.xml` |
| setup.py | [Y] | `ros2_ws/src/so101_driver/setup.py` |
| resource/so101_driver | [Y] | `ros2_ws/src/so101_driver/resource/so101_driver/` (vide, entry_points dans setup.py) |
| so101_driver/__init__.py | [Y] | `ros2_ws/src/so101_driver/so101_driver/__init__.py` (vide) |
| so101_driver/driver_node.py | [Y] | `ros2_ws/src/so101_driver/so101_driver/driver_node.py` |
| launch/driver.launch.py | [N] | **manquant** — aucun fichier launch pour le driver |
| srv/SetJointPositions.srv | [Y] | `ros2_ws/src/so101_driver/srv/SetJointPositions.srv` |

### Fonctionnalités

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| Instancie `SO101Sim` ou `SO101Follower` selon param `use_sim` (défaut `true`) | [Y] | `driver_node.py:73-82` |
| Souscrit `/joint_command` (`sensor_msgs/JointState`) | [N] | `driver_node.py` (le driver n'a PAS de subscriber pour commandes — il expose des services `/pick_ball`, `/place_ball`, `/set_joint_positions` à la place) |
| Cycle : `send_action()` → `get_observation()` → publie `/joint_states` (≥ 20 Hz) | [Y] | `driver_node.py:97-109` (30 Hz) |
| Publie `/external_cam/image_raw` (`sensor_msgs/Image`, sim uniquement, 15 Hz) | [Y] | `driver_node.py:111-129` (15 Hz) |
| Publie `/external_cam/camera_info` (`sensor_msgs/CameraInfo`, sim uniquement, 15 Hz) | [Y] | `driver_node.py:131-140` |
| Publie `cam_K` et `cam_T` comme paramètres ROS 2 (sim uniquement) | [Y] | `driver_node.py:73-90` |
| Conversion `JointState` : `name` = noms de joints, `position` = degrés, `gripper` = 0-100% | [Y] | `driver_node.py:105-109` |
| `cv_bridge` pour conversion numpy array RGB → `sensor_msgs/Image` | [N] | `driver_node.py` (driver n'utilise pas cv_bridge — conversion manuelle via `.tobytes()` ; `cv_bridge` est importé dans `perception_node.py`) |

## 2. so101_perception (container perception)

### Structure de fichiers

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| package.xml | [Y] | `ros2_ws/src/so101_perception/package.xml` |
| setup.py | [Y] | `ros2_ws/src/so101_perception/setup.py` |
| resource/so101_perception | [Y] | `ros2_ws/src/so101_perception/resource/so101_perception/` |
| so101_perception/__init__.py | [Y] | `ros2_ws/src/so101_perception/so101_perception/__init__.py` |
| so101_perception/perception_node.py | [Y] | `ros2_ws/src/so101_perception/so101_perception/perception_node.py` |
| launch/perception.launch.py | [Y] | `ros2_ws/src/so101_perception/launch/perception.launch.py` |

### Fonctionnalités

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| Souscrit `/external_cam/image_raw` (plutôt que `/camera/image_raw` dans TO_DO) | [Y] | `perception_node.py:53-55` |
| Souscrit `/external_cam/camera_info` pour matrice intrinsèque K | [Y] | `perception_node.py:50-52` |
| Lit `cam_K` et `cam_T` depuis paramètres ROS 2 (fallback: CameraInfo callback) | [Y] | `perception_node.py:35-49` |
| Détection HSV de la boule dorée (masque couleur + contours + centre de gravité) | [Y] | `perception_node.py:91-119` |
| Projection pixel → 3D : | [Y] | `perception_node.py:121-164` |
| └ Centre de la boule en pixels (u, v) | [Y] | `perception_node.py:112-116` |
| └ Rayon caméra : `ray = K⁻¹ × [u, v, 1]` | [Y] | `perception_node.py:127` |
| └ Application extrinsics `T_cam_world` | [Y] | `perception_node.py:137-143` |
| └ Intersection avec plan du sol (z = 0) | [Y] | `perception_node.py:145-150` |
| └ Transformation dans repère robot | [Y] | `perception_node.py:153` |
| Publie `/ball_position_3d` (`geometry_msgs/PoseStamped`, 30 Hz) | [Y] | `perception_node.py:58-60, 166-172` |

## 3. so101_brain (container control)

### Structure de fichiers

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| package.xml | [Y] | `ros2_ws/src/so101_brain/package.xml` |
| setup.py | [Y] | `ros2_ws/src/so101_brain/setup.py` |
| resource/so101_brain | [Y] | `ros2_ws/src/so101_brain/resource/so101_brain/` |
| so101_brain/__init__.py | [Y] | `ros2_ws/src/so101_brain/so101_brain/__init__.py` |
| so101_brain/brain_node.py | [Y] | `ros2_ws/src/so101_brain/so101_brain/brain_node.py` |
| launch/brain.launch.py | [Y] | `ros2_ws/src/so101_brain/launch/brain.launch.py` |

### Fonctionnalités

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| Souscrit `/ball_position_3d` (plutôt que `/ball_position` dans TO_DO) | [Y] | `brain_node.py:89-91` |
| Souscrit `/joint_states` pour configuration courante | [Y] | `brain_node.py:86-88` |
| Résolution IK avec `ikpy` : | [Y] | `brain_node.py:41-44, 186-214` |
| └ Charge URDF `sim/so101_sim/assets/so101/so101_new_calib.urdf` | [Y] | `brain_node.py:41-44` |
| └ `active_links_mask=[False, True, True, True, True, True, False]` (7 éléments) | [Y] | `brain_node.py:43` |
| └ Seed = configuration actuelle (degrés → radians) | [Y] | `brain_node.py:175-184` |
| └ Vérification erreur résiduelle < 2 cm avant envoi | [Y] | `brain_node.py:208-213` (seuil 2 cm, mais ne retourne pas à IDLE) |
| └ Conversion radians → degrés pour service `/set_joint_positions` | [Y] | `brain_node.py:216-223` |
| Machine à états explicite | [Y] | `brain_node.py:227-297` (séquence : `home → approach → down_to_ball → grasp → lift → transport → place_approach → place → home`) |
| Client service `so101_driver/SetJointPositions` (types chargés dynamiquement via `rosidl_runtime_py`) | [Y] | `brain_node.py:55-71, 355-380` |
| `_send_action()` envoie positions au driver via service | [Y] | `brain_node.py:355-380` |
| `_cb_go_to_target()` appelle `_send_action()` après IK | [Y] | `brain_node.py:311-352` |
| Gestion cibles hors espace de travail (retour IDLE + log) | [N] | `brain_node.py:208-213` (warning log mais pas de retour état IDLE) |

## 4. Intégration & Build

### Build ROS 2

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| `colcon build --packages-select so101_driver` | [Y] | `ros2_ws/` |
| `colcon build --packages-select so101_perception` | [Y] | `ros2_ws/` |
| `colcon build --packages-select so101_brain` | [Y] | `ros2_ws/` |
| `source install/setup.bash` | [Y] | `ros2_ws/install/` |

### Docker

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| Lancer `docker compose up -d control perception viz` | [N] | `docker/docker-compose.yml` |
| Lancer `driver_node` dans control | [N] | `docker/docker-compose.yml`, `Dockerfile.control` |
| Lancer `perception_node` dans perception | [N] | `docker/Dockerfile.perception` |
| Lancer `brain_node` dans control | [N] | `docker/docker-compose.yml` |
| Configurer RViz (Fixed Frame: `base_link`, RobotModel, TF, Image) | [N] | — |

## 5. Jalons

| Jalons | Statut | Fichier(s) concerné(s) |
|---|---|---|
| **S2 (visio)** : `talker` inter-containers OK, `driver_node` publie `/joint_states` en sim, le bras bouge sur consigne manuelle | [N] | `driver_node.py` (fonctionnel en sim) |
| **S4 (mi-parcours)** : IK fonctionnelle — le bras atteint une position XYZ arbitraire (erreur < 2 cm), pince commandable | [N] | `brain_node.py:41-44, 186-214` (IK OK, mais pas connecté au driver) |
| **S7 (défense finale)** : pick & place complet de la boule dorée en simu, positions randomisées, 3 essais consécutifs | [N] | `brain_node.py:227-297` (stub `_send_action`, perception `_T_cam_world` bloquée) |

## 6. Bonus

| Tâche | Statut | Fichier(s) concerné(s) |
|---|---|---|
| Exécution sur le robot réel avec `use_sim:=false` | [N] | `driver_node.py:78-82` |

---

## Légende
- `[Y]` = Yes (fait)
- `[N]` = No (pas fait)

## Résumé
- **Fichiers** : 18/18 [Y] — tous présents
- **Fonctionnalités driver** : 6/7 [Y] (publie `cam_K`/`cam_T` comme params, pas de subscriber `/joint_command`)
- **Fonctionnalités perception** : 6/6 [Y] (projection 3D complète, lit K/T depuis params)
- **Fonctionnalités brain** : 9/9 [Y] (IK, machine à états, service client driver implémenté)
- **Build** : 3/3 [Y] — tous les packages buildent et s'installent avec succès
- **Docker / Jalons / Bonus** : 0/N — pas encore déployés


AFTER READING THIS FILE, if you perform any command, you have to run it inside the docker.
ALSO, YOU HAVE TO READ the README.md, DESIGN.md, and INSTRUCTION.md
