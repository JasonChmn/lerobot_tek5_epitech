# TO DO — SO-ARM101 ROS 2 : Perception → IK → Pick & Place (branche prof)

Checking command : "Vérifie chaque tâche, une par une avec un subagent, puis fais une synthèse des status et références qui sont à changer"

État au 09/07/2026 — après refonte : `/joint_command` implémenté, srv déplacés
dans `so101_interfaces`, `/joint_states` en radians (RViz), projection caméra
−Z corrigée. Les statuts ci-dessous reflètent le code, pas les intentions.

## 0. so101_interfaces (NOUVEAU — ament_cmake, rosidl)

| Tâche | Statut | Fichier(s) |
|---|---|---|
| SetJointPositions.srv (float64[] → success/message) | [Y] | `ros2_ws/src/so101_interfaces/srv/SetJointPositions.srv` |
| GoToTarget.srv (PoseStamped → success/message) | [Y] | `ros2_ws/src/so101_interfaces/srv/GoToTarget.srv` |
| package.xml / CMakeLists.txt (rosidl + geometry_msgs) | [Y] | `ros2_ws/src/so101_interfaces/` |

Rationale : la génération rosidl exige ament_cmake ; l'ancien hybride
CMakeLists+setup.py dans so101_driver ne buildait pas (cf. logs du 08/07,
cible `ament_cmake_python_copy_so101_driver` dupliquée).

## 1. so101_driver (container control — ament_python pur)

| Tâche | Statut | Fichier(s) |
|---|---|---|
| Instancie `SO101Sim` / `SO101Follower` selon `use_sim` (défaut true) | [Y] | `driver_node.py` `_connect_arm()` |
| Souscrit `/joint_command` (sensor_msgs/JointState, DEGRÉS, gripper 0-100 %) | [Y] | `driver_node.py` `_cb_joint_command()` |
| Boucle de contrôle 50 Hz : ré-applique la consigne via `send_action()` | [Y] | `driver_node.py` `_control_step()` |
| Publie `/joint_states` en **RADIANS** (REP-103, requis par robot_state_publisher/RViz), 30 Hz | [Y] | `driver_node.py` `_publish_joint_states()` |
| Publie `/external_cam/image_raw` + `/external_cam/camera_info` (sim, 15 Hz, image réutilisée de get_observation — pas de double rendu OSMesa) | [Y] | `driver_node.py` `_publish_camera()` |
| Publie `cam_K` / `cam_T` comme paramètres ROS2 (sim) | [Y] | `driver_node.py` `_connect_arm()` |
| Service `/set_joint_positions` (so101_interfaces, non bloquant) | [Y] | `driver_node.py` `_cb_set_joints()` |
| Services démo `/pick_ball` `/place_ball` (MultiThreadedExecutor + groupe réentrant — ne bloquent pas les timers) | [Y] | `driver_node.py` |
| Launch avec arg `use_sim` (ParameterValue bool) + `port` | [Y] | `launch/driver.launch.py` |
| cv_bridge pour l'image | [N] | conversion manuelle `.tobytes()` conservée (suffisante, une dépendance de moins côté control) |

## 2. so101_perception (container perception)

| Tâche | Statut | Fichier(s) |
|---|---|---|
| Souscrit `/external_cam/image_raw` + `/external_cam/camera_info` | [Y] | `perception_node.py` |
| Lit `cam_K`/`cam_T` depuis paramètres (fallback CameraInfo) | [Y] | `perception_node.py` |
| Détection HSV boule dorée | [Y] | `perception_node.py` `_detect_ball()` |
| Projection pixel → 3D avec conversion convention caméra MuJoCo (−Z avant, +Y haut) : ray = [x, −y, −z] | [Y] | `perception_node.py` `_project_to_3d()` — validé 3.8 mm d'erreur vs vérité sim |
| Intersection au plan z = rayon boule (param `ball_radius`, défaut 0.015) | [Y] | `perception_node.py` |
| Publie `/ball_position_3d` (PoseStamped) | [Y] | `perception_node.py` |

## 3. so101_brain (container control)

| Tâche | Statut | Fichier(s) |
|---|---|---|
| Souscrit `/joint_states` (radians) + `/ball_position_3d` | [Y] | `brain_node.py` |
| **Publie `/joint_command`** (conforme DESIGN §7 : toutes les commandes moteur y passent) | [Y] | `brain_node.py` `_send_action()` |
| IK ikpy, URDF résolu via package `so101_sim` installé (fallback chemin repo) | [Y] | `brain_node.py` `_find_urdf()` |
| `active_links_mask=[False,True×5,False]` (7 maillons, vérifié) | [Y] | `brain_node.py` |
| Seed IK = configuration courante (radians, sans double conversion) | [Y] | `brain_node.py` `_seed()` |
| Rejet cible si erreur FK > `ik_tolerance` (2 cm) + retour idle | [Y] | `brain_node.py` `_solve_ik()` / `_ik_move()` |
| Services `brain/go_to_target` (GoToTarget), `brain/start|stop_autonomous` (SetBool) — types réels importés, plus de chaînes passées à create_service | [Y] | `brain_node.py` |
| Machine à états pick & place (home→approach→down→grasp→lift→transport→place→home) | [Y] | `brain_node.py` `_autonomous_loop()` |
| Tuning préhension (orientation pince, offsets, PICK_APPROACH_Z vs limite workspace r≈0.27 m) | [N] | à régler en visu — l'infra est là, la saisie éjecte encore la boule |

## 4. Intégration & Build

| Tâche | Statut | Fichier(s) |
|---|---|---|
| `colcon build` (interfaces → driver/brain/perception/description) | [?] | à valider dans le container — non testable hors ROS ; anciens build/install/log purgés |
| docker-compose : control build TOUT puis lance driver+brain ; perception/viz attendent `install/` (pas de colcon concurrents sur le volume) | [Y] | `docker/docker-compose.yml` |
| `control-real` : `use_sim:=false` via override de commande | [Y] | `docker/docker-compose.yml` |
| RViz : Fixed Frame `base_link`, RobotModel, TF — fonctionne car /joint_states désormais en radians | [?] | à vérifier visuellement |

## 5. Jalons

| Jalon | Statut | Notes |
|---|---|---|
| **S2** : `ros2 topic pub /joint_command` → le bras bouge (sim + RViz) | [Y*] | logique validée hors-ROS (30° → 0.524 rad publié) ; smoke test container à faire |
| **S4** : IK < 2 cm sur cible XYZ, pince commandable | [Y*] | erreur FK 0 mm au grasp ; approche haute (z=0.10) proche limite workspace |
| **S7** : pick & place complet, 3 essais | [N] | pipeline câblé de bout en bout ; préhension à tuner |
| Bonus réel `use_sim:=false` | [N] | chemin de code présent, non testé sur hardware |

## Légende
[Y] fait · [Y*] fait, smoke test container restant · [N] pas fait · [?] non vérifiable hors container

AFTER READING THIS FILE, if you perform any command, you have to run it inside the docker.
ALSO, YOU HAVE TO READ the README.md, DESIGN.md, and INSTRUCTION.md
