# Design — SO-ARM101 ROS 2 : Perception → IK → Pick & Place

## 1. Objectif

Contrôler un bras **SO-ARM101** (6 axes : 5 articulations + gripper) afin de réaliser un **pick & place** d'une boule dorée.

Pipeline :

```text
Caméra → Perception → IK → Commande moteur
```

Le projet doit fonctionner avec la **même API** en simulation et sur le robot réel. Le choix du backend est réalisé via le paramètre ROS 2 :

```text
use_sim
```

---

## 2. Backends

Les deux implémentations exposent exactement la même interface.

| Backend | Emplacement | Connexion | Observations | Actions | Package |
|---------|-------------|-----------|--------------|---------|---------|
| `SO101Follower` (réel) | `core/lerobot_min/` | Série (Feetech STS3215) | Joints + caméra | Consignes en degrés | `so101-core` |
| `SO101Sim` (MuJoCo) | `sim/so101_sim/` | MuJoCo + OSMesa | Joints + caméra | Consignes en degrés | `so101-sim` |

API commune :

```python
connect()
get_observation()
send_action(action)
disconnect()
```

---

## 3. Architecture ROS 2

```text
        /external_cam/image_raw
        /external_cam/camera_info          /ball_position_3d
┌──────────┐ ────────────────► ┌────────────────┐ ────────► ┌────────────────┐
│ driver   │                   │ perception     │           │ brain          │
│          │                   │ (HSV + 3D)     │           │ (IK ikpy)      │
└──────────┘                   └────────────────┘           └────────────────┘
     ▲                                                              |
     |                          /joint_command                      |
     └──────────────────────────────────────────────────────────────┘
                                                      |
                                                      v
                                          ┌──────────────────────────┐
                                          │ driver                   │
                                          │                          │
                                          │ send_action()            │
                                          │            │             │
                                          │            ▼             │
                                          │     SO101Sim /           │
                                          │   SO101Follower          │
                                          │            │             │
                                          │     Exécution            │
                                          │ (MuJoCo ou robot réel)   │
                                          │            │             │
                                          │            ▼             │
                                          │ get_observation()        │
                                          │            │             │
                                          │            ▼             │
                                          │ Publication ROS          │
                                          │ /joint_states            │
                                          └──────────────────────────┘
```

### driver

Le **driver** est le seul nœud qui communique avec le backend (simulation Mujoco ou robot réel SO101Follower).

À chaque cycle :

1. reçoit une commande sur `/joint_command`
2. appelle `send_action()`
3. le backend exécute la commande :
   - **simulation** : MuJoCo avance la physique ;
   - **réel** : les servomoteurs exécutent le mouvement ;
4. appelle `get_observation()`
5. construit un message `sensor_msgs/JointState`
6. publie `/joint_states` (≥ 20 Hz)

Le backend (**MuJoCo** ou **SO101Follower**) **ne publie jamais directement de topics ROS**. Il fournit uniquement une API (`send_action()` / `get_observation()`) utilisée par le driver.

En sim, le driver publie aussi `/external_cam/image_raw` + `/external_cam/camera_info`
(10 Hz) et `/drop_box_position` (PoseStamped, 10 Hz), et expose la calibration
caméra via les paramètres ROS 2 `cam_K` (3×3) et `cam_T` (4×4, caméra→monde).

### perception

- Souscrit à `/external_cam/image_raw` (+ `camera_info` / paramètres `cam_K`,
  `cam_T` du driver pour la calibration).
- Détection HSV de la boule dorée.
- Projection pixel → position 3D (K, T, intersection au plan z = rayon boule,
  convention caméra MuJoCo : regard selon **−Z**).
- Publie `/ball_position_3d` (geometry_msgs/PoseStamped, repère `world`).
- Fréquence cible : **5 à 10 Hz**.

### brain

- Souscrit à `/joint_states` (seed IK), `/ball_position_3d` et
  `/drop_box_position`.
- Résolution de l'IK (`ikpy`, orientation pince vers le bas −Z, fallback
  orientation libre si la contrainte sort la cible du workspace).
- Boucle pick & place autonome (machine à états publiée sur `/brain_state`).
- Publie `/joint_command`.
- Services : `/brain/go_to` (so101_interfaces/GoToTarget),
  `/brain/start`, `/brain/stop` (std_srvs/Trigger).

### interfaces

Les types de services custom sont dans le package **`so101_interfaces`**
(ament_cmake + rosidl) : `GoToTarget.srv`, `SetJointPositions.srv`
(utilisé par `/driver/set_joints`, réservé aux tests manuels).

---

## 4. Conventions

| Élément | Convention |
|---------|------------|
| `/joint_command` | Degrés absolus (`0°` = bras vertical), gripper `0–100 %` |
| `/joint_states` | **Radians** (REP-103 — requis par `robot_state_publisher`/RViz), gripper converti % → rad via limites URDF |
| Backend (`send_action`/`get_observation`) | Degrés, gripper `0–100 %` |
| Driver | ≥ 20 Hz |
| Perception | 5–10 Hz |
| Boucle de contrôle driver | 50 Hz (ré-application de la dernière consigne) |
| MuJoCo | La caméra regarde selon l'axe **−Z** (conversion pixel → rayon à adapter) |

---

## 5. Docker

| Service | Rôle |
|---------|------|
| `control` | ROS 2 + backend + simulation + IK (`ikpy`) + RViz |
| `perception` | OpenCV + détection HSV |
| `viz` | `robot_state_publisher` |
| `control-real` | Driver matériel (profil Compose `real`) |

### Configuration

- Communication ROS 2 via le réseau Docker Compose (DDS).
- Affichage RViz via X11 :

```bash
xhost +local:docker
```

Variables d'environnement (simulation Docker) :

```bash
MUJOCO_GL=osmesa
PYOPENGL_PLATFORM=osmesa
```

---

## 6. Jalons

| Jalon | Objectif |
|--------|----------|
| S2 | Le bras réagit aux messages `/joint_command` et publie `/joint_states` |
| S4 | IK fonctionnelle avec une erreur inférieure à **1 cm** |
| S7 | Pick & place complet sur une boule positionnée aléatoirement (3 essais) |
| Bonus | Exécution sur le robot réel avec `use_sim:=false` |

---

## 7. Contraintes

- `get_ball_position()` est **interdite** dans le pipeline (autorisée uniquement pour les tests unitaires).
- Toutes les commandes moteurs passent exclusivement par `/joint_command`.
- Versions imposées :
  - `numpy < 2`
  - `opencv == 4.10`
- Les variables suivantes sont requises **uniquement** dans Docker :

```bash
MUJOCO_GL=osmesa
PYOPENGL_PLATFORM=osmesa
```
