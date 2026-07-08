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
                         /camera/image
┌────────────────┐ -------------------------> ┌────────────────┐
│ perception     │                            │ brain          │
│ (Docker)       │                            │                │
└────────────────┘                            └────────────────┘
                                                      |
                                                      | /joint_command
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

Le **driver** est le seul nœud qui communique avec le backend (simulation ou robot réel).

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

### perception

- Capture les images de la caméra.
- Détecte la boule (HSV).
- Publie `/camera/image`.
- Fréquence : **5 à 10 Hz**.

### brain

- Souscrit à `/camera/image`.
- Détection HSV.
- Projection pixel → position 3D (K, T, plan du sol).
- Résolution de l'IK (`ikpy`).
- Publie `/joint_command`.

---

## 4. Conventions

| Élément | Convention |
|---------|------------|
| Angles | Degrés absolus (`0°` = bras vertical) |
| Gripper | Pourcentage d'ouverture (`0–100 %`) |
| Driver | ≥ 20 Hz |
| Perception | 5–10 Hz |
| Commandes | 50–100 Hz |
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
