# README_IK — Cinématique inverse du SO-ARM101 avec ikpy

> Document instructeur/étudiant avancé. Explique ce qu'est l'IK, comment
> l'implémenter avec `ikpy` sur ce bras, et les pièges vérifiés sur ce repo.

## 1. FK / IK en 30 secondes

- **Cinématique directe (FK, forward kinematics)** : angles des joints → pose
  (position + orientation) de l'organe terminal (TCP, *tool center point* =
  pointe de la pince). Calcul déterministe, une seule réponse : on multiplie
  les transformations homogènes de chaque segment.
- **Cinématique inverse (IK, inverse kinematics)** : pose cible du TCP →
  angles des joints. Problème inverse, en général **pas de solution unique**
  (plusieurs configurations atteignent le même point), parfois **aucune**
  (cible hors de l'espace de travail).

`ikpy` résout l'IK **numériquement** : optimisation (moindres carrés) qui part
d'une configuration initiale et minimise l'erreur entre la pose FK courante et
la cible. Conséquences pratiques : la solution dépend du point de départ, et
l'algorithme peut converger vers un minimum local (voir §5).

## 2. Le cas SO-ARM101 : 5 DOF, pas 6

Le bras a **5 joints** (`shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`) + la pince (qui n'est pas un DOF cinématique : elle
ne déplace pas le TCP). Une pose complète dans l'espace = 6 DOF (3 position +
3 orientation). **Avec 5 DOF, on ne peut pas imposer position ET orientation
arbitraires simultanément.**

Ce que ça implique concrètement :
- IK **en position seule** (x, y, z) : fonctionne partout dans l'espace de
  travail. C'est le mode à utiliser par défaut.
- IK position + orientation complète : échouera ou donnera un compromis
  dégradé pour la plupart des cibles. Ne pas s'acharner : c'est une limite
  physique du bras, pas un bug.
- Cas utile intermédiaire : contraindre **un seul axe** d'orientation (ex.
  pince pointant vers le bas pour saisir par le dessus). ikpy le permet via
  `orientation_mode` (voir §4). C'est généralement suffisant pour le pick &
  place.

## 3. L'URDF : lequel, et pourquoi celui-là

Fichier fourni : `sim/assets/so101/so101_new_calib.urdf` — c'est l'URDF
**officiel** du repo TheRobotStudio/SO-ARM100 (variante SO-101, calibration
"new" alignée sur la convention LeRobot récente). Points vérifiés :

- Les **noms de joints correspondent exactement** au driver LeRobot et au
  modèle MuJoCo : `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
  `wrist_roll`, `gripper`. Aucun mapping de noms à faire.
- La chaîne se termine par un joint **fixe** `gripper_frame_joint` →
  `gripper_frame_link` : c'est le **TCP** (pointe de la pince). ikpy suit
  naturellement cette branche et ignore la branche mobile de la mâchoire.
  Donc la position que calcule/atteint l'IK est bien **le point de saisie**,
  pas le poignet (~10 cm d'écart, vérifié).
- **Aucune modification de l'URDF n'est nécessaire** pour ikpy. (La question
  s'était posée : la réponse, après test, est non. Les warnings au parsing
  sont bénins, voir §5.)

Attention si vous récupérez un autre URDF (export Fusion 360, vieux repos) :
noms de joints différents, TCP absent, axes inversés — restez sur celui fourni.

## 4. Implémentation ikpy — le strict nécessaire

```python
import numpy as np
import ikpy.chain

chain = ikpy.chain.Chain.from_urdf_file(
    "sim/assets/so101/so101_new_calib.urdf",
    # 7 links : [Base fixe, 5 joints actifs, gripper_frame fixe]
    active_links_mask=[False, True, True, True, True, True, False],
)

# --- FK : angles (RADIANS) -> pose 4x4 du TCP
q = [0, *np.radians([15, -30, 40, -20, 10]), 0]   # padding des links fixes !
T = chain.forward_kinematics(q)
position_tcp = T[:3, 3]

# --- IK position seule
q_sol = chain.inverse_kinematics(
    target_position=[0.25, 0.05, 0.10],
    initial_position=q_courant,        # TOUJOURS seeder avec la config actuelle
)
angles_deg = np.degrees(q_sol[1:6])    # -> consignes driver (degrés)

# --- IK avec contrainte "pince vers le bas" (saisie par le dessus)
q_sol = chain.inverse_kinematics(
    target_position=[0.25, 0.05, 0.03],
    target_orientation=[0, 0, -1],     # axe Z du TCP pointant vers -Z monde
    orientation_mode="Z",
    initial_position=q_courant,
)
```

## 5. Pièges vérifiés et leurs solutions

| # | Piège | Symptôme | Solution |
|---|-------|----------|----------|
| 1 | **`active_links_mask` absent** | Warnings, l'optimiseur "bouge" des links fixes, solutions incohérentes | Toujours passer `[False, True×5, False]` (vérifié : la chaîne parsée fait exactement 7 links dans cet ordre) |
| 2 | **Unités** | Bras qui part en butée | ikpy travaille en **radians**, le driver LeRobot en **degrés**. Convertir aux deux frontières, nulle part ailleurs |
| 3 | **Vecteur d'angles de taille 7** | `ValueError` ou décalage d'un joint | FK/IK attendent un vecteur couvrant TOUS les links, y compris les fixes : `[0, q1..q5, 0]` |
| 4 | **Pas de seed (`initial_position`)** | Le bras fait des grands mouvements erratiques entre deux cibles proches, ou "flip" de coude | Seeder l'IK avec la configuration articulaire **courante** (lue sur `/joint_states`). L'optimiseur converge alors vers la solution la plus proche → trajectoires continues |
| 5 | **Cible hors espace de travail** | Solution avec grosse erreur résiduelle, sans exception | ikpy ne lève **pas** d'erreur : il retourne le meilleur compromis. **Toujours vérifier** : `np.linalg.norm(chain.forward_kinematics(q_sol)[:3,3] - cible) < seuil` (ex. 5 mm) avant d'envoyer la consigne |
| 6 | **Imposer une orientation 6D complète** | Erreur résiduelle énorme | Bras 5 DOF (§2) : position seule, ou `orientation_mode="Z"` avec un seul axe |
| 7 | **Warnings au parsing URDF** (`gripper_frame_joint ... axis attribute`) | Inquiétude injustifiée | Bénin : l'URDF officiel met un `axis` sur un joint fixe, ikpy l'ignore proprement. Ne rien changer |
| 8 | **Limites articulaires** | IK propose des angles que les servos refusent | ikpy lit les limites de l'URDF et les respecte, mais après conversion/calibration réelle, re-clamper côté driver ne coûte rien (`max_relative_target` du driver LeRobot fait déjà un clamp de sécurité en vitesse) |

## 6. Aller plus loin (mentionné en cours, hors scope projet)

- **MoveIt2** : le standard industriel ROS2 (planification avec évitement de
  collisions, contraintes, temps réel). Ici on écrit l'IK "à la main" pour
  comprendre ce que MoveIt2 automatise.
- **Solveurs QP** (ex. `placo`, utilisé par LeRobot upstream) : formulation en
  optimisation quadratique avec tâches et contraintes — plus robuste qu'ikpy,
  mais boîte noire pédagogiquement. C'est volontairement retiré de ce repo.
- Pour les curieux : IK analytique (5 DOF s'y prête), jacobienne + moindres
  carrés amortis (DLS) — c'est ce qu'ikpy fait sous le capot.
