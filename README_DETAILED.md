# README_DETAILED — tek5_robotics : état complet du projet

Document instructeur. Synthèse de toutes les décisions, de ce qui est
construit et validé, de ce qui reste à faire, et des commandes de test.
Complète `INSTRUCTION.md` (visu/lancement) et `docs/SUJET.md` (côté étudiant).

---

## 1. Objectif et périmètre

Projet Tek5 (5ᵉ année PGE, dev SW généralistes sans background robotique) :
contrôler **un bras SO-ARM101 follower** — perception caméra, IK, pick &
place d'une boule dorée. Dérivé du fork LeRobot (~990 fichiers) réduit à
l'essentiel : **tout le ML/CUDA/datasets/policies a été retiré**. Extraction
chirurgicale : ~30 fichiers copiés avec closure d'imports, 3 fichiers purgés
(torch, HF Hub) annotés.

## 2. Décisions de design (actées)

| Décision | Choix | Justification courte |
|---|---|---|
| Middleware | **ROS2 Jazzy** (couche écrite par les étudiants) | Valeur CV, séparation contrôle/perception. Les STS3215 sont des smart servos (PID onboard) : l'hôte streame des consignes position à 50-100 Hz max, pas de contrôle haute fréquence à perdre. `ros2_control`/MoveIt2 : mentionnés en cours, pas implémentés. |
| Simulateur | **MuJoCo** (pas Gazebo) | Modèle SO-101 officiel TheRobotStudio (URDF+MJCF, joints alignés sur le driver LeRobot), rendu offscreen trivial en Docker (OSMesa). Rien de Gazebo n'était réutilisable. |
| Architecture | **Jumeau numérique** : `SO101Sim` expose la même API que `SO101Follower` (`connect / get_observation / send_action / disconnect`) | Un seul `driver_node` étudiant, paramètre ROS2 `use_sim`. Passage au réel = changer un paramètre. |
| IK | **ikpy, écrite par les étudiants** | `model/kinematics.py` (solution clé en main) exclu du repo. URDF validé : frame terminale = TCP, aucune modif. |
| Perception | **Docker séparé**, bridge = topics DDS sur réseau compose | Pattern industriel. Bonus bras réel : Linux only (passthrough `/dev`). |
| Visualisation | **RViz depuis le container control** (X11 forwardé) | RViz est un nœud ROS2 natif : mêmes DDS/QoS/tf2 que le pipeline, zéro couche de traduction. Prérequis module : hôte Linux (acté). Voir §5. |

## 3. Arborescence et rôle

```
core/lerobot_min/     driver Feetech + SO101Follower + caméras OpenCV (pip: so101-core)
sim/                  SO101Sim + scène MJCF (pip: so101-sim)
  scene_tek5.xml      boule dorée randomisable (settling physique), boîte de dépôt,
                      caméra externe fixe (K/T exposés = "TF fournie")
  so101_tek5.xml      bras recoloré GRIS (les meshes officiels sont jaunes →
                      conflit HSV avec la boule or)
docker/               Dockerfile.control (ROS2+core+sim+ikpy+viz), Dockerfile.perception,
                      compose (control, perception, viz, profil real)
ros2_ws/src/
  example_py_pkg      talker d'exemple (test DDS)
  so101_description   URDF réécrit en package:// + meshes + launch viz
                      (robot_state_publisher, arg demo:=) — package FOURNI
scripts/
  demo_sim.py         smoke test : rend une image caméra
  view_live.py        viewer MuJoCo natif + trajectoire démo (local only)
  jog.py              test moteur unifié sim/réel (voir §6)
  calibrate_real.py   calibration du bras physique (procédure LeRobot)
docs/SUJET.md         contrat étudiant : 3 nœuds (driver/perception/brain), topics,
                      fréquences (perception 5-10 Hz, driver ≥20 Hz), jalons S2/S4/S7,
                      barème, section visu
INSTRUCTION.md        lancement + visu, local vs Docker
```

Interdits/garde-fous pédagogiques : `get_ball_position()` = vérité terrain,
interdite dans le pipeline (autorisée en tests unitaires) ; consignes moteur
uniquement via `/joint_command`.

## 4. Ce qui est validé

- **Pipeline bout-en-bout exécuté** (solution de référence, non distribuée) :
  HSV → pixel→3D (K, T, plan sol) → ikpy → consignes. Perception **1,5 mm**
  de précision moyenne (5 seeds), IK roundtrip 0 mm, TCP atteint à 0,1 mm.
  → le projet est résoluble tel que spécifié.
- **Chaîne viz validée en conditions réelles** : `robot_state_publisher` du
  service `viz` publie `/robot_description` + TF complet
  (base_link → … → moving_jaw) vérifiés par `ros2 topic echo /tf` et lecture
  dans RViz.
- **jog.py testé en sim** : status / consigne joint / sweep / zero,
  consigne = lecture à 0,1° près.
- **Fix numpy/opencv appliqué** : `ros:jazzy` embarque numpy 1.26 par apt
  (pas de RECORD pip) ; opencv 5.x exigerait numpy≥2 → crash upgrade, et
  numpy 2 casserait cv_bridge. Invariant de l'image : **numpy reste <2**,
  épingles `"numpy<2" "opencv-python-headless==4.10.*"` dans les deux
  Dockerfiles. Étudiants YOLO : contraintes dans la même commande pip
  (avertissement dans le sujet, commande volontairement non fournie ;
  alternative valorisée : export ONNX + onnxruntime).
- **Fix rendu offscreen** : `PYOPENGL_PLATFORM=osmesa` requis en plus de
  `MUJOCO_GL=osmesa` (PyOpenGL récents) — dans le Dockerfile. Règle : ces
  variables sont **Docker only**, en local aucune variable (GLFW/EGL natif).

## 5. Visualisation — décision et historique

**Chemin officiel : RViz, exécuté dans le container control, X11 forwardé**
(`xhost +local:docker` + DISPLAY + /tmp/.X11-unix montés — voir compose).
RViz tourne sur le réseau DDS interne : plus besoin de --network host ni de
gérer le multicast vers l'hôte. Prérequis module : hôte Linux (natif ou VM),
acté — cohérent avec le bonus bras réel (passthrough /dev) et l'outillage
industrie.

Pièges connus :
- Sans accélération GPU dans le container (NVIDIA sans container-toolkit,
  VM sans 3D) : `LIBGL_ALWAYS_SOFTWARE=1 rviz2` (llvmpipe, largement
  suffisant pour un bras 6 DoF à 10 Hz).
- Wayland : XWayland gère le forwarding ; QT_QPA_PLATFORM=xcb est dans le
  compose.
- Sans /joint_states, bras en morceaux + erreurs RobotModel = comportement
  attendu, transformé en livrable S2. Smoke test : launch demo:=true.
- Le RobotModel lit l'URDF → bras jaune (le recolorage gris n'existe que
  dans le MJCF sim). Sans conséquence pipeline.

**Foxglove : abandonné (08/07/2026)** après une journée de debug. Symptôme :
en connexion live via `foxglove_bridge`, le RobotModel s'affichait "en
morceaux" (sous-groupes à joints fixes assemblés mais mal orientés entre
eux), alors que le graphe ROS était prouvé sain (`/tf` à 10 Hz, `view_frames`
complet, `tf2_echo` correct). Diagnostic tranché par un test MCAP : les mêmes
TF, rejouées dans le même client Foxglove depuis un fichier enregistré,
s'affichent correctement assemblées. Conclusion : les données et
l'interprétation TF du client sont saines, le bug est dans le chemin **live**
foxglove_bridge (paquet apt, série 3.x réécrite sur le Foxglove SDK) ↔ client
2026 — probablement protocole ou cache de transforms côté session
websocket, non identifié précisément (non nécessaire, cf. décision
ci-dessous). Autres frictions notées en cours de route, désormais sans
objet : Firefox web cassait le rendu TF (Chrome/app desktop uniquement),
échelle des axes TF par défaut illisible sur un bras de 30 cm, résolution
`package://` des meshes uniquement via websocket (pas en lecture fichier
locale). Arbitrage : une dépendance SaaS + bridge hors de notre contrôle,
susceptible de se recasser à chaque mise à jour du client chez 15 campus,
n'a pas sa place sur le chemin critique du module. Foxglove n'est plus
mentionné aux étudiants ; RViz est le seul chemin documenté et supporté.

**Viewer MuJoCo natif** (`view_live.py`) : local, instructeur, inchangé.

## 6. Tester les moteurs — sim vs réel

Même script, même API, mêmes unités : `scripts/jog.py`.

### Joints, unités et plages

Consignes en **degrés absolus** (0 = pose de référence, bras tendu vertical),
sauf le gripper en **pourcentage d'ouverture**. Butées lues dans le modèle
officiel — au-delà, le joint sature à la butée (la lecture ne rejoindra pas
la consigne, c'est normal, pas un bug) :

| Joint           | Rôle                              | Plage        | Zone de test conseillée |
|-----------------|-----------------------------------|--------------|-------------------------|
| `shoulder_pan`  | rotation de la base (axe vertical)| ±110°        | ±45°                    |
| `shoulder_lift` | épaule (lève/baisse le bras)      | ±100°        | −30° à +30°             |
| `elbow_flex`    | coude                             | ±97°         | ±45°                    |
| `wrist_flex`    | poignet (tangage)                 | ±95°         | ±45°                    |
| `wrist_roll`    | rotation de la pince              | −157°/+163°  | ±90°                    |
| `gripper`       | ouverture mâchoire                | **0-100 %**  | 20-80                   |

Signes : `shoulder_pan` positif = sens trigo vu de dessus ; `shoulder_lift`
et `elbow_flex` négatifs penchent le bras vers l'avant de la scène. En cas
de doute sur un signe : petite consigne (±15°) + regarder la caméra/RViz.

**Sur le bras réel uniquement** : les butées logicielles n'empêchent pas les
collisions bras-table — `shoulder_lift`/`elbow_flex` à forte amplitude
peuvent planter la pince dans le support. Rester dans la zone de test
conseillée tant que la géométrie n'est pas maîtrisée.

### En simulation (local, venv uv ; ou dans le container control)

```bash
python3 scripts/jog.py status              # positions courantes
python3 scripts/jog.py shoulder_pan 30     # consigne un joint, vérifie la lecture
python3 scripts/jog.py sweep elbow_flex    # aller-retour −30/+30/0
python3 scripts/jog.py sweep gripper       # 20/80/50
python3 scripts/jog.py zero                # tout à zéro, gripper 50
```

### Sur le bras réel (hôte Linux, bras sur USB)

```bash
# 0. installer le core en local (une fois) :
uv pip install -e core/lerobot_min

# 1. le bus répond ? scan des IDs Feetech (attendu : 6 moteurs, IDs 1-6)
python3 scripts/jog.py --real --port /dev/ttyACM0 scan

# 2. première utilisation (ou après démontage) : calibration interactive
python3 scripts/calibrate_real.py /dev/ttyACM0 tek5_arm
#    → stockée dans ~/.cache/tek5_robotics/calibration/

# 3. mêmes commandes qu'en sim, avec --real :
python3 scripts/jog.py --real status
python3 scripts/jog.py --real shoulder_pan 20
python3 scripts/jog.py --real sweep wrist_roll
python3 scripts/jog.py --real zero
```

Conseils réel : commencer par des consignes **faibles** (±20°), zone
dégagée, gripper d'abord (aucun risque de collision). `consigne ≈ lue` à
1-2° près = servo OK ; écart constant = calibration à refaire ; pas de
réponse au scan = câblage/alim/ID.

### Via ROS2 (une fois le driver_node étudiant écrit)

```bash
ros2 topic pub --once /joint_command <type_choisi> "{...shoulder_pan: 30...}"
ros2 topic echo /joint_states
```

C'est le test du jalon S2 — identique en sim et en réel par construction.

## 7. Nettoyage repo (constaté dans le zip du 07/07)

- [ ] **Supprimer `Dockerfile.control` à la racine** — doublon obsolète
      (sans les paquets viz) de `docker/Dockerfile.control`.
- [ ] **Gitignorer les artefacts colcon** : `ros2_ws/build/`,
      `ros2_ws/install/`, `ros2_ws/log/` sont commités.
- [ ] `__pycache__/` au passage.
- [ ] Vider `~/.local/share/Trash` des anciennes copies du repo
      (`lerobot_tek5_epitech`, `tek5_viz_delta*`) — source de confusion
      constatée lors du debug Foxglove (containers pilotés en croisé).

## 8. Reste à faire

**Technique**
- [ ] Build Docker complet validé de bout en bout côté instructeur, y compris
      `viz` (fait partiellement : viz fonctionne, rebuild propre à confirmer
      après nettoyage §7).
- [ ] Test DDS inter-containers (`talker` → `ros2 topic echo`) — prérequis
      absolu, à faire avant kickoff.
- [ ] Bras physique : calibration + `jog.py --real` + `use_sim:=false`
      validés sur le hardware réel (bloqué par la réception des bras).
- [ ] Valider RViz-in-Docker sur 3 configs : laptop NVIDIA/X11 (attention
      GLX container : nvidia-container-toolkit ou LIBGL_ALWAYS_SOFTWARE=1),
      Ubuntu 24.04 Wayland vanilla, VM (3D on/off).
- [ ] Optionnel : livrer un so101.rviz préconfiguré (RobotModel + TF +
      Image) et un alias `rviz2 -d` dans la doc — épargne la config manuelle
      aux 15 campus.
- [ ] Optionnel : web_video_server (port 8080, MJPEG navigateur) comme
      commodité caméra sans RViz — apt + port compose si retenu.

**Documentation**
- [ ] SUJET.md : ajouter l'avertissement numpy système apt (piège YOLO).
- [ ] SUJET.md : réécrire la section visu sur RViz (RobotModel, TF, Image),
      supprimer toute référence Foxglove.
- [ ] Garder l'indice visio #2 pour S5 : convention caméra MuJoCo (regarde
      vers **-Z**, flip d'axes pixel→rayon) — mur prévisible.

**Logistique (contexte inchangé)**
- [ ] Sourcing 15 bras Alibaba, devis réel, commande test 8-10 moteurs.
- [ ] Contact HF après finalisation du plan.
- [ ] Commit du reboot dans le fork (`git rm -r .` → coller → commit).

## 9. Jalons étudiants (rappel)

- **S2** : talker inter-containers OK, `driver_node` publie `/joint_states`,
  bras bouge sur `ros2 topic pub /joint_command`. (= le bras "s'assemble"
  dans RViz.)
- **S4** : IK fonctionnelle, erreur < 1 cm, pince commandable.
- **S7** : pick & place complet, boule randomisée, 3 essais consécutifs.
- **Bonus** : même code, `use_sim:=false`, bras physique.
