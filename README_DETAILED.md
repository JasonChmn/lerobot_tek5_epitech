# README_DETAILED — tek5_robotics : état complet du projet

Document instructeur. Synthèse de toutes les décisions, de ce qui est
construit et validé, de ce qui reste à faire, et des commandes de test.
Complète `INSTRUCTION.md` (runbook : commandes de test copy-paste) et
`docs/SUJET.md` (côté étudiant).

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
  demo_pick_place.py  démo instructeur : pick & place complet standalone
                      (SO101Sim + ikpy, vérité terrain autorisée hors pipeline)
  view_live.py        viewer MuJoCo natif + trajectoire démo (local only)
  decimate_meshes.py  décimation des STL (160K→12K verts, perf OSMesa) — one-shot,
                      déjà appliqué aux meshes du repo
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
- **Fix IK transport (09/07)** : avec contrainte d'orientation −Z stricte,
  ikpy ne peut pas atteindre la drop_box à z=0.10 (erreur FK 31 mm > tol 2 cm)
  → la boucle autonome avortait au transport à chaque cycle. Corrigé :
  `PLACE_APPROACH_Z=0.08` (15 mm, dans la tolérance) + fallback orientation
  libre dans `_solve_ik` quand la solution contrainte sort de la tolérance.
  Le brain lit désormais la position de dépôt sur `/drop_box_position`
  (publié par le driver) au lieu d'une constante, et exige une détection
  boule fraîche à chaque cycle (pose remise à None avant `_wait_ball`).
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

**Viewer MuJoCo natif** (`view_live.py`, `mujoco.viewer`) : outil instructeur
LOCAL uniquement — voir §6, sous-section « Outils instructeur (local) ».

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

### En simulation (dans le container control)

```bash
docker compose exec control bash -lc "python3 /opt/so101/scripts/jog.py status"
docker compose exec control bash -lc "python3 /opt/so101/scripts/jog.py shoulder_pan 30"
docker compose exec control bash -lc "python3 /opt/so101/scripts/jog.py sweep elbow_flex"
docker compose exec control bash -lc "python3 /opt/so101/scripts/jog.py sweep gripper"
docker compose exec control bash -lc "python3 /opt/so101/scripts/jog.py zero"
```

### Sur le bras réel (container control-real, bras sur USB)

```bash
cd docker
docker compose --profile real up -d control-real   # passthrough /dev/ttyACM0

# 1. le bus répond ? scan des IDs Feetech (attendu : 6 moteurs, IDs 1-6)
docker compose exec control-real bash -lc \
  "python3 /opt/so101/scripts/jog.py --real --port /dev/ttyACM0 scan"

# 2. première utilisation (ou après démontage) : calibration interactive
docker compose exec control-real bash -lc \
  "python3 /opt/so101/scripts/calibrate_real.py /dev/ttyACM0 tek5_arm"

# 3. mêmes commandes qu'en sim, avec --real :
docker compose exec control-real bash -lc "python3 /opt/so101/scripts/jog.py --real status"
docker compose exec control-real bash -lc "python3 /opt/so101/scripts/jog.py --real shoulder_pan 20"
docker compose exec control-real bash -lc "python3 /opt/so101/scripts/jog.py --real sweep wrist_roll"
```

Conseils réel : commencer par des consignes **faibles** (±20°), zone
dégagée, gripper d'abord (aucun risque de collision). `consigne ≈ lue` à
1-2° près = servo OK ; écart constant = calibration à refaire ; pas de
réponse au scan = câblage/alim/ID.

### Outils instructeur — LOCAL uniquement (hors Docker)

`view_live.py` et `mujoco.viewer` ouvrent une **fenêtre MuJoCo interactive**
(GLFW/EGL, GPU de l'hôte). Ils ne tournent **pas** en Docker (le container est
en OSMesa offscreen, sans fenêtre) et ne sont **pas** sur le chemin étudiant.

Point important : ils instancient une **sim séparée** du pipeline Docker — ce
n'est PAS la même simulation. On ne peut donc PAS y regarder la démo pick &
place (qui vit dans le container). Usage réel : inspecter le *modèle*
(géométrie, joints, butées, perturbations manuelles) hors pipeline. Pour voir
la démo, c'est RViz (§10.2).

```bash
# setup local une fois :
uv venv ~/.venvs/tek5 && source ~/.venvs/tek5/bin/activate
uv pip install mujoco opencv-python

python3 scripts/view_live.py --seconds 60     # bras animé (trajectoire démo interne)
python3 scripts/view_live.py --cam            # + fenêtre OpenCV caméra externe
python3 -m mujoco.viewer --mjcf sim/so101_sim/assets/so101/scene_tek5.xml  # modèle statique
```

En local, **aucune** variable d'environnement (MuJoCo prend le GPU). Ne jamais
forcer `MUJOCO_GL=osmesa` en local → crash `'NoneType' ... glGetError`.

### Via ROS2 (une fois le driver_node étudiant écrit)

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan], position: [30.0]}"
ros2 topic echo /joint_states   # positions en radians (convention RViz)
```

C'est le test du jalon S2 — identique en sim et en réel par construction.

## 7. Nettoyage repo

- [x] `Dockerfile.control` racine supprimé (doublon de `docker/`).
- [x] Artefacts colcon purgés (`ros2_ws/build|install|log`, `__pycache__`,
      `core/lerobot_min/build`) — reste à les gitignorer.
- [x] Doublons srv supprimés (`so101_brain/srv/`, `so101_driver/srv/` —
      les définitions vivent dans `so101_interfaces`).
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

## 10. Annexe — environnement, RViz pas-à-pas, dépannage

(Contenu de référence extrait d'INSTRUCTION.md, devenu runbook pur.)

### 10.1 Deux environnements, deux stacks de rendu

|                    | **Local (hôte)**                    | **Docker (containers)**            |
|--------------------|-------------------------------------|-------------------------------------|
| Rendu MuJoCo       | GLFW/EGL natif (driver GPU)         | `MUJOCO_GL=osmesa` (offscreen soft) |
| Fenêtre interactive| `view_live.py`, `mujoco.viewer`     | RViz via X11 forwardé               |
| Variables env      | **aucune**                          | déjà dans le Dockerfile             |
| Python             | venv `uv`                           | dans l'image                        |
| Usage              | instructeur SEULEMENT : debug modèle (§6) | tout le reste : pipeline, démo, tests |

Règle : `MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa` c'est **Docker
uniquement**. En local, ne rien mettre — MuJoCo prend le GPU. Forcer osmesa
en local sans `libosmesa6` : crash cryptique `'NoneType' ... glGetError`.
Tester le chemin de rendu Docker en local (rarement utile) :
`sudo apt install libosmesa6` puis préfixer `demo_sim.py` avec les deux
variables.

Terminaux : `docker compose exec <service> bash` ouvre un nouveau shell dans
un container déjà lancé — en ouvrir autant que nécessaire. 1 terminal =
1 shell = soit hôte, soit container.

### 10.2 RViz — configuration pas-à-pas et dépannage

Une fois (puis **File → Save Config** dans le workspace) :
1. **Global Options → Fixed Frame : `base_link`**
2. **Add → RobotModel** → Description Topic : `/robot_description`
3. **Add → TF** (facultatif : Marker Scale ~0.2, sinon illisible sur un bras
   de 30 cm)
4. **Add → Image** → Topic : `/external_cam/image_raw` (driver actif)
5. La boule : **Add → Marker** sur le topic publié par la perception
   étudiante — la visu montre ce que le code voit, pas la vérité terrain.

Smoke test sans code étudiant (joints figés à zéro) :

```bash
cd docker
docker compose run --rm viz bash -lc \
  "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && \
    colcon build --packages-select so101_description --symlink-install && \
    source install/setup.bash && \
    ros2 launch so101_description viz.launch.py demo:=true"
```

Comportement attendu sans `/joint_states` : bras "en morceaux" + erreurs
RobotModel — c'est le contrat du jalon S2 (le driver_node étudiant doit les
publier). Le RobotModel lit l'URDF → bras jaune (le recolorage gris n'existe
que dans le MJCF sim), sans conséquence.

| Symptôme | Cause | Fix |
|---|---|---|
| `could not connect to display` | xhost pas fait / DISPLAY vide | `xhost +local:docker` sur l'hôte, relancer |
| Fenêtre noire ou crash GL | pas d'accélération GPU dans le container | `LIBGL_ALWAYS_SOFTWARE=1 rviz2` (llvmpipe, suffisant) |
| VM très lente | 3D VM désactivée | activer la 3D, sinon fallback ci-dessus |

### 10.3 Docker — détail du lancement

Le service `control` est le seul à faire `colcon build` (un colcon
concurrent sur le même volume = conflits) ; `perception` et `viz` attendent
`install/`. `control` lance ensuite `driver.launch.py` puis
`brain.launch.py`. Le smoke test image dans Docker :

```bash
docker compose exec control \
  bash -lc "python3 /opt/so101/scripts/demo_sim.py /tmp/sortie.png"
```

Les artefacts `ros2_ws/{build,install,log}` sont créés **root** par le
container sur le volume monté → `sudo rm -rf` pour purger (gitignorés).
