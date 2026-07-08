# INSTRUCTION — Lancer et visualiser la simulation

Prérequis : **hôte Linux** (natif ou VM), Docker, un serveur X (X11 ou
Wayland/XWayland — les deux marchent). C'est un module de robotique :
l'outillage du métier (ROS2, RViz) tourne sous Linux, c'est assumé.

Deux environnements distincts, deux stacks de rendu :

|                    | **Local (hôte)**                    | **Docker (containers)**            |
|--------------------|-------------------------------------|-------------------------------------|
| Rendu MuJoCo       | GLFW/EGL natif (driver GPU)         | `MUJOCO_GL=osmesa` (offscreen soft) |
| Fenêtre interactive| ✅ (`view_live.py`, `mujoco.viewer`) | RViz via X11 forwardé (voir §5)     |
| Variables env      | **aucune**                          | déjà dans le Dockerfile             |
| Python             | venv `uv` (voir §1)                 | dans l'image                        |
| Usage              | instructeur : démos, debug modèle   | étudiants : pipeline complet        |

Règle simple : `MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa` c'est **Docker
uniquement**. En local, ne mets rien — MuJoCo prend ton GPU. Si tu forces
osmesa en local sans `libosmesa6` installé : crash cryptique
`'NoneType' ... glGetError`.

---

## 0. Démarrage express (3 terminaux)

Ouvre **3 terminaux séparés**. Les `docker compose exec` ouvrent un nouveau
shell dans un container déjà lancé — tu peux en ouvrir autant que tu veux
vers le même service.

**Terminal 1** — stack + contrôle du bras (rester dedans) :

```bash
cd docker
xhost +local:docker                          # une fois par session hôte
docker compose up -d control perception viz
docker compose exec control bash             # entre dans le container
```

Tu es maintenant dans le container `control`. Tape directement :

```bash
source /opt/ros/jazzy/setup.bash
python3 /opt/so101/scripts/jog.py status
python3 /opt/so101/scripts/jog.py shoulder_pan 30
```

Ce terminal reste ouvert pour toutes tes commandes `jog.py`.

**Terminal 2** — RViz (sur l'hôte, nouveau terminal) :

```bash
cd docker
docker compose exec control bash -lc "LIBGL_ALWAYS_SOFTWARE=1 && source /opt/ros/jazzy/setup.bash && rviz2"
```

La fenêtre RViz s'ouvre sur ton bureau. Configure une fois (voir §6), puis
laisse tourner pendant que tu tapes des commandes dans le Terminal 1 — le
bras doit bouger dans RViz.

**Terminal 3** — perception (sur l'hôte, nouveau terminal) :

```bash
cd docker
docker compose exec perception bash
```

Résumé : 1 terminal = 1 shell = soit hôte, soit dans un container.

---

## 1. Setup local (une fois)

```bash
uv venv ~/.venvs/tek5 && source ~/.venvs/tek5/bin/activate
uv pip install mujoco opencv-python
```

(`opencv-python` complet, pas headless : nécessaire pour les fenêtres de
`view_live.py --cam`.)

## 2. Local — smoke test image

```bash
python3 scripts/demo_sim.py sortie.png
```

Rend la caméra externe après 200 steps + imprime la vérité terrain de la
boule. Si une image sort, la sim est fonctionnelle.

## 3. Local — visu live interactive

### 3a. Demo - Bras animé, physique pilotée (recommandé)

```bash
python3 scripts/view_live.py --seconds 60
python3 scripts/view_live.py --cam    # + fenêtre OpenCV caméra externe
```

Viewer MuJoCo natif (orbite clic droit, zoom molette, double-clic sur un
corps + Ctrl+drag pour le perturber) pendant que `SO101Sim` exécute une
trajectoire de démo via la même API `send_action` que le bras réel. `--cam`
montre en parallèle exactement ce que verront les étudiants.

### 3b. Scène statique seule (inspection du modèle)

```bash
python3 -m mujoco.viewer --mjcf sim/so101_sim/assets/so101/scene_tek5.xml
```

Sliders des actuateurs dans le panneau de droite, mais aucun contrôleur.

### 3c. (optionnel) Tester le chemin de rendu Docker en local

```bash
sudo apt install libosmesa6
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python3 scripts/demo_sim.py sortie.png
```

Utile uniquement pour reproduire le comportement du container hors Docker.

---

## 4. Docker — smoke test image

```bash
cd docker
docker compose up -d control
docker compose exec control \
  bash -lc "cd /opt/so101 && python3 /ros2_ws/../scripts/demo_sim.py /tmp/sortie.png"
```

Les variables `MUJOCO_GL`/`PYOPENGL_PLATFORM` sont dans l'image, rien à
passer. (Adapter le chemin du script selon ton montage ; le point testé est
que le rendu OSMesa produit une image.)

## 5. Docker — visualisation RViz (chemin étudiant officiel)

RViz s'exécute **dans le container control** avec la fenêtre forwardée en
X11 vers l'hôte. Avantage décisif : RViz est un nœud ROS2 sur le réseau DDS
interne du compose — il voit tous les topics nativement, aucun bricolage
réseau. Le service `viz` fait tourner `robot_state_publisher` (URDF du
package fourni `so101_description`).

```bash
cd docker
xhost +local:docker        # une fois par session, sur l'hôte
docker compose up -d control viz
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && rviz2"
```

Configuration RViz (une fois, puis File → Save Config) :
1. **Global Options → Fixed Frame : `base_link`**
2. **Add → RobotModel**, puis dans ses propriétés :
   **Description Topic : `/robot_description`**
3. **Add → TF** (facultatif : réduire Marker Scale à ~0.2)
4. **Add → Image → Topic : `/camera/image_raw`** (une fois le driver_node actif)
5. La boule : **Add → Marker** sur le topic publié par votre perception.

Smoke test sans code étudiant (joints figés à zéro) :

```bash
cd docker
docker compose run --rm viz bash -lc \
  "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && \
    colcon build --packages-select so101_description --symlink-install && \
    source install/setup.bash && \
    ros2 launch so101_description viz.launch.py demo:=true"
```

**Comportement attendu** : sans nœud publiant `/joint_states`, le bras
apparaît "en morceaux" dans RViz (TF des joints inexistant, erreurs sur le
RobotModel). C'est le contrat : le `driver_node` étudiant publie
`/joint_states` depuis `get_observation()` — livrable du jalon S2. La boule
n'apparaît que si la perception la publie (Marker) : la visu montre ce que
le code étudiant voit, pas la vérité terrain.

### Dépannage affichage

| Symptôme | Cause | Fix |
|---|---|---|
| `could not connect to display` | xhost pas fait / DISPLAY vide | `xhost +local:docker` sur l'hôte, relancer |
| Fenêtre noire ou crash GL | pas d'accélération GPU dans le container | `LIBGL_ALWAYS_SOFTWARE=1 rviz2` (llvmpipe, suffisant pour ce projet) |
| VM très lente | accélération 3D VM désactivée | activer la 3D dans la VM, sinon fallback ci-dessus |

## 6. Foxglove (bonus, non supporté)

Foxglove Studio peut se connecter à un `foxglove_bridge` ajouté manuellement.
Non documenté, non supporté : des incompatibilités bridge/client ont été
constatées (bras "en morceaux" alors que les TF sont corrects). RViz est la
voie officielle.

---

## Récap

| Besoin                          | Commande                              | Où          |
|---------------------------------|---------------------------------------|-------------|
| Smoke test sim                  | `demo_sim.py` → PNG                   | local/Docker|
| Voir le robot bouger en live    | `view_live.py` (MuJoCo natif)         | local       |
| Inspecter le modèle MJCF        | `python3 -m mujoco.viewer`            | local       |
| Workflow étudiant (3D + image)  | `rviz2` dans control (X11)            | Docker      |
