#!/usr/bin/env python3
"""decimate_meshes.py — décimation des STL de la sim MuJoCo (outil instructeur).

Pourquoi : le rendu offscreen dans Docker passe par OSMesa/llvmpipe (CPU).
Le vertex stage n'est pas parallélisé : 160K sommets ≈ 200 ms/frame, ce qui
affamait l'executor du driver (/joint_states à 5 Hz au lieu de 30).
Cible : ~10 % des sommets → rendu <40 ms/frame, visuellement équivalent.

Ne touche QUE sim/so101_sim/assets/so101/assets/*.stl (rendu MuJoCo).
Les meshes RViz (ros2_ws/src/so101_description/meshes) restent pleine
résolution : ils sont rendus par le GPU de l'hôte.

Usage :
    pip install open3d
    python3 scripts/decimate_meshes.py [--ratio 0.08] [--restore]

Note : open3d, pas trimesh+fast_simplification — ce dernier a un plancher
qualité qui refuse de descendre sous ~30 % des faces sur ces STL CAD.

Les originaux sont sauvegardés dans assets/original_stl/ au premier passage.
"""
import argparse
import shutil
from pathlib import Path

import open3d as o3d

ASSETS = Path(__file__).resolve().parent.parent / "sim/so101_sim/assets/so101/assets"
BACKUP = ASSETS / "original_stl"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ratio", type=float, default=0.08, help="fraction de faces conservées")
    ap.add_argument("--restore", action="store_true", help="restaurer les STL originaux")
    args = ap.parse_args()

    stls = sorted(ASSETS.glob("*.stl"))
    if not stls:
        raise SystemExit(f"Aucun STL dans {ASSETS}")

    if args.restore:
        for f in sorted(BACKUP.glob("*.stl")):
            shutil.copy2(f, ASSETS / f.name)
            print(f"restauré : {f.name}")
        return

    BACKUP.mkdir(exist_ok=True)
    total_before = total_after = 0
    print(f"{'fichier':42s} {'verts avant':>11s} {'verts après':>11s}")
    for f in stls:
        if not (BACKUP / f.name).exists():
            shutil.copy2(f, BACKUP / f.name)  # backup une seule fois

        m = o3d.io.read_triangle_mesh(str(BACKUP / f.name))
        m.remove_duplicated_vertices()
        v0 = len(m.vertices)
        target = max(int(len(m.triangles) * args.ratio), 60)
        m2 = m.simplify_quadric_decimation(target_number_of_triangles=target)
        m2.compute_vertex_normals()  # STL exige des normales
        o3d.io.write_triangle_mesh(str(f), m2)
        v1 = len(m2.vertices)
        total_before += v0
        total_after += v1
        print(f"{f.name:42s} {v0:11d} {v1:11d}")

    print(f"\nTotal : {total_before} -> {total_after} verts "
          f"({100 * total_after / total_before:.1f} %)")


if __name__ == "__main__":
    main()
