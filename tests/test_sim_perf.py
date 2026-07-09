"""SO-ARM101 fix validation: test_observation_no_image (render removed from get_observation).

Run locally with the repo venv (uv), NOT in docker:
    cd sim/ && MUJOCO_GL=glfw uv run pytest ../tests/test_sim_perf.py -v

MUJOCO_GL must be set to a local GL backend (glfw/opengl) — do NOT set osmesa locally.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Ensure sim/ is importable from project root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sim"))

from so101_sim import SO101Sim

JNT_KEYS = {f"{j}.pos" for j in [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper"
]}


def test_observation_no_image():
    """La clé 'external_cam' ne doit plus apparaître dans l'observation."""
    s = SO101Sim(seed=0)
    s.connect()
    try:
        obs = s.get_observation()
        assert "external_cam" not in obs
        assert set(obs.keys()) == JNT_KEYS
    finally:
        s.disconnect()


def test_observation_fast():
    """100 appels get_observation() : moyenne < 5 ms (était ~170 ms avec le render)."""
    s = SO101Sim(seed=0)
    s.connect()
    try:
        times = []
        for _ in range(100):
            t0 = __import__("time").perf_counter()
            s.get_observation()
            times.append((__import__("time").perf_counter() - t0) * 1000)
        mean_ms = np.mean(times)
        print(f"\n  test_observation_fast: mean={mean_ms:.2f} ms (target < 5 ms)")
        assert mean_ms < 5.0, f"Mean observation time {mean_ms:.2f} ms exceeds 5 ms limit"
    finally:
        s.disconnect()


def test_render_still_works():
    """render_camera() retourne bien une image RGB (240, 320, 3) uint8."""
    s = SO101Sim(seed=0)
    s.connect()
    try:
        img = s.render_camera()
        assert img.shape == (240, 320, 3)
        assert img.dtype == np.uint8
    finally:
        s.disconnect()


def test_command_convergence():
    """Envoyer shoulder_pan.pos=30.0 pendant 150 itérations → convergence < 1.0."""
    s = SO101Sim(seed=0)
    s.connect()
    try:
        for _ in range(150):
            s.send_action({"shoulder_pan.pos": 30.0})
        obs = s.get_observation()
        diff = abs(obs["shoulder_pan.pos"] - 30.0)
        print(f"\n  test_command_convergence: final={obs['shoulder_pan.pos']:.2f}, error={diff:.2f}")
        assert diff < 1.0, f"Convergence failed: error={diff:.2f} >= 1.0"
    finally:
        s.disconnect()


def test_hsv_detection_at_320x240():
    """Render + détection HSV : au moins un contour > 25 px (balle visible à basse rés)."""
    import cv2

    HSV_MIN = np.array([15, 80, 150])
    HSV_MAX = np.array([35, 255, 255])

    s = SO101Sim(seed=0)
    s.connect()
    try:
        img = s.render_camera()  # RGB
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = max((cv2.contourArea(c) for c in contours), default=0)
        n_contours = len([c for c in contours if cv2.contourArea(c) > 25])

        print(f"\n  test_hsv_detection: contours>25px={n_contours}, max_area={max_area:.1f}")
        assert max_area > 25, (
            f"Ball not visible (max_area={max_area:.1f} <= 25). "
            "Ball may be out of frame for seed=0."
        )
    except AssertionError:
        s.disconnect()
        # Retry with seed=3
        s = SO101Sim(seed=3)
        s.connect()
        try:
            img = s.render_camera()
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_area = max((cv2.contourArea(c) for c in contours), default=0)
            n_contours = len([c for c in contours if cv2.contourArea(c) > 25])

            print(f"  test_hsv_detection (seed=3): contours>25px={n_contours}, max_area={max_area:.1f}")
            assert max_area > 25, (
                f"Ball not visible with seed=3 either (max_area={max_area:.1f} <= 25). "
                "Resolution too low or ball position unreachable."
            )
        finally:
            s.disconnect()
