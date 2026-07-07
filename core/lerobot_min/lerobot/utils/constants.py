# Version allégée de lerobot/utils/constants.py (Apache-2.0, HuggingFace Inc.)
# Purgée des constantes dataset/policy/HF Hub. Calibration stockée en local.
import os
from pathlib import Path

OBS_STR = "observation"
OBS_STATE = OBS_STR + ".state"
OBS_IMAGE = OBS_STR + ".image"
OBS_IMAGES = OBS_IMAGE + "s"
ACTION = "action"

ROBOTS = "robots"
TELEOPERATORS = "teleoperators"

# Calibration : ~/.cache/tek5_robotics/calibration (surchargable par env var)
_default_home = Path.home() / ".cache" / "tek5_robotics"
HF_LEROBOT_HOME = Path(os.getenv("HF_LEROBOT_HOME", _default_home)).expanduser()
HF_LEROBOT_CALIBRATION = Path(
    os.getenv("HF_LEROBOT_CALIBRATION", HF_LEROBOT_HOME / "calibration")
).expanduser()
