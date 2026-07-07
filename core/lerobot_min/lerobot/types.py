# Version allégée de lerobot/types.py (Apache-2.0, HuggingFace Inc.)
# Purgée des types torch/policy inutiles pour le module Tek5.
from typing import Any

RobotAction = dict[str, Any]
RobotObservation = dict[str, Any]
