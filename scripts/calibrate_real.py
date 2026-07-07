"""Calibration du bras réel (première utilisation ou après démontage).

Usage : python3 scripts/calibrate_real.py /dev/ttyACM0 mon_bras
Suit la procédure interactive LeRobot (positions min/max de chaque joint).
La calibration est stockée dans ~/.cache/tek5_robotics/calibration/.
"""
import sys

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
rid = sys.argv[2] if len(sys.argv) > 2 else "tek5_arm"

cfg = SO101FollowerConfig(port=port, id=rid, use_degrees=True)
robot = SO101Follower(cfg)
robot.connect(calibrate=True)
print("Calibration OK. Positions courantes :")
print({k: round(v, 1) for k, v in robot.get_observation().items() if k.endswith(".pos")})
robot.disconnect()
