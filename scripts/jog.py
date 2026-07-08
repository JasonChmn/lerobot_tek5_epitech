"""Jog moteur — tester le bras en simu OU en réel avec la même commande.

Simu (défaut) :
    python3 scripts/jog.py status
    python3 scripts/jog.py shoulder_pan 30
    python3 scripts/jog.py sweep shoulder_pan
    python3 scripts/jog.py zero

Réel (bras branché, calibré via calibrate_real.py) :
    python3 scripts/jog.py --real [--port /dev/ttyACM0] shoulder_pan 30
    python3 scripts/jog.py --real scan          # ping du bus Feetech (IDs moteurs)

Joints : shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll (degrés),
gripper (0-100). Même API des deux côtés — c'est le point du jumeau numérique.
"""
import argparse
import sys
import time

sys.path.insert(0, "sim")

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def get_robot(real: bool, port: str, rid: str):
    if real:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        r = SO101Follower(SO101FollowerConfig(port=port, id=rid, use_degrees=True))
    else:
        from so101_sim import SO101Sim
        r = SO101Sim(seed=0)
    r.connect()
    return r


def print_status(r):
    obs = r.get_observation()
    for j in JOINTS:
        print(f"  {j:15s} {obs[f'{j}.pos']:8.1f}")


def settle(r, action, steps=100):
    """Envoie la consigne et laisse converger (sim : steppe ; réel : attend)."""
    for _ in range(steps):
        r.send_action(action)
        time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", action="store_true")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="tek5_arm")
    ap.add_argument("cmd", help="status | zero | scan | sweep <joint> | <joint> <valeur>")
    ap.add_argument("value", nargs="?", help="consigne (degrés, gripper 0-100) ou joint pour sweep")
    args = ap.parse_args()

    if args.cmd == "scan":
        if not args.real:
            sys.exit("scan = bus série réel uniquement (--real)")
        from lerobot.motors.feetech import FeetechMotorsBus
        print(FeetechMotorsBus.scan_port(args.port))
        return

    r = get_robot(args.real, args.port, args.id)
    try:
        if args.cmd == "status":
            print_status(r)
        elif args.cmd == "zero":
            settle(r, {f"{j}.pos": 0.0 for j in JOINTS[:-1]} | {"gripper.pos": 50.0})
            print_status(r)
        elif args.cmd == "sweep":
            j = args.value if args.value in JOINTS else "shoulder_pan"
            hi, lo, home = (80.0, 20.0, 50.0) if j == "gripper" else (30.0, -30.0, 0.0)
            print(f"sweep {j} : {lo} → {hi} → {home}")
            for target in (hi, lo, home):
                settle(r, {f"{j}.pos": target})
                print(f"  consigne {target:6.1f} → lu {r.get_observation()[f'{j}.pos']:6.1f}")
        elif args.cmd in JOINTS:
            if args.value is None:
                sys.exit(f"usage : jog.py {args.cmd} <valeur>")
            val = float(args.value)
            settle(r, {f"{args.cmd}.pos": val})
            print(f"consigne {val:.1f} → lu {r.get_observation()[f'{args.cmd}.pos']:.1f}")
        else:
            sys.exit(f"commande inconnue : {args.cmd} (joints : {', '.join(JOINTS)})")
    finally:
        r.disconnect()


if __name__ == "__main__":
    main()
